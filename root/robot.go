// Package root controls an iRobot Root rt1 robot.
//
// A Robot owns one goroutine that reads frames from its Transport. Replies
// are routed to the call waiting for them by (device, command, packet ID);
// everything else is decoded as an Event, recorded in the cached State and
// fanned out to subscribers.
//
// Methods block until the robot replies and take a context. Cancelling the
// context of a motion command stops the wheels; cancelling a sound command
// stops the sound.
package root

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/koeng101/rootgo/packet"
)

var (
	// ErrDisconnected is returned once the transport fails or the Robot is closed.
	ErrDisconnected = errors.New("root: disconnected")
	// ErrStopped is returned by actions that were cancelled by Stop or by
	// the robot's nose button.
	ErrStopped = errors.New("root: robot stopped")
	// ErrBusy is returned when an action is started while another action
	// of the same kind (motion, sound or marker) is still running.
	ErrBusy = errors.New("root: another action of this kind is running")
	// ErrIDInUse is returned when the packet ID wrapped around onto an
	// identical request that is still waiting for its reply.
	ErrIDInUse = errors.New("root: packet ID still in use")
)

// DefaultQueryTimeout bounds queries (and their replies) whose context has no
// deadline, so a lost reply cannot block forever. Actions are not bounded.
const DefaultQueryTimeout = 3 * time.Second

// Transport carries 20-byte frames to and from a robot.
type Transport interface {
	// ReadFrame blocks until the robot sends a frame. After Close it must
	// return an error.
	ReadFrame() ([]byte, error)
	// WriteFrame sends one frame, returning once the robot has accepted it.
	WriteFrame([]byte) error
	Close() error
}

// slot groups actions that the robot can only run one at a time.
type slot uint8

const (
	slotNone slot = iota // queries: unlimited concurrency
	slotMotion
	slotSound
	slotMarker
	numSlots
)

type key struct{ dev, cmd, id uint8 }

type result struct {
	p   packet.Packet
	err error
}

type call struct {
	slot slot
	done chan result // buffered 1; written once, by whoever removes the call from pending
}

// Robot is a connection to one robot. It is safe for concurrent use.
type Robot struct {
	t Transport

	wmu    sync.Mutex // serializes writes so packet IDs reach the robot in order
	nextID uint8

	mu      sync.Mutex
	pending map[key]*call
	active  [numSlots]bool
	subs    map[*Subscription]struct{}
	state   State
	closed  bool
	err     error

	protoMu    sync.Mutex
	protoKnown bool
	proto      [2]uint8 // major, minor

	done       chan struct{} // closed when the reader exits
	badFrames  atomic.Uint64
	trace      atomic.Pointer[func(sent bool, p packet.Packet)]
	unexpected atomic.Uint64
}

// New starts a Robot on an already connected transport.
func New(t Transport) *Robot {
	r := &Robot{
		t:       t,
		pending: make(map[key]*call),
		subs:    make(map[*Subscription]struct{}),
		done:    make(chan struct{}),
	}
	go r.readLoop()
	return r
}

// Close closes the transport and waits for the reader to exit. Pending calls
// fail with ErrDisconnected and subscriptions are closed.
func (r *Robot) Close() error {
	err := r.t.Close()
	<-r.done
	return err
}

// Done is closed once the robot is disconnected; Err then reports why.
func (r *Robot) Done() <-chan struct{} { return r.done }

// Err returns the reason the robot disconnected, or nil while connected.
func (r *Robot) Err() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.err
}

// Trace calls fn with every packet sent (sent is true) or received, for
// debugging. fn runs on the sending or reading goroutine and must not block.
// A nil fn turns tracing off.
func (r *Robot) Trace(fn func(sent bool, p packet.Packet)) {
	if fn == nil {
		r.trace.Store(nil)
	} else {
		r.trace.Store(&fn)
	}
}

// BadFrames counts received frames dropped for a bad length or CRC.
func (r *Robot) BadFrames() uint64 { return r.badFrames.Load() }

func (r *Robot) readLoop() {
	defer close(r.done)
	for {
		b, err := r.t.ReadFrame()
		if err != nil {
			r.shutdown(fmt.Errorf("%w: %v", ErrDisconnected, err))
			return
		}
		p, err := packet.Parse(b)
		if err != nil {
			r.badFrames.Add(1)
			continue
		}
		if fn := r.trace.Load(); fn != nil {
			(*fn)(false, p)
		}
		r.dispatch(p)
	}
}

func (r *Robot) dispatch(p packet.Packet) {
	r.mu.Lock()
	k := key{p.Dev, p.Cmd, p.ID}
	if c, ok := r.pending[k]; ok {
		delete(r.pending, k)
		r.mu.Unlock()
		c.done <- result{p: p}
		return
	}
	ev := decodeEvent(p)
	if ev == nil {
		// Unknown packet, or a late reply to a call that was cancelled.
		r.unexpected.Add(1)
		r.mu.Unlock()
		return
	}
	r.state.update(ev)
	if _, ok := ev.(StopButton); ok {
		r.failActionsLocked(ErrStopped)
	}
	for s := range r.subs {
		s.deliver(ev)
	}
	r.mu.Unlock()
}

func (r *Robot) shutdown(err error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.closed = true
	r.err = err
	for k, c := range r.pending {
		delete(r.pending, k)
		c.done <- result{err: err}
	}
	for s := range r.subs {
		delete(r.subs, s)
		close(s.c)
	}
}

// failActionsLocked fails every waiting action (but not queries). The robot
// drops pending actions when stopped, so their replies would never arrive.
func (r *Robot) failActionsLocked(err error) {
	for k, c := range r.pending {
		if c.slot != slotNone {
			delete(r.pending, k)
			c.done <- result{err: err}
		}
	}
}

// send writes one packet. When c is non-nil it is registered to receive the
// reply before the write, so a fast reply cannot be missed.
func (r *Robot) send(dev, cmd uint8, payload []byte, c *call) (key, error) {
	r.wmu.Lock()
	defer r.wmu.Unlock()

	k := key{dev, cmd, r.nextID}
	r.mu.Lock()
	if r.closed {
		err := r.err
		r.mu.Unlock()
		return k, err
	}
	if c != nil {
		if _, ok := r.pending[k]; ok {
			r.mu.Unlock()
			return k, ErrIDInUse
		}
		r.pending[k] = c
	}
	r.mu.Unlock()

	// IDs advance even if the write fails: the robot may have received it.
	r.nextID++
	p := packet.New(dev, cmd, k.id, payload)
	if fn := r.trace.Load(); fn != nil {
		(*fn)(true, p)
	}
	b := p.Bytes()
	if err := r.t.WriteFrame(b[:]); err != nil {
		if c != nil {
			r.forget(k, c)
		}
		return k, fmt.Errorf("root: write: %w", err)
	}
	return k, nil
}

// forget removes c from pending if it is still there.
func (r *Robot) forget(k key, c *call) {
	r.mu.Lock()
	if r.pending[k] == c {
		delete(r.pending, k)
	}
	r.mu.Unlock()
}

// command sends a packet that has no reply.
func (r *Robot) command(ctx context.Context, dev, cmd uint8, payload []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	_, err := r.send(dev, cmd, payload, nil)
	return err
}

// query sends a packet and waits for its reply.
func (r *Robot) query(ctx context.Context, dev, cmd uint8, payload []byte) (packet.Packet, error) {
	if _, ok := ctx.Deadline(); !ok {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, DefaultQueryTimeout)
		defer cancel()
	}
	return r.request(ctx, slotNone, dev, cmd, payload)
}

// action sends a packet that starts a robot action and waits for its
// "finished" reply. Only one action per slot may run at a time.
func (r *Robot) action(ctx context.Context, s slot, dev, cmd uint8, payload []byte) (packet.Packet, error) {
	return r.request(ctx, s, dev, cmd, payload)
}

func (r *Robot) request(ctx context.Context, s slot, dev, cmd uint8, payload []byte) (packet.Packet, error) {
	if s != slotNone {
		release, err := r.acquire(s)
		if err != nil {
			return packet.Packet{}, err
		}
		defer release()
	}
	return r.roundTrip(ctx, s, dev, cmd, payload)
}

// roundTrip sends one packet and waits for its reply. For actions, the
// caller must already hold slot s.
func (r *Robot) roundTrip(ctx context.Context, s slot, dev, cmd uint8, payload []byte) (packet.Packet, error) {
	if err := ctx.Err(); err != nil {
		return packet.Packet{}, err
	}
	c := &call{slot: s, done: make(chan result, 1)}
	k, err := r.send(dev, cmd, payload, c)
	if err != nil {
		return packet.Packet{}, err
	}
	select {
	case res := <-c.done:
		return res.p, res.err
	case <-ctx.Done():
	}
	// Prefer a reply that raced with cancellation.
	select {
	case res := <-c.done:
		return res.p, res.err
	default:
	}
	r.forget(k, c)
	r.abort(s)
	return packet.Packet{}, ctx.Err()
}

// acquire claims slot s for one action, which may span several packets.
func (r *Robot) acquire(s slot) (release func(), err error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed {
		return nil, r.err
	}
	if r.active[s] {
		return nil, ErrBusy
	}
	r.active[s] = true
	return func() {
		r.mu.Lock()
		r.active[s] = false
		r.mu.Unlock()
	}, nil
}

// abort halts the robot-side effect of a cancelled action. It is best
// effort: the caller's context is already done and the error is not useful
// to report over the cancellation itself.
func (r *Robot) abort(s slot) {
	switch s {
	case slotMotion:
		r.send(devMotors, cmdSetSpeeds, make([]byte, 8), nil)
	case slotSound:
		r.send(devSound, cmdStopSound, nil, nil)
	}
}
