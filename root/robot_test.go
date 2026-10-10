package root

import (
	"context"
	"encoding/binary"
	"errors"
	"io"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/koeng101/rootgo/packet"
)

// fakeTransport is an in-memory Transport. Frames the Robot writes appear on
// sent; frames pushed with reply/event are read by the Robot.
type fakeTransport struct {
	in        chan []byte
	sent      chan packet.Packet
	closeOnce sync.Once
	closed    chan struct{}
}

func newFake() *fakeTransport {
	return &fakeTransport{
		in:     make(chan []byte, 64),
		sent:   make(chan packet.Packet, 1024),
		closed: make(chan struct{}),
	}
}

func (f *fakeTransport) ReadFrame() ([]byte, error) {
	select {
	case b := <-f.in:
		return b, nil
	case <-f.closed:
		return nil, io.EOF
	}
}

func (f *fakeTransport) WriteFrame(b []byte) error {
	select {
	case <-f.closed:
		return io.ErrClosedPipe
	default:
	}
	p, err := packet.Parse(b)
	if err != nil {
		return err
	}
	f.sent <- p
	return nil
}

func (f *fakeTransport) Close() error {
	f.closeOnce.Do(func() { close(f.closed) })
	return nil
}

func (f *fakeTransport) push(p packet.Packet) {
	b := p.Bytes()
	f.in <- b[:]
}

// next returns the next frame the Robot wrote.
func (f *fakeTransport) next(t *testing.T) packet.Packet {
	t.Helper()
	select {
	case p := <-f.sent:
		return p
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for robot to send a packet")
		return packet.Packet{}
	}
}

// reply answers req with the given payload, as the robot would.
func (f *fakeTransport) reply(req packet.Packet, payload []byte) {
	f.push(packet.New(req.Dev, req.Cmd, req.ID, payload))
}

func setup(t *testing.T) (*Robot, *fakeTransport) {
	t.Helper()
	f := newFake()
	r := New(f)
	r.proto, r.protoKnown = [2]uint8{1, 3}, true
	t.Cleanup(func() { r.Close() })
	return r, f
}

func posePayload(ts uint32, x, y int32, heading int16) []byte {
	b := make([]byte, 14)
	binary.BigEndian.PutUint32(b, ts)
	binary.BigEndian.PutUint32(b[4:], uint32(x))
	binary.BigEndian.PutUint32(b[8:], uint32(y))
	binary.BigEndian.PutUint16(b[12:], uint16(heading))
	return b
}

type ret[T any] struct {
	v   T
	err error
}

func async[T any](fn func() (T, error)) <-chan ret[T] {
	ch := make(chan ret[T], 1)
	go func() {
		v, err := fn()
		ch <- ret[T]{v, err}
	}()
	return ch
}

func wait[T any](t *testing.T, ch <-chan ret[T]) (T, error) {
	t.Helper()
	select {
	case r := <-ch:
		return r.v, r.err
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for call to return")
		var zero T
		return zero, nil
	}
}

func TestQueryReply(t *testing.T) {
	r, f := setup(t)
	ch := async(func() (string, error) { return r.Name(context.Background()) })
	req := f.next(t)
	if req.Dev != 0 || req.Cmd != 2 {
		t.Fatalf("sent %v, want get name", req)
	}
	f.reply(req, []byte("Rooty\x00"))
	name, err := wait(t, ch)
	if err != nil || name != "Rooty" {
		t.Fatalf("Name() = %q, %v", name, err)
	}
}

func TestIDsIncrementAndWrap(t *testing.T) {
	r, f := setup(t)
	ctx := context.Background()
	for i := range 300 {
		if err := r.SetLights(ctx, LightsOn, 1, 2, 3); err != nil {
			t.Fatal(err)
		}
		if p := f.next(t); p.ID != uint8(i) {
			t.Fatalf("packet %d has ID %d", i, p.ID)
		}
	}
}

func TestDriveReturnsPose(t *testing.T) {
	r, f := setup(t)
	ch := async(func() (Pose, error) { return r.Drive(context.Background(), -150) })
	req := f.next(t)
	if req.Dev != 1 || req.Cmd != 8 || int32(binary.BigEndian.Uint32(req.Payload[:])) != -150 {
		t.Fatalf("sent %v, want drive -150", req)
	}
	f.reply(req, posePayload(1234, -5, -150, 900))
	pose, err := wait(t, ch)
	want := Pose{Timestamp: 1234 * time.Millisecond, X: -5, Y: -150, Heading: 90}
	if err != nil || pose != want {
		t.Fatalf("Drive() = %+v, %v; want %+v", pose, err, want)
	}
}

func TestCancelledMotionStopsWheels(t *testing.T) {
	r, f := setup(t)
	ctx, cancel := context.WithCancel(context.Background())
	ch := async(func() (Pose, error) { return r.Drive(ctx, 1000) })
	req := f.next(t)
	cancel()
	if _, err := wait(t, ch); !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v, want context.Canceled", err)
	}
	stop := f.next(t)
	if stop.Dev != 1 || stop.Cmd != 4 || stop.Payload != [16]byte{} {
		t.Fatalf("sent %v, want wheel speeds 0,0", stop)
	}
	// A late finished reply is ignored and the slot is free again.
	f.reply(req, posePayload(0, 0, 0, 0))
	ch = async(func() (Pose, error) { return r.Drive(context.Background(), 10) })
	f.reply(f.next(t), posePayload(0, 0, 10, 900))
	if _, err := wait(t, ch); err != nil {
		t.Fatal(err)
	}
}

func TestCancelledSayStopsSound(t *testing.T) {
	r, f := setup(t)
	ctx, cancel := context.WithCancel(context.Background())
	ch := async(func() (struct{}, error) { return struct{}{}, r.Say(ctx, "hello") })
	f.next(t)
	cancel()
	if _, err := wait(t, ch); !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v", err)
	}
	if p := f.next(t); p.Dev != 5 || p.Cmd != 1 {
		t.Fatalf("sent %v, want stop sound", p)
	}
}

func TestSecondMotionIsBusy(t *testing.T) {
	r, f := setup(t)
	ch := async(func() (Pose, error) { return r.Drive(context.Background(), 100) })
	req := f.next(t)
	if _, err := r.Rotate(context.Background(), 90); !errors.Is(err, ErrBusy) {
		t.Fatalf("Rotate during Drive: err = %v, want ErrBusy", err)
	}
	// Other kinds of action are independent of motion.
	sound := async(func() (struct{}, error) { return struct{}{}, r.PlayNote(context.Background(), 440, time.Second) })
	note := f.next(t)
	f.reply(note, nil)
	f.reply(req, posePayload(0, 0, 100, 900))
	if _, err := wait(t, ch); err != nil {
		t.Fatal(err)
	}
	if _, err := wait(t, sound); err != nil {
		t.Fatal(err)
	}
}

func TestQueriesDuringMotionRepliedOutOfOrder(t *testing.T) {
	r, f := setup(t)
	drive := async(func() (Pose, error) { return r.Drive(context.Background(), 100) })
	driveReq := f.next(t)
	batt := async(func() (Battery, error) { return r.Battery(context.Background()) })
	battReq := f.next(t)

	f.reply(battReq, []byte{0, 0, 0, 1, 0x0F, 0xA0, 87})
	b, err := wait(t, batt)
	if err != nil || b.MilliVolts != 4000 || b.Percent != 87 {
		t.Fatalf("Battery() = %+v, %v", b, err)
	}
	f.reply(driveReq, posePayload(0, 0, 100, 900))
	if _, err := wait(t, drive); err != nil {
		t.Fatal(err)
	}
}

func TestStopButtonFailsActionsNotQueries(t *testing.T) {
	r, f := setup(t)
	drive := async(func() (Pose, error) { return r.Drive(context.Background(), 1000) })
	f.next(t)
	pos := async(func() (Pose, error) { return r.Position(context.Background()) })
	posReq := f.next(t)

	f.push(packet.New(0, 4, 0, nil)) // nose button
	if _, err := wait(t, drive); !errors.Is(err, ErrStopped) {
		t.Fatalf("Drive err = %v, want ErrStopped", err)
	}
	f.reply(posReq, posePayload(0, 1, 2, 900))
	if p, err := wait(t, pos); err != nil || p.X != 1 {
		t.Fatalf("Position() = %+v, %v", p, err)
	}
}

func TestStopFailsActions(t *testing.T) {
	r, f := setup(t)
	drive := async(func() (Pose, error) { return r.Drive(context.Background(), 1000) })
	f.next(t)
	if err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if p := f.next(t); p.Dev != 0 || p.Cmd != 3 {
		t.Fatalf("sent %v, want stop and reset", p)
	}
	if _, err := wait(t, drive); !errors.Is(err, ErrStopped) {
		t.Fatalf("Drive err = %v, want ErrStopped", err)
	}
}

func TestCloseFailsPendingAndSubscriptions(t *testing.T) {
	r, f := setup(t)
	sub := r.Subscribe(1)
	q := async(func() (string, error) { return r.Name(context.Background()) })
	f.next(t)
	r.Close()
	if _, err := wait(t, q); !errors.Is(err, ErrDisconnected) {
		t.Fatalf("err = %v, want ErrDisconnected", err)
	}
	if _, ok := <-sub.C; ok {
		t.Fatal("subscription channel still open")
	}
	if _, err := r.Name(context.Background()); !errors.Is(err, ErrDisconnected) {
		t.Fatalf("after close: err = %v", err)
	}
	if _, ok := <-r.Subscribe(1).C; ok {
		t.Fatal("subscription after close is open")
	}
}

func TestQueryDefaultTimeout(t *testing.T) {
	if testing.Short() {
		t.Skip("waits for DefaultQueryTimeout")
	}
	r, f := setup(t)
	ch := async(func() (string, error) { return r.Name(context.Background()) })
	f.next(t)
	select {
	case res := <-ch:
		if !errors.Is(res.err, context.DeadlineExceeded) {
			t.Fatalf("err = %v", res.err)
		}
	case <-time.After(DefaultQueryTimeout + time.Second):
		t.Fatal("query did not time out")
	}
}

func TestIDCollision(t *testing.T) {
	r, f := setup(t)
	pos := async(func() (Pose, error) { return r.Position(context.Background()) })
	posReq := f.next(t)
	for range 255 {
		r.SetLights(context.Background(), LightsOff, 0, 0, 0)
		f.next(t)
	}
	// The ID has wrapped onto the still-pending Position request.
	if _, err := r.Position(context.Background()); !errors.Is(err, ErrIDInUse) {
		t.Fatalf("err = %v, want ErrIDInUse", err)
	}
	f.reply(posReq, posePayload(0, 0, 0, 0))
	if _, err := wait(t, pos); err != nil {
		t.Fatal(err)
	}
}

func TestEventsFanOutAndState(t *testing.T) {
	r, f := setup(t)
	a, b := r.Subscribe(4), r.Subscribe(1)
	defer a.Close()

	f.push(packet.New(12, 0, 0, []byte{0, 0, 0, 7, 0x80}))                   // left bumper
	f.push(packet.New(17, 0, 1, []byte{0, 0, 0, 8, 0xA0}))                   // FL + RR touch
	f.push(packet.New(20, 0, 2, []byte{0, 0, 0, 9, 1, 0x01, 0x00, 0, 0xC8})) // cliff

	want := []Event{
		Bumpers{7 * time.Millisecond, true, false},
		Touch{8 * time.Millisecond, true, false, true, false},
		Cliff{9 * time.Millisecond, true, 256, 200},
	}
	for i, w := range want {
		select {
		case got := <-a.C:
			if got != w {
				t.Fatalf("event %d = %#v, want %#v", i, got, w)
			}
		case <-time.After(2 * time.Second):
			t.Fatal("timed out waiting for event")
		}
	}
	// b had room for one event and dropped the rest without blocking a.
	if got := <-b.C; got != want[0] {
		t.Fatalf("b got %#v", got)
	}
	if b.Dropped() != 2 {
		t.Fatalf("b dropped %d, want 2", b.Dropped())
	}
	b.Close()
	b.Close()

	s := r.State()
	if s.Bumpers == nil || !s.Bumpers.Left || s.Cliff == nil || !s.Cliff.Cliff || s.Battery != nil {
		t.Fatalf("State() = %+v", s)
	}
}

func TestColorEvent(t *testing.T) {
	p := packet.New(4, 2, 0, []byte{0x01, 0x23, 0x4E})
	c, ok := decodeEvent(p).(Colors)
	if !ok {
		t.Fatal("not a Colors event")
	}
	want := []ColorID{ColorWhite, ColorBlack, ColorRed, ColorGreen, ColorBlue, ColorLowSaturation}
	for i, w := range want {
		if c[i] != w {
			t.Fatalf("color %d = %d, want %d", i, c[i], w)
		}
	}
}

func TestBadFramesIgnored(t *testing.T) {
	r, f := setup(t)
	bad := packet.New(12, 0, 0, nil).Bytes()
	bad[19] ^= 0xFF
	f.in <- bad[:]
	f.in <- []byte{1, 2, 3}
	ch := async(func() (string, error) { return r.Name(context.Background()) })
	f.reply(f.next(t), []byte("ok"))
	if _, err := wait(t, ch); err != nil {
		t.Fatal(err)
	}
	if n := r.BadFrames(); n != 2 {
		t.Fatalf("BadFrames = %d, want 2", n)
	}
}

func TestSayChunksOnRuneBoundaries(t *testing.T) {
	r, f := setup(t)
	phrase := "hello robot, ünïcödé is fine"
	ch := async(func() (struct{}, error) { return struct{}{}, r.Say(context.Background(), phrase) })
	var got string
	for got != phrase {
		req := f.next(t)
		if req.Dev != 5 || req.Cmd != 4 {
			t.Fatalf("sent %v", req)
		}
		got += cString(req.Payload[:])
		f.reply(req, nil)
	}
	if _, err := wait(t, ch); err != nil {
		t.Fatal(err)
	}
}

func TestEncodings(t *testing.T) {
	r, f := setup(t)
	ctx := context.Background()

	r.SetWheelSpeeds(ctx, -100, 100)
	if p := f.next(t); p.Payload != [16]byte{0xFF, 0xFF, 0xFF, 0x9C, 0, 0, 0, 0x64} {
		t.Errorf("SetWheelSpeeds payload % X", p.Payload)
	}
	go r.Rotate(ctx, -90.5)
	p := f.next(t)
	if p.Cmd != 12 || int32(binary.BigEndian.Uint32(p.Payload[:])) != -905 {
		t.Errorf("Rotate payload % X", p.Payload)
	}
	f.reply(p, nil)
	r.SetLights(ctx, LightsSpin, 10, 20, 30)
	if p := f.next(t); p.Dev != 3 || p.Cmd != 2 || p.Payload != [16]byte{3, 10, 20, 30} {
		t.Errorf("SetLights sent %v", p)
	}
	if err := r.SetName(ctx, "this name is too long"); err == nil {
		t.Error("SetName accepted a 21-byte name")
	}
}

func TestLegacyProtocolPose(t *testing.T) {
	f := newFake()
	r := New(f)
	defer r.Close()

	ch := async(func() (Pose, error) { return r.Drive(context.Background(), 100) })
	drive := f.next(t)
	f.reply(drive, nil) // protocol 1.2: finished reply has no pose

	ver := f.next(t)
	if ver.Dev != 0 || ver.Cmd != 0 || ver.Payload[0] != byte(BoardMain) {
		t.Fatalf("sent %v, want get versions", ver)
	}
	f.reply(ver, []byte{0xA5, 2, 1, 1, 1, 1, 0, 1, 2})

	pos := f.next(t)
	if pos.Dev != 1 || pos.Cmd != 16 {
		t.Fatalf("sent %v, want get position", pos)
	}
	// Observed on firmware 2.1: ts, then int16 X, Y, heading.
	f.reply(pos, []byte{0x00, 0x01, 0x8c, 0x16, 0x00, 0x3a, 0xff, 0x98, 0x03, 0x5e})
	got, err := wait(t, ch)
	if err != nil {
		t.Fatal(err)
	}
	// Raw heading 86.2° clockwise from the start is 3.8° in 1.3 convention.
	if got.Timestamp != 101398*time.Millisecond || got.X != 58 || got.Y != -104 || math.Abs(got.Heading-3.8) > 1e-9 {
		t.Fatalf("pose = %+v", got)
	}

	// The version is cached.
	ch = async(func() (Pose, error) { return r.Position(context.Background()) })
	f.reply(f.next(t), []byte{0, 0, 0, 0, 0, 1, 0, 2})
	if got, err := wait(t, ch); err != nil || got.X != 1 || got.Y != 2 {
		t.Fatalf("Position() = %+v, %v", got, err)
	}
}

func TestLegacyHeading(t *testing.T) {
	for _, tc := range []struct {
		raw  int16
		want float64
	}{
		{0, 90},      // at reset, facing +Y
		{900, 0},     // turned 90° clockwise, facing +X
		{1800, 270},  // facing -Y
		{2700, 180},  // facing -X
		{3595, 90.5}, // just short of a full clockwise turn
	} {
		if got := legacyHeading(tc.raw); math.Abs(got-tc.want) > 1e-9 {
			t.Errorf("legacyHeading(%d) = %v, want %v", tc.raw, got, tc.want)
		}
	}
}

func TestNameAfterSerial(t *testing.T) {
	r, f := setup(t)
	ch := async(func() (string, error) { return r.NameAfterSerial(context.Background()) })
	req := f.next(t)
	if req.Dev != 0 || req.Cmd != 14 {
		t.Fatalf("sent %v, want get serial number", req)
	}
	f.reply(req, []byte("RT1908F04403"))
	set := f.next(t)
	if set.Dev != 0 || set.Cmd != 1 || cString(set.Payload[:]) != "RT1908F04403" {
		t.Fatalf("sent %v, want set name to the serial", set)
	}
	if name, err := wait(t, ch); err != nil || name != "RT1908F04403" {
		t.Fatalf("NameAfterSerial() = %q, %v", name, err)
	}
}
