package root

import (
	"encoding/binary"
	"sync/atomic"
	"time"

	"github.com/koeng101/rootgo/packet"
)

// Event is a packet the robot sends on its own: one of StopButton,
// MotorStall, Colors, Bumpers, Light, Battery, Touch or Cliff.
type Event interface{ event() }

// StopButton is sent when the nose button is pressed. The robot stops and
// cancels its pending actions, which then fail with ErrStopped.
type StopButton struct{}

// Motor identifies a motor in a MotorStall event.
type Motor uint8

const (
	MotorLeft Motor = iota
	MotorRight
	MotorMarker
)

// StallCause is why a motor stalled.
type StallCause uint8

const (
	StallNone StallCause = iota
	StallOverCurrent
	StallUnderCurrent
	StallUnderSpeed
	StallSaturatedPID
	StallTimeout
)

// MotorStall is sent when a motor stalls.
type MotorStall struct {
	Timestamp time.Duration // since robot power-on
	Motor     Motor
	Cause     StallCause
}

// ColorID is a color identified by the color sensor. The protocol
// documents only white, black, red, green and blue; the rest are values the
// robot sends in practice, named as in iRobot's Python SDK.
type ColorID uint8

const (
	ColorWhite         ColorID = 0
	ColorBlack         ColorID = 1
	ColorRed           ColorID = 2
	ColorGreen         ColorID = 3
	ColorBlue          ColorID = 4
	ColorOrange        ColorID = 5
	ColorYellow        ColorID = 6
	ColorMagenta       ColorID = 7
	ColorLime          ColorID = 8
	ColorCyan          ColorID = 9
	ColorViolet        ColorID = 10
	ColorCerise        ColorID = 11
	ColorLowIntensity  ColorID = 13
	ColorLowSaturation ColorID = 14 // e.g. a gray floor
)

// Colors is sent when the color identified under any of the 32 color
// sensors changes. Index 0 is the leftmost sensor.
type Colors [32]ColorID

// Bumpers is sent when a bumper is pressed or released.
type Bumpers struct {
	Timestamp   time.Duration
	Left, Right bool
}

// LightState summarizes the ambient light seen by the two eyes.
type LightState uint8

const (
	LightBothDark      LightState = 4
	LightRightBrighter LightState = 5
	LightLeftBrighter  LightState = 6
	LightBothBright    LightState = 7
)

// Light reports ambient light. Left and Right are in decipercent (0-1000).
// State is only set on events, not on LightValues replies.
type Light struct {
	Timestamp   time.Duration
	State       LightState
	Left, Right uint16
}

// Battery reports the battery level.
type Battery struct {
	Timestamp  time.Duration
	MilliVolts uint16
	Percent    uint8
}

// Touch reports the four top touch sensors.
type Touch struct {
	Timestamp                                  time.Duration
	FrontLeft, FrontRight, RearRight, RearLeft bool
}

// Cliff is sent when the front cliff sensor starts or stops seeing a cliff.
type Cliff struct {
	Timestamp   time.Duration
	Cliff       bool
	SensorMV    uint16
	ThresholdMV uint16
}

func (StopButton) event() {}
func (MotorStall) event() {}
func (Colors) event()     {}
func (Bumpers) event()    {}
func (Light) event()      {}
func (Battery) event()    {}
func (Touch) event()      {}
func (Cliff) event()      {}

func timestamp(b []byte) time.Duration {
	return time.Duration(binary.BigEndian.Uint32(b)) * time.Millisecond
}

// decodeEvent returns the event in p, or nil if p is not an event.
func decodeEvent(p packet.Packet) Event {
	b := p.Payload[:]
	switch {
	case p.Dev == devGeneral && p.Cmd == 4:
		return StopButton{}
	case p.Dev == devMotors && p.Cmd == 29:
		return MotorStall{timestamp(b), Motor(b[4]), StallCause(b[5])}
	case p.Dev == devColor && p.Cmd == 2:
		var c Colors
		for i := range c {
			nib := b[i/2] >> 4
			if i%2 == 1 {
				nib = b[i/2] & 0x0F
			}
			c[i] = ColorID(nib)
		}
		return c
	case p.Dev == devBumpers && p.Cmd == 0:
		return Bumpers{timestamp(b), b[4]&0x80 != 0, b[4]&0x40 != 0}
	case p.Dev == devLight && p.Cmd == 0:
		return Light{timestamp(b), LightState(b[4]), binary.BigEndian.Uint16(b[5:]), binary.BigEndian.Uint16(b[7:])}
	case p.Dev == devBattery && p.Cmd == 0:
		return decodeBattery(b)
	case p.Dev == devTouch && p.Cmd == 0:
		return Touch{timestamp(b), b[4]&0x80 != 0, b[4]&0x40 != 0, b[4]&0x20 != 0, b[4]&0x10 != 0}
	case p.Dev == devCliff && p.Cmd == 0:
		return Cliff{timestamp(b), b[4] != 0, binary.BigEndian.Uint16(b[5:]), binary.BigEndian.Uint16(b[7:])}
	}
	return nil
}

func decodeBattery(b []byte) Battery {
	return Battery{timestamp(b), binary.BigEndian.Uint16(b[4:]), b[6]}
}

// State holds the most recent event of each kind; nil fields have not been
// seen since connecting.
type State struct {
	MotorStall *MotorStall
	Colors     *Colors
	Bumpers    *Bumpers
	Light      *Light
	Battery    *Battery
	Touch      *Touch
	Cliff      *Cliff
}

func (s *State) update(ev Event) {
	switch e := ev.(type) {
	case MotorStall:
		s.MotorStall = &e
	case Colors:
		s.Colors = &e
	case Bumpers:
		s.Bumpers = &e
	case Light:
		s.Light = &e
	case Battery:
		s.Battery = &e
	case Touch:
		s.Touch = &e
	case Cliff:
		s.Cliff = &e
	}
}

// State returns a snapshot of the most recent events, without asking the
// robot.
func (r *Robot) State() State {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.state // fields point at values that are never mutated
}

// Subscription receives events on C until it is closed or the robot
// disconnects, at which point C is closed.
type Subscription struct {
	C       <-chan Event
	c       chan Event
	r       *Robot
	dropped atomic.Uint64
}

// Subscribe returns a subscription whose channel buffers up to buf events.
// Events that arrive while the buffer is full are dropped, so a slow
// subscriber never delays replies or other subscribers.
func (r *Robot) Subscribe(buf int) *Subscription {
	c := make(chan Event, buf)
	s := &Subscription{C: c, c: c, r: r}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed {
		close(c)
	} else {
		r.subs[s] = struct{}{}
	}
	return s
}

// Close stops delivery and closes C. It is safe to call more than once.
func (s *Subscription) Close() {
	s.r.mu.Lock()
	defer s.r.mu.Unlock()
	if _, ok := s.r.subs[s]; ok {
		delete(s.r.subs, s)
		close(s.c)
	}
}

// Dropped counts events discarded because C was full.
func (s *Subscription) Dropped() uint64 { return s.dropped.Load() }

// deliver is called with r.mu held.
func (s *Subscription) deliver(ev Event) {
	select {
	case s.c <- ev:
	default:
		s.dropped.Add(1)
	}
}
