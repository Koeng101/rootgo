package root

import (
	"context"
	"encoding/binary"
	"time"
)

// ColorBank selects 8 of the 32 color sensors, left to right.
type ColorBank uint8

const (
	ColorBank0to7 ColorBank = iota
	ColorBank8to15
	ColorBank16to23
	ColorBank24to31
)

// ColorLighting selects the LED illumination used while measuring.
type ColorLighting uint8

const (
	ColorLightingOff ColorLighting = iota
	ColorLightingRed
	ColorLightingGreen
	ColorLightingBlue
	ColorLightingAll
)

// ColorFormat selects the units of color sensor readings.
type ColorFormat uint8

const (
	ColorFormatADC       ColorFormat = 0 // 12-bit ADC counts
	ColorFormatMillivolt ColorFormat = 1
)

// ColorSensor reads eight raw color sensor values from one bank, leftmost
// first.
func (r *Robot) ColorSensor(ctx context.Context, bank ColorBank, lighting ColorLighting, format ColorFormat) ([8]uint16, error) {
	var out [8]uint16
	p, err := r.query(ctx, devColor, 1, []byte{byte(bank), byte(lighting), byte(format)})
	if err != nil {
		return out, err
	}
	for i := range out {
		out[i] = binary.BigEndian.Uint16(p.Payload[2*i:])
	}
	return out, nil
}

// LightValues reads the ambient light seen by the two eyes. State is not
// set.
func (r *Robot) LightValues(ctx context.Context) (Light, error) {
	p, err := r.query(ctx, devLight, 1, nil)
	if err != nil {
		return Light{}, err
	}
	b := p.Payload[:]
	return Light{Timestamp: timestamp(b), Left: binary.BigEndian.Uint16(b[4:]), Right: binary.BigEndian.Uint16(b[6:])}, nil
}

// Acceleration is an accelerometer reading in milli-g.
type Acceleration struct {
	Timestamp time.Duration
	X, Y, Z   int16
}

// Accelerometer reads the accelerometer.
func (r *Robot) Accelerometer(ctx context.Context) (Acceleration, error) {
	p, err := r.query(ctx, devAccel, 1, nil)
	if err != nil {
		return Acceleration{}, err
	}
	b := p.Payload[:]
	return Acceleration{
		Timestamp: timestamp(b),
		X:         int16(binary.BigEndian.Uint16(b[4:])),
		Y:         int16(binary.BigEndian.Uint16(b[6:])),
		Z:         int16(binary.BigEndian.Uint16(b[8:])),
	}, nil
}

// EventSource is a device whose events can be enabled or disabled.
type EventSource uint8

const (
	EventsMotorStall EventSource = EventSource(devMotors)
	EventsColor      EventSource = EventSource(devColor)
	EventsBumpers    EventSource = EventSource(devBumpers)
	EventsLight      EventSource = EventSource(devLight)
	EventsBattery    EventSource = EventSource(devBattery)
	EventsTouch      EventSource = EventSource(devTouch)
	EventsCliff      EventSource = EventSource(devCliff)
)

// eventBits encodes devices as the protocol's 128-bit field: device 0 is
// the lowest bit of the last payload byte.
func eventBits(srcs []EventSource) []byte {
	b := make([]byte, 16)
	for _, s := range srcs {
		if s < 128 {
			b[15-s/8] |= 1 << (s % 8)
		}
	}
	return b
}

// EnableEvents turns on events from the given sources. Events are on by
// default.
func (r *Robot) EnableEvents(ctx context.Context, srcs ...EventSource) error {
	return r.command(ctx, devGeneral, 7, eventBits(srcs))
}

// DisableEvents turns off events from the given sources. The stop button
// event cannot be disabled.
func (r *Robot) DisableEvents(ctx context.Context, srcs ...EventSource) error {
	return r.command(ctx, devGeneral, 9, eventBits(srcs))
}

// EnabledEvents asks which devices currently send events. The result may
// include devices other than the EventSource constants, such as device 0.
func (r *Robot) EnabledEvents(ctx context.Context) ([]EventSource, error) {
	p, err := r.query(ctx, devGeneral, 11, nil)
	if err != nil {
		return nil, err
	}
	var out []EventSource
	for d := range 128 {
		if p.Payload[15-d/8]&(1<<(d%8)) != 0 {
			out = append(out, EventSource(d))
		}
	}
	return out, nil
}
