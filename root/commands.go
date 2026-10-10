package root

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"time"
	"unicode/utf8"

	"github.com/koeng101/rootgo/packet"
)

// Device numbers.
const (
	devGeneral uint8 = 0
	devMotors  uint8 = 1
	devMarker  uint8 = 2
	devLEDs    uint8 = 3
	devColor   uint8 = 4
	devSound   uint8 = 5
	devBumpers uint8 = 12
	devLight   uint8 = 13
	devBattery uint8 = 14
	devAccel   uint8 = 16
	devTouch   uint8 = 17
	devCliff   uint8 = 20
)

// Command numbers used in more than one place.
const (
	cmdSetSpeeds uint8 = 4
	cmdStopSound uint8 = 1
)

// Board selects which board Versions reports on.
type Board uint8

const (
	BoardMain  Board = 0xA5
	BoardColor Board = 0xC6
)

// Versions are the firmware, hardware, bootloader and BLE protocol
// versions of one board.
type Versions struct {
	Board                                       Board
	FirmwareMajor, FirmwareMinor, FirmwarePatch uint8
	HardwareMajor, HardwareMinor                uint8
	BootMajor, BootMinor                        uint8
	ProtocolMajor, ProtocolMinor                uint8
}

// Versions asks for the versions of a board.
func (r *Robot) Versions(ctx context.Context, b Board) (Versions, error) {
	p, err := r.query(ctx, devGeneral, 0, []byte{byte(b)})
	if err != nil {
		return Versions{}, err
	}
	v := p.Payload
	return Versions{
		Board:         Board(v[0]),
		FirmwareMajor: v[1], FirmwareMinor: v[2], FirmwarePatch: v[9],
		HardwareMajor: v[3], HardwareMinor: v[4],
		BootMajor: v[5], BootMinor: v[6],
		ProtocolMajor: v[7], ProtocolMinor: v[8],
	}, nil
}

// SetName sets the robot's BLE advertising name, at most 16 bytes of UTF-8.
func (r *Robot) SetName(ctx context.Context, name string) error {
	if len(name) > packet.PayloadSize {
		return fmt.Errorf("root: name is %d bytes, max %d", len(name), packet.PayloadSize)
	}
	return r.command(ctx, devGeneral, 1, []byte(name))
}

// NameAfterSerial sets the robot's advertising name to its serial number,
// giving it a unique name, and returns that name. Robots ship named "ROOT",
// so this makes several robots easy to tell apart when scanning.
func (r *Robot) NameAfterSerial(ctx context.Context) (string, error) {
	serial, err := r.SerialNumber(ctx)
	if err != nil {
		return "", err
	}
	if serial == "" {
		return "", fmt.Errorf("root: robot reported an empty serial number")
	}
	if err := r.SetName(ctx, serial); err != nil {
		return "", err
	}
	return serial, nil
}

// Name asks for the robot's BLE advertising name.
func (r *Robot) Name(ctx context.Context) (string, error) {
	p, err := r.query(ctx, devGeneral, 2, nil)
	if err != nil {
		return "", err
	}
	return cString(p.Payload[:]), nil
}

// Stop immediately stops the robot and cancels its pending actions, which
// fail with ErrStopped. It also resets the robot's position to the origin.
func (r *Robot) Stop(ctx context.Context) error {
	if err := r.command(ctx, devGeneral, 3, nil); err != nil {
		return err
	}
	r.mu.Lock()
	r.failActionsLocked(ErrStopped)
	r.mu.Unlock()
	return nil
}

// Disconnect asks the robot to drop the BLE connection. Call Close
// afterwards to release the transport.
func (r *Robot) Disconnect(ctx context.Context) error {
	return r.command(ctx, devGeneral, 6, nil)
}

// SerialNumber asks for the robot's serial number.
func (r *Robot) SerialNumber(ctx context.Context) (string, error) {
	p, err := r.query(ctx, devGeneral, 14, nil)
	if err != nil {
		return "", err
	}
	return cString(p.Payload[:12]), nil
}

// SKU asks for the robot's product SKU.
func (r *Robot) SKU(ctx context.Context) (string, error) {
	p, err := r.query(ctx, devGeneral, 15, nil)
	if err != nil {
		return "", err
	}
	return cString(p.Payload[:]), nil
}

// SetWheelSpeeds sets both wheel speeds in mm/s; positive is forwards. The
// robot limits speeds to ±100 mm/s (measured on an rt1).
func (r *Robot) SetWheelSpeeds(ctx context.Context, left, right int32) error {
	b := make([]byte, 8)
	binary.BigEndian.PutUint32(b, uint32(left))
	binary.BigEndian.PutUint32(b[4:], uint32(right))
	return r.command(ctx, devMotors, cmdSetSpeeds, b)
}

// SetLeftSpeed sets the left wheel speed in mm/s.
func (r *Robot) SetLeftSpeed(ctx context.Context, speed int32) error {
	return r.command(ctx, devMotors, 6, int32Bytes(speed))
}

// SetRightSpeed sets the right wheel speed in mm/s.
func (r *Robot) SetRightSpeed(ctx context.Context, speed int32) error {
	return r.command(ctx, devMotors, 7, int32Bytes(speed))
}

// Pose is the robot's position estimate. The robot starts at the origin
// facing +Y with a heading of 90°; X extends to its right.
type Pose struct {
	Timestamp time.Duration // since robot power-on
	X, Y      int32         // mm
	Heading   float64       // degrees, [0, 360), counterclockwise positive
}

// Protocol returns the robot's BLE protocol version. The first call asks the
// robot; later calls return the cached answer.
func (r *Robot) Protocol(ctx context.Context) (major, minor uint8, err error) {
	r.protoMu.Lock()
	defer r.protoMu.Unlock()
	if !r.protoKnown {
		v, err := r.Versions(ctx, BoardMain)
		if err != nil {
			return 0, 0, err
		}
		r.proto = [2]uint8{v.ProtocolMajor, v.ProtocolMinor}
		r.protoKnown = true
	}
	return r.proto[0], r.proto[1], nil
}

// legacy reports whether the robot speaks protocol 1.2 or older, where
// motion replies carry no pose and Get Position packs its fields as int16.
func (r *Robot) legacy(ctx context.Context) (bool, error) {
	major, minor, err := r.Protocol(ctx)
	if err != nil {
		return false, err
	}
	return major < 1 || major == 1 && minor < 3, nil
}

func decodePose(p packet.Packet) Pose {
	b := p.Payload[:]
	return Pose{
		Timestamp: timestamp(b),
		X:         int32(binary.BigEndian.Uint32(b[4:])),
		Y:         int32(binary.BigEndian.Uint32(b[8:])),
		Heading:   float64(int16(binary.BigEndian.Uint16(b[12:]))) / 10,
	}
}

// decodeLegacyPose decodes a protocol 1.2 Get Position reply: timestamp,
// then X, Y and heading as int16. This layout is not in the published
// documentation; it was observed on firmware 2.1.
func decodeLegacyPose(p packet.Packet) Pose {
	b := p.Payload[:]
	return Pose{
		Timestamp: timestamp(b),
		X:         int32(int16(binary.BigEndian.Uint16(b[4:]))),
		Y:         int32(int16(binary.BigEndian.Uint16(b[6:]))),
		Heading:   legacyHeading(int16(binary.BigEndian.Uint16(b[8:]))),
	}
}

// legacyHeading converts a protocol 1.2 heading to the 1.3 convention.
// Firmware 2.1 reports decidegrees clockwise from the starting
// orientation (0 at reset), while 1.3 reports counterclockwise-positive
// yaw starting at 90°. Both use the same X/Y axes.
func legacyHeading(raw int16) float64 {
	h := math.Mod(900-float64(raw), 3600)
	if h < 0 {
		h += 3600
	}
	return h / 10
}

// move runs a motion action and returns the pose when it finishes. Older
// firmware does not report the pose with the finished reply, so it is
// requested separately.
func (r *Robot) move(ctx context.Context, cmd uint8, payload []byte) (Pose, error) {
	release, err := r.acquire(slotMotion)
	if err != nil {
		return Pose{}, err
	}
	defer release()
	return r.moveHeld(ctx, cmd, payload)
}

// moveHeld is move for a caller that already holds the motion slot.
func (r *Robot) moveHeld(ctx context.Context, cmd uint8, payload []byte) (Pose, error) {
	p, err := r.roundTrip(ctx, slotMotion, devMotors, cmd, payload)
	if err != nil {
		return Pose{}, err
	}
	legacy, err := r.legacy(ctx)
	if err != nil {
		return Pose{}, fmt.Errorf("root: motion finished but pose unavailable: %w", err)
	}
	if !legacy {
		return decodePose(p), nil
	}
	pose, err := r.Position(ctx)
	if err != nil {
		return Pose{}, fmt.Errorf("root: motion finished but pose unavailable: %w", err)
	}
	return pose, nil
}

// Drive drives mm millimeters in a straight line (negative is backwards)
// and returns the pose when finished.
func (r *Robot) Drive(ctx context.Context, mm int32) (Pose, error) {
	return r.move(ctx, 8, int32Bytes(mm))
}

// Rotate turns in place by degrees (positive is clockwise) and returns the
// pose when finished.
func (r *Robot) Rotate(ctx context.Context, degrees float64) (Pose, error) {
	return r.move(ctx, 12, int32Bytes(decidegrees(degrees)))
}

// Arc drives along an arc of degrees (positive is clockwise) with radius mm
// (positive puts the center to the robot's right) and returns the pose when
// finished.
func (r *Robot) Arc(ctx context.Context, degrees float64, radius int32) (Pose, error) {
	b := make([]byte, 8)
	binary.BigEndian.PutUint32(b, uint32(decidegrees(degrees)))
	binary.BigEndian.PutUint32(b[4:], uint32(radius))
	return r.move(ctx, 27, b)
}

// GravityMode controls when gravity compensation is active, which helps
// the robot drive on a vertical whiteboard.
type GravityMode uint8

const (
	GravityOff        GravityMode = 0
	GravityOn         GravityMode = 1
	GravityMarkerDown GravityMode = 2 // the default
)

// SetGravityCompensation sets the gravity compensation mode and amount, in
// percent (0-100; the default is 50).
func (r *Robot) SetGravityCompensation(ctx context.Context, mode GravityMode, percent float64) error {
	if percent < 0 || percent > 100 {
		return fmt.Errorf("root: gravity compensation %v%% is outside 0-100", percent)
	}
	b := []byte{byte(mode), 0, 0}
	binary.BigEndian.PutUint16(b[1:], uint16(math.Round(percent*10)))
	return r.command(ctx, devMotors, 13, b)
}

// ResetPosition resets the robot's pose to the origin, facing 90°.
func (r *Robot) ResetPosition(ctx context.Context) error {
	return r.command(ctx, devMotors, 15, nil)
}

// Position asks for the robot's current pose.
func (r *Robot) Position(ctx context.Context) (Pose, error) {
	legacy, err := r.legacy(ctx)
	if err != nil {
		return Pose{}, err
	}
	p, err := r.query(ctx, devMotors, 16, nil)
	if err != nil {
		return Pose{}, err
	}
	if legacy {
		return decodeLegacyPose(p), nil
	}
	return decodePose(p), nil
}

// MarkerPosition is the position of the marker/eraser actuator.
type MarkerPosition uint8

const (
	MarkerUp   MarkerPosition = 0 // marker and eraser up
	MarkerDown MarkerPosition = 1 // marker down, eraser up
	EraserDown MarkerPosition = 2 // marker up, eraser down
)

// SetMarker moves the marker/eraser and waits until it has moved.
func (r *Robot) SetMarker(ctx context.Context, pos MarkerPosition) error {
	_, err := r.action(ctx, slotMarker, devMarker, 0, []byte{byte(pos)})
	return err
}

// LightPattern is an LED animation.
type LightPattern uint8

const (
	LightsOff LightPattern = iota
	LightsOn
	LightsBlink
	LightsSpin
)

// SetLights sets the LED animation and color.
func (r *Robot) SetLights(ctx context.Context, pattern LightPattern, red, green, blue uint8) error {
	return r.command(ctx, devLEDs, 2, []byte{byte(pattern), red, green, blue})
}

// PlayNote plays a tone and waits until it finishes. Durations are
// truncated to milliseconds and capped at 65.535s.
func (r *Robot) PlayNote(ctx context.Context, hz uint32, d time.Duration) error {
	b := make([]byte, 6)
	binary.BigEndian.PutUint32(b, hz)
	binary.BigEndian.PutUint16(b[4:], uint16(min(d.Milliseconds(), math.MaxUint16)))
	_, err := r.action(ctx, slotSound, devSound, 0, b)
	return err
}

// StopSound stops any sound that is playing.
func (r *Robot) StopSound(ctx context.Context) error {
	return r.command(ctx, devSound, cmdStopSound, nil)
}

// Say speaks phrase in robot language and waits until it finishes. Phrases
// longer than one packet are sent in pieces split on character boundaries.
func (r *Robot) Say(ctx context.Context, phrase string) error {
	release, err := r.acquire(slotSound)
	if err != nil {
		return err
	}
	defer release()
	for _, chunk := range splitUTF8(phrase, packet.PayloadSize) {
		if _, err := r.roundTrip(ctx, slotSound, devSound, 4, []byte(chunk)); err != nil {
			return err
		}
	}
	return nil
}

// Modulation is a sound property varied while a sweep plays.
type Modulation uint8

const (
	ModulationNone       Modulation = 0
	ModulationVolume     Modulation = 1
	ModulationPulseWidth Modulation = 2
	ModulationFrequency  Modulation = 3
)

// Sweep describes a tone that slides linearly between two frequencies.
type Sweep struct {
	StartHz, EndHz  float64
	Duration        time.Duration // up to 65.535s
	Attack, Release time.Duration // volume ramps, up to 255ms each, within Duration
	Volume          uint8         // 0 (silent) to 255
	Modulation      Modulation
	ModulationHz    uint8
}

// PlaySweep plays a sweep and waits until it finishes.
func (r *Robot) PlaySweep(ctx context.Context, s Sweep) error {
	ms := func(d time.Duration) int64 { return d.Milliseconds() }
	switch {
	case s.StartHz < 0 || s.EndHz < 0 || s.StartHz > math.MaxUint32/1000 || s.EndHz > math.MaxUint32/1000:
		return fmt.Errorf("root: sweep frequency out of range")
	case ms(s.Duration) < 0 || ms(s.Duration) > math.MaxUint16:
		return fmt.Errorf("root: sweep duration %v out of range", s.Duration)
	case ms(s.Attack) < 0 || ms(s.Attack) > math.MaxUint8 || ms(s.Release) < 0 || ms(s.Release) > math.MaxUint8:
		return fmt.Errorf("root: sweep attack and release must be 0-255ms")
	case ms(s.Attack)+ms(s.Release) > ms(s.Duration):
		return fmt.Errorf("root: sweep attack plus release exceeds its duration")
	}
	b := make([]byte, 15)
	binary.BigEndian.PutUint32(b, uint32(math.Round(s.StartHz*1000)))
	binary.BigEndian.PutUint32(b[4:], uint32(math.Round(s.EndHz*1000)))
	binary.BigEndian.PutUint16(b[8:], uint16(ms(s.Duration)))
	b[10], b[11] = uint8(ms(s.Attack)), uint8(ms(s.Release))
	b[12], b[13], b[14] = s.Volume, byte(s.Modulation), s.ModulationHz
	_, err := r.action(ctx, slotSound, devSound, 5, b)
	return err
}

// Battery asks for the battery level.
func (r *Robot) Battery(ctx context.Context) (Battery, error) {
	p, err := r.query(ctx, devBattery, 1, nil)
	if err != nil {
		return Battery{}, err
	}
	return decodeBattery(p.Payload[:]), nil
}

func int32Bytes(v int32) []byte {
	return binary.BigEndian.AppendUint32(nil, uint32(v))
}

func decidegrees(deg float64) int32 {
	return int32(math.Round(deg * 10))
}

// cString returns b up to its first NUL byte.
func cString(b []byte) string {
	if i := bytes.IndexByte(b, 0); i >= 0 {
		b = b[:i]
	}
	return string(b)
}

// splitUTF8 splits s into pieces of at most n bytes without splitting a
// character.
func splitUTF8(s string, n int) []string {
	var out []string
	for len(s) > n {
		i := n
		for i > 0 && !utf8.RuneStart(s[i]) {
			i--
		}
		out = append(out, s[:i])
		s = s[i:]
	}
	if s != "" {
		out = append(out, s)
	}
	return out
}
