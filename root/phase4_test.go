package root

import (
	"context"
	"encoding/binary"
	"math"
	"testing"
	"time"
)

func TestEventBits(t *testing.T) {
	b := eventBits([]EventSource{EventsMotorStall, EventsColor, EventsCliff})
	want := make([]byte, 16)
	want[15] = 1<<1 | 1<<4 // devices 1 and 4
	want[13] = 1 << 4      // device 20
	if string(b) != string(want) {
		t.Fatalf("eventBits = % x, want % x", b, want)
	}
}

func TestEnabledEvents(t *testing.T) {
	r, f := setup(t)
	ch := async(func() ([]EventSource, error) { return r.EnabledEvents(context.Background()) })
	req := f.next(t)
	if req.Dev != 0 || req.Cmd != 11 {
		t.Fatalf("sent %v", req)
	}
	payload := make([]byte, 16)
	payload[15] = 1 | 1<<4 // devices 0 and 4
	payload[14] = 1 << 4   // device 12
	f.reply(req, payload)
	got, err := wait(t, ch)
	if err != nil || len(got) != 3 || got[0] != 0 || got[1] != EventsColor || got[2] != EventsBumpers {
		t.Fatalf("EnabledEvents() = %v, %v", got, err)
	}
}

func TestSensorQueries(t *testing.T) {
	r, f := setup(t)
	ctx := context.Background()

	cs := async(func() ([8]uint16, error) {
		return r.ColorSensor(ctx, ColorBank8to15, ColorLightingAll, ColorFormatMillivolt)
	})
	req := f.next(t)
	if req.Dev != 4 || req.Cmd != 1 || req.Payload[0] != 1 || req.Payload[1] != 4 || req.Payload[2] != 1 {
		t.Fatalf("ColorSensor sent %v", req)
	}
	f.reply(req, []byte{0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0x10, 0})
	if v, err := wait(t, cs); err != nil || v != [8]uint16{1, 2, 3, 4, 5, 6, 7, 4096} {
		t.Fatalf("ColorSensor() = %v, %v", v, err)
	}

	acc := async(func() (Acceleration, error) { return r.Accelerometer(ctx) })
	f.reply(f.next(t), []byte{0, 0, 0, 5, 0xFF, 0xF6, 0, 20, 0x03, 0xE8})
	if a, err := wait(t, acc); err != nil || a != (Acceleration{5 * time.Millisecond, -10, 20, 1000}) {
		t.Fatalf("Accelerometer() = %+v, %v", a, err)
	}

	light := async(func() (Light, error) { return r.LightValues(ctx) })
	f.reply(f.next(t), []byte{0, 0, 0, 1, 0x01, 0xF4, 0x03, 0xE8})
	if l, err := wait(t, light); err != nil || l.Left != 500 || l.Right != 1000 {
		t.Fatalf("LightValues() = %+v, %v", l, err)
	}
}

func TestSweepAndGravityEncoding(t *testing.T) {
	r, f := setup(t)
	ctx := context.Background()
	ch := async(func() (struct{}, error) {
		return struct{}{}, r.PlaySweep(ctx, Sweep{StartHz: 440, EndHz: 880.5, Duration: time.Second,
			Attack: 10 * time.Millisecond, Release: 20 * time.Millisecond, Volume: 200,
			Modulation: ModulationVolume, ModulationHz: 5})
	})
	req := f.next(t)
	want := [16]byte{0, 0x06, 0xB6, 0xC0, 0, 0x0D, 0x6F, 0x74, 0x03, 0xE8, 10, 20, 200, 1, 5, 0}
	if req.Dev != 5 || req.Cmd != 5 || req.Payload != want {
		t.Fatalf("PlaySweep sent %v", req)
	}
	f.reply(req, nil)
	if _, err := wait(t, ch); err != nil {
		t.Fatal(err)
	}
	if err := r.PlaySweep(ctx, Sweep{Duration: 10 * time.Millisecond, Attack: 10 * time.Millisecond, Release: time.Millisecond}); err == nil {
		t.Error("PlaySweep accepted attack+release > duration")
	}

	r.SetGravityCompensation(ctx, GravityOn, 75)
	if p := f.next(t); p.Dev != 1 || p.Cmd != 13 || p.Payload[0] != 1 || binary.BigEndian.Uint16(p.Payload[1:]) != 750 {
		t.Fatalf("SetGravityCompensation sent %v", p)
	}
}

func TestClockwiseTurn(t *testing.T) {
	for _, tc := range []struct{ from, to, want float64 }{
		{90, 0, 90},
		{0, 90, -90},
		{10, 350, 20},
		{350, 10, -20},
		{90, 270, 180},
		{270, 90, 180},
	} {
		if got := clockwiseTurn(tc.from, tc.to); math.Abs(got-tc.want) > 1e-9 {
			t.Errorf("clockwiseTurn(%v, %v) = %v, want %v", tc.from, tc.to, got, tc.want)
		}
	}
}

func TestNavigateTo(t *testing.T) {
	r, f := setup(t)
	ch := async(func() (Pose, error) { return r.NavigateTo(context.Background(), 100, 0) })

	pos := f.next(t) // start at the origin facing +Y (90°)
	f.reply(pos, posePayload(0, 0, 0, 900))
	rot := f.next(t)
	if rot.Cmd != 12 || int32(binary.BigEndian.Uint32(rot.Payload[:])) != 900 {
		t.Fatalf("sent %v, want rotate 90° clockwise", rot)
	}
	// Another motion is rejected while navigating.
	if _, err := r.Drive(context.Background(), 1); err != ErrBusy {
		t.Fatalf("Drive during NavigateTo: %v", err)
	}
	f.reply(rot, posePayload(0, 0, 0, 0))
	drive := f.next(t)
	if drive.Cmd != 8 || int32(binary.BigEndian.Uint32(drive.Payload[:])) != 100 {
		t.Fatalf("sent %v, want drive 100", drive)
	}
	f.reply(drive, posePayload(0, 100, 0, 0))
	if p, err := wait(t, ch); err != nil || p.X != 100 {
		t.Fatalf("NavigateTo() = %+v, %v", p, err)
	}
}

func TestNoteHz(t *testing.T) {
	for _, tc := range []struct {
		note string
		want float64
	}{
		{"A4", 440}, {"A5", 880}, {"C4", 261.6256}, {"C#4", 277.1826}, {"Db4", 277.1826}, {"B3", 246.9417},
	} {
		got, err := NoteHz(tc.note)
		if err != nil || math.Abs(got-tc.want) > 1e-3 {
			t.Errorf("NoteHz(%q) = %v, %v; want %v", tc.note, got, err, tc.want)
		}
	}
	for _, bad := range []string{"", "H4", "A", "A#", "Cx4"} {
		if _, err := NoteHz(bad); err == nil {
			t.Errorf("NoteHz(%q) succeeded", bad)
		}
	}
}
