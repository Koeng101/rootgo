// Command rootprobe checks the BLE transport and core library against a real
// Root robot: it scans, connects, reads robot info, exercises lights and
// sound, optionally tests motion and cancellation, then prints events.
//
// Cross-compile for an Apple Silicon Mac with:
//
//	GOOS=darwin GOARCH=arm64 CGO_ENABLED=0 go build ./cmd/rootprobe
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"sync"
	"time"

	"github.com/koeng101/rootgo/ble"
	"github.com/koeng101/rootgo/packet"
	"github.com/koeng101/rootgo/root"
)

func main() {
	var (
		scanFor = flag.Duration("scan", 5*time.Second, "how long to list robots before connecting")
		name    = flag.String("name", "", "connect to the robot with this name (default: first RT1)")
		move    = flag.Bool("move", false, "drive, rotate, and test that cancelling a drive stops the wheels")
		trace   = flag.Bool("trace", false, "log every packet sent and received")
		rename  = flag.Bool("rename-serial", false, "set the robot's name to its serial number")
		quiet   = flag.Bool("quiet", false, "skip the lights and sound checks")
		heading = flag.Bool("heading", false, "rotate in place and report poses, to check the heading convention")
		phase4  = flag.Bool("phase4", false, "test sensors, event enabling, sweep, notes and navigation (drives a ~15cm square)")
		speed   = flag.Bool("speed", false, "drive back and forth at increasing wheel speeds and measure them (needs ~1m clear)")
		events  = flag.Duration("events", 15*time.Second, "how long to print events (touch the robot!)")
	)
	flag.Parse()
	log.SetFlags(log.Ltime | log.Lmicroseconds)

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	log.Printf("scanning for %v...", *scanFor)
	var mu sync.Mutex
	seen := map[string]ble.Advertisement{}
	scanCtx, cancel := context.WithTimeout(ctx, *scanFor)
	err := ble.Scan(scanCtx, func(a ble.Advertisement) {
		mu.Lock()
		defer mu.Unlock()
		old, ok := seen[a.ID]
		seen[a.ID] = a
		if ok && old.Name == a.Name && old.Model == a.Model && (old.State == nil) == (a.State == nil) &&
			(a.State == nil || *old.State == *a.State) {
			return
		}
		state := "?"
		if a.State != nil {
			state = fmt.Sprintf("%+v", *a.State)
		}
		log.Printf("  found %q model=%q rssi=%d id=%s state=%s", a.Name, a.Model, a.RSSI, a.ID, state)
	})
	cancel()
	if err != nil {
		log.Fatal(err)
	}
	if ctx.Err() != nil {
		return
	}

	match := ble.Model("RT1")
	if *name != "" {
		match = ble.Name(*name)
	}
	log.Printf("connecting...")
	dialCtx, cancel := context.WithTimeout(ctx, 20*time.Second)
	conn, adv, err := ble.Dial(dialCtx, match)
	cancel()
	if err != nil {
		log.Fatalf("dial: %v", err)
	}
	log.Printf("connected to %q (%s)", adv.Name, adv.ID)

	r := root.New(conn)
	defer r.Close()
	if *trace {
		r.Trace(func(sent bool, p packet.Packet) {
			dir := "<-"
			if sent {
				dir = "->"
			}
			log.Printf("    %s %v", dir, p)
		})
	}

	check := func(what string, err error) {
		if err != nil {
			log.Printf("FAIL %s: %v", what, err)
		}
	}

	name2, err := r.Name(ctx)
	check("Name", err)
	serial, err := r.SerialNumber(ctx)
	check("SerialNumber", err)
	sku, err := r.SKU(ctx)
	check("SKU", err)
	log.Printf("name=%q serial=%q sku=%q", name2, serial, sku)
	for _, b := range []root.Board{root.BoardMain, root.BoardColor} {
		v, err := r.Versions(ctx, b)
		check("Versions", err)
		log.Printf("board %#x: firmware %d.%d.%d hardware %d.%d boot %d.%d protocol %d.%d", byte(b),
			v.FirmwareMajor, v.FirmwareMinor, v.FirmwarePatch, v.HardwareMajor, v.HardwareMinor,
			v.BootMajor, v.BootMinor, v.ProtocolMajor, v.ProtocolMinor)
	}
	batt, err := r.Battery(ctx)
	check("Battery", err)
	log.Printf("battery %d%% %dmV", batt.Percent, batt.MilliVolts)

	if *rename {
		newName, err := r.NameAfterSerial(ctx)
		check("NameAfterSerial", err)
		got, err := r.Name(ctx)
		check("Name", err)
		log.Printf("renamed to %q; robot now reports name %q", newName, got)
	}

	major, minor, err := r.Protocol(ctx)
	check("Protocol", err)
	log.Printf("protocol %d.%d", major, minor)

	if !*quiet {
		check("SetLights", r.SetLights(ctx, root.LightsSpin, 0, 255, 0))
		check("PlayNote", r.PlayNote(ctx, 440, 300*time.Millisecond))
		check("Say", r.Say(ctx, "hello from Go, this is a long phrase"))
	}
	if *move {
		testMotion(ctx, r, check)
	}
	if *heading {
		testHeading(ctx, r, check)
	}
	if *speed {
		testSpeed(ctx, r, check)
	}
	if *phase4 {
		testPhase4(ctx, r, check)
	}

	if *events > 0 {
		sub := r.Subscribe(64)
		log.Printf("TAP NOW: printing events for %v; press touch sensors and bumpers...", *events)
		check("SetLights", r.SetLights(ctx, root.LightsBlink, 0, 0, 255))
		check("PlayNote", r.PlayNote(ctx, 880, 500*time.Millisecond))
		timeout := time.After(*events)
	loop:
		for {
			select {
			case ev, ok := <-sub.C:
				if !ok {
					break loop
				}
				if _, ok := ev.(root.Colors); ok {
					continue // too chatty while the robot sits on the floor
				}
				log.Printf("  event %T %+v", ev, ev)
			case <-timeout:
				break loop
			case <-ctx.Done():
				break loop
			}
		}
		sub.Close()
		if n := sub.Dropped(); n > 0 {
			log.Printf("  dropped %d events", n)
		}
	}

	check("SetLights off", r.SetLights(context.Background(), root.LightsOff, 0, 0, 0))
	log.Printf("bad frames: %d", r.BadFrames())
}

func testMotion(ctx context.Context, r *root.Robot, check func(string, error)) {
	log.Printf("motion: drive 100mm")
	p, err := r.Drive(ctx, 100)
	check("Drive", err)
	log.Printf("  pose %+v", p)

	log.Printf("motion: rotate 90° clockwise")
	p, err = r.Rotate(ctx, 90)
	check("Rotate", err)
	log.Printf("  pose %+v", p)

	log.Printf("motion: drive 1000mm, cancelled after 1s")
	cctx, cancel := context.WithTimeout(ctx, time.Second)
	_, err = r.Drive(cctx, 1000)
	cancel()
	if !errors.Is(err, context.DeadlineExceeded) {
		log.Printf("FAIL cancelled Drive returned %v", err)
	}
	a, err := r.Position(ctx)
	check("Position", err)
	time.Sleep(2 * time.Second)
	b, err := r.Position(ctx)
	check("Position", err)
	moved := (b.X-a.X)*(b.X-a.X) + (b.Y-a.Y)*(b.Y-a.Y)
	if moved > 5*5 {
		log.Printf("FAIL wheels kept going after cancel: %+v -> %+v (stop-on-cancel needs Stop and Reset)", a, b)
	} else {
		log.Printf("  ok: robot stopped after cancel (%+v -> %+v)", a, b)
	}
}

func testHeading(ctx context.Context, r *root.Robot, check func(string, error)) {
	log.Printf("heading: reset position")
	check("ResetPosition", r.ResetPosition(ctx))
	p, err := r.Position(ctx)
	check("Position", err)
	log.Printf("  after reset:           %+v", p)
	for _, step := range []float64{90, 90, -180} {
		p, err = r.Rotate(ctx, step)
		check("Rotate", err)
		log.Printf("  after rotate %+5.0f°:   %+v", step, p)
	}
	p, err = r.Drive(ctx, 50)
	check("Drive", err)
	log.Printf("  after drive 50mm:      %+v", p)
	check("SetWheelSpeeds", r.SetWheelSpeeds(ctx, 0, 0))
	p, err = r.Rotate(ctx, 90)
	check("Rotate", err)
	log.Printf("  after rotate   +90°:   %+v", p)
	p, err = r.Drive(ctx, 50)
	check("Drive", err)
	log.Printf("  after drive 50mm:      %+v", p)
	_, err = r.Rotate(ctx, -90)
	check("Rotate", err)
}

func testSpeed(ctx context.Context, r *root.Robot, check func(string, error)) {
	sign := int32(1)
	for _, v := range []int32{50, 100, 150, 200, 300, 500} {
		s := v * sign
		sign = -sign
		check("SetWheelSpeeds", r.SetWheelSpeeds(ctx, s, s))
		time.Sleep(600 * time.Millisecond) // let it accelerate
		a, err := r.Position(ctx)
		check("Position", err)
		time.Sleep(800 * time.Millisecond)
		b, err := r.Position(ctx)
		check("Position", err)
		check("SetWheelSpeeds", r.SetWheelSpeeds(ctx, 0, 0))
		dx, dy := float64(b.X-a.X), float64(b.Y-a.Y)
		dt := (b.Timestamp - a.Timestamp).Seconds()
		log.Printf("speed: commanded %4d mm/s, measured %6.1f mm/s (%+v -> %+v)", s, math.Hypot(dx, dy)/dt, a, b)
		time.Sleep(700 * time.Millisecond)
	}
}

func testPhase4(ctx context.Context, r *root.Robot, check func(string, error)) {
	ev, err := r.EnabledEvents(ctx)
	check("EnabledEvents", err)
	log.Printf("enabled events: %v", ev)

	check("DisableEvents", r.DisableEvents(ctx, root.EventsColor))
	ev, err = r.EnabledEvents(ctx)
	check("EnabledEvents", err)
	log.Printf("  after disabling color: %v", ev)
	check("EnableEvents", r.EnableEvents(ctx, root.EventsColor))
	ev, err = r.EnabledEvents(ctx)
	check("EnabledEvents", err)
	log.Printf("  after re-enabling:     %v", ev)

	l, err := r.LightValues(ctx)
	check("LightValues", err)
	log.Printf("light: %+v", l)
	a, err := r.Accelerometer(ctx)
	check("Accelerometer", err)
	log.Printf("accelerometer: %+v", a)
	for bank := root.ColorBank0to7; bank <= root.ColorBank24to31; bank++ {
		v, err := r.ColorSensor(ctx, bank, root.ColorLightingAll, root.ColorFormatMillivolt)
		check("ColorSensor", err)
		log.Printf("color bank %d (mV, all LEDs): %v", bank, v)
	}

	check("SetGravityCompensation", r.SetGravityCompensation(ctx, root.GravityMarkerDown, 50))

	log.Printf("sweep 300 -> 1200 Hz")
	check("PlaySweep", r.PlaySweep(ctx, root.Sweep{StartHz: 300, EndHz: 1200, Duration: time.Second,
		Attack: 50 * time.Millisecond, Release: 100 * time.Millisecond, Volume: 200}))
	log.Printf("notes C4 E4 G4 C5")
	for _, n := range []string{"C4", "E4", "G4", "C5"} {
		hz, err := root.NoteHz(n)
		check("NoteHz", err)
		check("PlayNote", r.PlayNote(ctx, uint32(math.Round(hz)), 200*time.Millisecond))
	}

	log.Printf("navigate a 150mm square")
	check("ResetPosition", r.ResetPosition(ctx))
	for _, pt := range [][2]int32{{0, 150}, {150, 150}, {150, 0}, {0, 0}} {
		p, err := r.NavigateTo(ctx, pt[0], pt[1])
		check("NavigateTo", err)
		log.Printf("  NavigateTo(%d, %d) -> %+v", pt[0], pt[1], p)
	}
	p, err := r.TurnTo(ctx, 90)
	check("TurnTo", err)
	log.Printf("  TurnTo(90) -> %+v", p)
}
