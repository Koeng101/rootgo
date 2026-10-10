# rootgo

Control an [iRobot Root](https://edu.irobot.com/what-we-offer/root-robot) rt1 robot from Go over Bluetooth Low Energy.

```go
conn, _, err := ble.Dial(ctx, ble.Model("RT1"))
if err != nil {
	log.Fatal(err)
}
r := root.New(conn)
defer r.Close()

r.SetLights(ctx, root.LightsSpin, 0, 255, 0)
r.Say(ctx, "hello")
pose, err := r.Drive(ctx, 100) // mm; blocks until the robot finishes
```

The library implements iRobot's [Root BLE protocol](https://github.com/iRobotEducation/root-robot-ble-protocol). It covers every command and event the rt1 supports, plus host-side navigation. Its only dependency is [purego](https://github.com/ebitengine/purego), which the macOS backend uses.

## Status

| | |
|---|---|
| Robot | Root **rt1**. rt0 and Create 3 are out of scope. |
| Firmware | Protocol 1.2 (firmware 2.1) is tested on hardware. Protocol 1.3 follows the published spec. |
| macOS | Works. CoreBluetooth is called through the Objective-C runtime, with no cgo, so it cross-compiles from Linux. |
| Linux | Not yet. A BlueZ (D-Bus) backend is planned. |

The packet format is tested against the protocol's reference vectors and against the Python SDK. The core is tested with a fake transport under the race detector. Every command and event below has been run on a real rt1, except the marker/eraser.

## Packages

- **`root`** is the robot API. Methods block until the robot replies and take a `context.Context`.
- **`ble`** finds robots and connects to them. A `*ble.Conn` implements `root.Transport`.
- **`packet`** is the 20-byte wire format and its CRC-8.

## Concepts

**Blocking calls with contexts.** Queries (`Battery`, `Position`, …) return the robot's reply. If the context has no deadline, they time out after 3 s. Actions (`Drive`, `Rotate`, `Arc`, `Say`, `PlayNote`, `PlaySweep`, `SetMarker`, …) return when the robot reports that they finished.

**Cancelling stops the robot.** If the context of a motion command is cancelled, the wheels are stopped. Cancelling a sound command stops the sound.

```go
ctx, cancel := context.WithTimeout(ctx, time.Second)
defer cancel()
_, err := r.Drive(ctx, 1000) // context.DeadlineExceeded after 1s, and the robot stops
```

**One action of each kind at a time.** Motion, sound and the marker each allow one running action. Starting a second one returns `ErrBusy`, and actions of different kinds can run together. `Stop` or the nose button cancels pending actions, and they return `ErrStopped`.

**Events.** Each subscriber gets its own buffered channel. A full channel drops events and counts them, so it never blocks the robot. `State()` returns the latest event of each kind without a round trip.

```go
events := r.Subscribe(16)
defer events.Close()
for ev := range events.C {
	switch e := ev.(type) {
	case root.Bumpers:
		log.Printf("bumped: left=%v right=%v", e.Left, e.Right)
	case root.StopButton:
		return
	}
}
```

Event types are `StopButton`, `Bumpers`, `Touch`, `Cliff`, `Light`, `Battery`, `Colors` and `MotorStall`.

**Units and coordinates.** Distances are in mm and angles in degrees. Wheel speeds are in mm/s, and the robot caps them at ±100. `Rotate` and `Arc` treat positive angles as clockwise. `Pose` starts at (0, 0) facing +Y with a heading of 90°. +X is to the robot's right, and the heading is counterclockwise-positive.

**Navigation.** Root firmware has no navigate command. `NavigateTo(x, y)` and `TurnTo(heading)` compute their moves from the robot's position estimate. They hold the motion slot until they finish.

## Firmware differences

Protocol 1.2 (firmware 2.1) differs from the published 1.3 spec in ways the library handles for you. It reads the version once (`Protocol`) and then:

- **Motion replies carry no pose**, so `Drive`, `Rotate` and `Arc` ask for the position afterwards.
- **`Get Position` packs X, Y and heading as int16** instead of int32.
- **Heading is clockwise from the start** (0 at reset). The library converts it to the 1.3 convention, so `Pose` means the same thing on both.

Also, the color sensor reports more colors than the documented five. The library keeps the raw values; for example, `ColorLowSaturation` (14) is a gray floor.

## Running on macOS

Build on any machine; no cgo is needed:

```sh
GOOS=darwin GOARCH=arm64 CGO_ENABLED=0 go build ./examples/hello
```

Run the binary from a terminal app on the Mac. macOS asks that app for Bluetooth permission on first use; grant it under System Settings → Privacy & Security → Bluetooth. A program started directly over SSH is denied Bluetooth. To start one remotely, launch it inside Terminal with `open -a Terminal script.command`.

## Examples

| | |
|---|---|
| [`examples/hello`](examples/hello) | Connect, light up, say hello, report the battery. |
| [`examples/square`](examples/square) | Drive (or `-draw`) a square with `NavigateTo`. Ctrl-C stops mid-move. |
| [`examples/bumper`](examples/bumper) | Drive until bumped, back off, turn away and carry on. |
| [`examples/piano`](examples/piano) | The four touch sensors under the top panel play notes. |
| [`cmd/rootprobe`](cmd/rootprobe) | The hardware test suite, with a packet trace (`-trace`), motion and cancel checks (`-move`), the speed test (`-speed`) and more. |

## Testing

```sh
go test -race ./...
```

The tests need no robot. For hardware checks, run `rootprobe`; see `rootprobe -h` for its options.
