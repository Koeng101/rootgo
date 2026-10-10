// Package ble finds and connects to Root robots over Bluetooth Low Energy.
//
// A *Conn implements root.Transport:
//
//	conn, adv, err := ble.Dial(ctx, ble.Model("RT1"))
//	robot := root.New(conn)
//
// On macOS it uses CoreBluetooth through the Objective-C runtime (no cgo),
// so it cross-compiles with GOOS=darwin. Run the binary from a local
// terminal app; macOS asks that app for Bluetooth permission on first use.
package ble

import (
	"bytes"
	"context"
	"errors"
)

// GATT identifiers, named from the robot's point of view as in the
// protocol documentation.
const (
	RootServiceUUID = "48c5d828-ac2a-442d-97a3-0c9822b04979" // advertised by every Root robot
	UARTServiceUUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
	RXCharUUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // the host writes packets here
	TXCharUUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // the robot notifies packets here
)

var (
	ErrUnsupported = errors.New("ble: not supported on this platform yet")
	ErrClosed      = errors.New("ble: connection closed")
)

// Advertisement describes a robot seen while scanning.
type Advertisement struct {
	// ID identifies the robot to Connect. On macOS it is a UUID that
	// CoreBluetooth assigns per host, not the robot's MAC address.
	ID               string
	Name             string
	Model            string // "RT1", "RT0" or "Ci3"; empty until the manufacturer data is seen
	RSSI             int
	ManufacturerData []byte // including the leading 2-byte company ID
}

// parseModel extracts the robot type string that follows the 2-byte
// company ID in the manufacturer data.
func parseModel(mfr []byte) string {
	if len(mfr) <= 2 {
		return ""
	}
	return string(bytes.TrimRight(mfr[2:], "\x00"))
}

// Model matches robots of the given type, such as "RT1".
func Model(model string) func(Advertisement) bool {
	return func(a Advertisement) bool { return a.Model == model }
}

// Name matches the robot with the given advertised name.
func Name(name string) func(Advertisement) bool {
	return func(a Advertisement) bool { return a.Name == name }
}

// Dial scans until a robot satisfies match, then connects to it.
func Dial(ctx context.Context, match func(Advertisement) bool) (*Conn, Advertisement, error) {
	scanCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	found := make(chan Advertisement, 1)
	err := Scan(scanCtx, func(a Advertisement) {
		if match(a) {
			select {
			case found <- a:
				cancel()
			default:
			}
		}
	})
	select {
	case a := <-found:
		c, err := Connect(ctx, a.ID)
		return c, a, err
	default:
	}
	if err == nil {
		err = ctx.Err()
	}
	return nil, Advertisement{}, err
}
