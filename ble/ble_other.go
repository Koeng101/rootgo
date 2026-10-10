//go:build !darwin

package ble

import "context"

// Conn is a connection to one robot.
type Conn struct{}

// Scan reports Root robots to fn until ctx is done.
func Scan(ctx context.Context, fn func(Advertisement)) error { return ErrUnsupported }

// Connect connects to the robot with the given advertisement ID.
func Connect(ctx context.Context, id string) (*Conn, error) { return nil, ErrUnsupported }

func (c *Conn) ReadFrame() ([]byte, error) { return nil, ErrUnsupported }
func (c *Conn) WriteFrame([]byte) error    { return ErrUnsupported }
func (c *Conn) Close() error               { return nil }
