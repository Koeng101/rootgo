package root

import (
	"context"
	"math"
)

// clockwiseTurn returns the clockwise rotation in (-180, 180] that turns a
// robot facing from to face to. Headings are counterclockwise-positive.
func clockwiseTurn(from, to float64) float64 {
	d := math.Mod(from-to, 360)
	if d <= -180 {
		d += 360
	} else if d > 180 {
		d -= 360
	}
	return d
}

// TurnTo rotates in place to face heading (degrees, counterclockwise from
// +X, so 90 is the robot's starting direction) and returns the final pose.
func (r *Robot) TurnTo(ctx context.Context, heading float64) (Pose, error) {
	release, err := r.acquire(slotMotion)
	if err != nil {
		return Pose{}, err
	}
	defer release()
	p, err := r.Position(ctx)
	if err != nil {
		return Pose{}, err
	}
	return r.moveHeld(ctx, 12, int32Bytes(decidegrees(clockwiseTurn(p.Heading, heading))))
}

// NavigateTo turns toward the point (x, y) in mm, then drives to it, and
// returns the final pose. Root firmware has no navigation command, so this
// is computed from the robot's position estimate. Other motion commands are
// rejected with ErrBusy until it finishes.
func (r *Robot) NavigateTo(ctx context.Context, x, y int32) (Pose, error) {
	release, err := r.acquire(slotMotion)
	if err != nil {
		return Pose{}, err
	}
	defer release()
	p, err := r.Position(ctx)
	if err != nil {
		return Pose{}, err
	}
	dx, dy := float64(x-p.X), float64(y-p.Y)
	dist := math.Hypot(dx, dy)
	if dist < 1 {
		return p, nil
	}
	bearing := math.Atan2(dy, dx) * 180 / math.Pi
	if turn := clockwiseTurn(p.Heading, bearing); math.Abs(turn) >= 0.1 {
		if _, err := r.moveHeld(ctx, 12, int32Bytes(decidegrees(turn))); err != nil {
			return Pose{}, err
		}
	}
	return r.moveHeld(ctx, 8, int32Bytes(int32(math.Round(dist))))
}
