// Command bumper drives forward until it bumps into something, then backs
// up, turns away from the obstacle and carries on. Press the nose button
// to quit.
package main

import (
	"context"
	"errors"
	"log"
	"time"

	"github.com/koeng101/rootgo/ble"
	"github.com/koeng101/rootgo/root"
)

func main() {
	ctx := context.Background()

	dialCtx, cancel := context.WithTimeout(ctx, 30*time.Second)
	conn, _, err := ble.Dial(dialCtx, ble.Model("RT1"))
	cancel()
	if err != nil {
		log.Fatal(err)
	}
	r := root.New(conn)
	defer r.Close()

	events := r.Subscribe(16)
	defer events.Close()

	if err := r.SetWheelSpeeds(ctx, 100, 100); err != nil {
		log.Fatal(err)
	}
	for ev := range events.C {
		switch e := ev.(type) {
		case root.StopButton:
			log.Print("nose button pressed, quitting")
			return
		case root.Bumpers:
			if !e.Left && !e.Right {
				continue // released
			}
			log.Printf("bumped (left=%v right=%v)", e.Left, e.Right)
			turn := 90.0 // clockwise, away from a left bump
			if e.Right && !e.Left {
				turn = -90
			}
			_, err := r.Drive(ctx, -50)
			if err == nil {
				_, err = r.Rotate(ctx, turn)
			}
			if errors.Is(err, root.ErrStopped) {
				log.Print("nose button pressed, quitting")
				return
			}
			if err != nil {
				log.Fatal(err)
			}
			if err := r.SetWheelSpeeds(ctx, 100, 100); err != nil {
				log.Fatal(err)
			}
		}
	}
	log.Printf("disconnected: %v", r.Err())
}
