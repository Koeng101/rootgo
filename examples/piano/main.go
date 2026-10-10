// Command piano turns the four touch sensors under the robot's top panel
// into piano keys. Press the nose button to quit.
package main

import (
	"context"
	"log"
	"math"
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
	log.Print("touch the top panel to play; press the nose button to quit")

	for ev := range events.C {
		switch e := ev.(type) {
		case root.StopButton:
			return
		case root.Touch:
			var note string
			switch {
			case e.FrontLeft:
				note = "C4"
			case e.FrontRight:
				note = "E4"
			case e.RearLeft:
				note = "G4"
			case e.RearRight:
				note = "C5"
			default:
				continue // released
			}
			hz, err := root.NoteHz(note)
			if err != nil {
				log.Fatal(err)
			}
			if err := r.PlayNote(ctx, uint32(math.Round(hz)), 250*time.Millisecond); err != nil {
				log.Print(err)
			}
		}
	}
}
