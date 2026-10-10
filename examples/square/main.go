// Command square drives a square using the robot's position estimate, and
// with -draw lowers the marker to draw it.
package main

import (
	"context"
	"flag"
	"log"
	"os"
	"os/signal"
	"time"

	"github.com/koeng101/rootgo/ble"
	"github.com/koeng101/rootgo/root"
)

func main() {
	side := flag.Int("side", 150, "side length in mm")
	draw := flag.Bool("draw", false, "lower the marker while driving")
	flag.Parse()

	// Ctrl-C cancels the context, which stops the robot mid-move.
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	dialCtx, cancel := context.WithTimeout(ctx, 30*time.Second)
	conn, _, err := ble.Dial(dialCtx, ble.Model("RT1"))
	cancel()
	if err != nil {
		log.Fatal(err)
	}
	r := root.New(conn)
	defer r.Close()

	if err := r.ResetPosition(ctx); err != nil {
		log.Fatal(err)
	}
	if *draw {
		if err := r.SetMarker(ctx, root.MarkerDown); err != nil {
			log.Fatal(err)
		}
		defer r.SetMarker(context.Background(), root.MarkerUp)
	}

	s := int32(*side)
	for _, pt := range [][2]int32{{0, s}, {s, s}, {s, 0}, {0, 0}} {
		p, err := r.NavigateTo(ctx, pt[0], pt[1])
		if err != nil {
			log.Fatal(err)
		}
		log.Printf("at (%d, %d), heading %.1f°", p.X, p.Y, p.Heading)
	}
	if _, err := r.TurnTo(ctx, 90); err != nil {
		log.Fatal(err)
	}
}
