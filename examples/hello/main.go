// Command hello connects to the first Root rt1 it finds, says hello and
// reports its battery.
package main

import (
	"context"
	"log"
	"time"

	"github.com/koeng101/rootgo/ble"
	"github.com/koeng101/rootgo/root"
)

func main() {
	ctx := context.Background()

	dialCtx, cancel := context.WithTimeout(ctx, 30*time.Second)
	conn, adv, err := ble.Dial(dialCtx, ble.Model("RT1"))
	cancel()
	if err != nil {
		log.Fatal(err)
	}
	r := root.New(conn)
	defer r.Close()
	log.Printf("connected to %s", adv.Name)

	if err := r.SetLights(ctx, root.LightsSpin, 0, 128, 255); err != nil {
		log.Fatal(err)
	}
	if err := r.Say(ctx, "hello"); err != nil {
		log.Fatal(err)
	}
	b, err := r.Battery(ctx)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("battery at %d%%", b.Percent)
	r.SetLights(ctx, root.LightsOff, 0, 0, 0)
}
