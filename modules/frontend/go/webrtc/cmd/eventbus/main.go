package main

import (
	"encoding/json"
	"log"
	"net"
	"time"

	"github.com/farm-ng/core/pkg/eventbus"
)

func prettyPrint(i interface{}) string {
	s, _ := json.MarshalIndent(i, "", "  ")
	return string(s)
}

func main() {
	b := eventbus.NewEventBus(&eventbus.EventBusConfig{
		MulticastGroup: net.UDPAddr{IP: net.ParseIP("239.20.20.21"), Port: 10000},
		ServiceName:    "go-eventbus",
	})
	b.AddSubscriptions([]string{".*"})

	stateTicker := time.NewTicker(1 * time.Second)
	announcementsTicker := time.NewTicker(10 * time.Second)

	go func() {
		for {
			select {
			case <-stateTicker.C:
				log.Println("State", prettyPrint(b.State))
			case <-announcementsTicker.C:
				log.Println("Announcements", prettyPrint(b.Announcements))
			}
		}
	}()
	b.Start()
}
