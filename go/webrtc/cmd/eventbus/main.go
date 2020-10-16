package main

import (
	"encoding/json"
	"github.com/farm-ng/tractor/webrtc/cmd/conf"
	"log"
	"net"
	"time"

	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
)

func prettyPrint(i interface{}) string {
	s, _ := json.MarshalIndent(i, "", "  ")
	return string(s)
}

func main() {
	b := eventbus.NewEventBus(&eventbus.EventBusConfig{
		MulticastGroup: net.UDPAddr{
			IP:   net.ParseIP(conf.EventBusAddr),
			Port: conf.EventBusPort,
		},
		ServiceName: "go-eventbus",
	})

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
