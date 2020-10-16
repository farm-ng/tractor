package conf

import (
	"log"
)

const (
	EventBusAddr = "239.20.20.21"
	EventBusPort = 10000

	// TODO: @jin dev only; remove before merge
	SpaDistRelPath = "app/frontend/dist"
	//SpaDistRelPath = "build/frontend"

	SignalingServerWebrtcPort = 58127 // explicitly for ease of cloud server security configuration
)

func init() {
	log.SetFlags(log.LstdFlags | log.Llongfile)  // Debug only; could impact performance
}
