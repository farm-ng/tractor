package main

import (
	"log"
	"net"
	"net/http"

	"github.com/pion/rtp"
	"github.com/rs/cors"

	"github.com/farm-ng/tractor/genproto"
	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/farm-ng/tractor/webrtc/internal/server"
)

const (
	eventBusAddr = "239.20.20.21:10000"
	rtpAddr      = "239.20.20.20:5000"
	serverAddr   = ":9900"
)

func main() {

	// Create eventbus client
	eventChan := make(chan *pb.Event)
	eventBus := eventbus.NewEventBus(eventBusAddr, eventChan)
	go eventBus.Start()

	// Create RTP client
	// TODO : Encapsulate this better

	// Open a UDP Listener for RTP Packets
	const maxRtpDatagramSize = 4096
	resolvedRtpAddr, err := net.ResolveUDPAddr("udp", rtpAddr)
	if err != nil {
		panic(err)
	}
	listener, err := net.ListenMulticastUDP("udp", nil, resolvedRtpAddr)
	if err != nil {
		panic(err)
	}
	listener.SetReadBuffer(maxRtpDatagramSize)
	log.Println("Waiting for RTP Packets at:", rtpAddr)

	// Listen for a single RTP Packet, we need this to determine the SSRC
	inboundRTPPacket := make([]byte, maxRtpDatagramSize)
	n, _, err := listener.ReadFromUDP(inboundRTPPacket)
	if err != nil {
		panic(err)
	}

	// Unmarshal the incoming packet
	packet := &rtp.Packet{}
	if err = packet.Unmarshal(inboundRTPPacket[:n]); err != nil {
		panic(err)
	}
	log.Println("RTP packet received. Starting server.")

	proxy := proxy.NewProxy(eventBus, eventChan, packet.SSRC, listener)
	proxy.Start()
	server := server.NewServer(proxy)
	twirpHandler := genproto.NewWebRTCProxyServiceServer(server, nil)

	// Enable CORS
	corsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"},
		AllowedMethods: []string{"POST"},
		AllowedHeaders: []string{"Content-Type"},
	})

	http.ListenAndServe(serverAddr, corsWrapper.Handler(twirpHandler))
}