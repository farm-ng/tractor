package main

import (
	"log"
	"net"
	"net/http"
	"os"
	"path"
	"time"

	"github.com/gorilla/mux"
	"github.com/rs/cors"

	"github.com/farm-ng/tractor/genproto"
	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/api"
	"github.com/farm-ng/tractor/webrtc/internal/blobstore"
	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
	"github.com/farm-ng/tractor/webrtc/internal/metrics"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/farm-ng/tractor/webrtc/internal/spa"
)

const (
	eventBusAddr = "239.20.20.21"
	eventBusPort = 10000
	rtpAddr      = "127.0.0.1:5000"
	// Set this too low and we see packet loss in chrome://webrtc-internals, and on the network interface (`netstat -suna`)
	// But what should it be? `sysctl net.core.rmem_max`?
	rtpReadBufferSize  = 1024 * 1024 * 8
	maxRtpDatagramSize = 4096
	defaultServerAddr  = ":8585"
)

func main() {
	// Create EventBus proxy
	eventChan := make(chan *pb.Event)
	eventBus := eventbus.NewEventBus(&eventbus.EventBusConfig{
		MulticastGroup: (net.UDPAddr{IP: net.ParseIP(eventBusAddr), Port: eventBusPort}),
		ServiceName:    "webrtc-proxy",
	}).WithEventChannel(&eventbus.EventChannelConfig{
		Channel:              eventChan,
		PublishAnnouncements: true,
	})

	eventBusProxy := proxy.NewEventBusProxy((&proxy.EventBusProxyConfig{EventBus: eventBus, EventSource: eventChan}))

	// Create RTP proxy
	rtpProxy := proxy.NewRtpProxy(&proxy.RtpProxyConfig{ListenAddr: rtpAddr, ReadBufferSize: rtpReadBufferSize, MaxDatagramSize: maxRtpDatagramSize})

	// Create and start webRTC proxy
	proxy := proxy.NewProxy(eventBusProxy, rtpProxy)
	proxy.Start()

	// Create API handler
	server := api.NewServer(proxy)
	twirpHandler := genproto.NewWebRTCProxyServiceServer(server, nil)
	corsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"},
		AllowedMethods: []string{"POST"},
		AllowedHeaders: []string{"Content-Type"},
	})
	api := corsWrapper.Handler(twirpHandler)

	// Create SPA handler
	farmNgRoot := os.Getenv("FARM_NG_ROOT")
	if farmNgRoot == "" {
		log.Fatalln("FARM_NG_ROOT must be set.")
	}
	spa := spa.Handler{StaticPath: path.Join(farmNgRoot, "build/frontend"), IndexPath: "index.html"}

	// Create blobstore handler
	blobstoreRoot := os.Getenv("BLOBSTORE_ROOT")
	if blobstoreRoot == "" {
		log.Fatalln("BLOBSTORE_ROOT must be set.")
	}
	blobstoreCorsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"}, // TODO: Security issue
		AllowedMethods: []string{"GET", "POST"},
		AllowedHeaders: []string{"Content-Type"},
	})
	log.Println("Serving blobstore from ", blobstoreRoot)
	blobstore := blobstoreCorsWrapper.Handler(blobstore.FileServer(&blobstore.RWDir{Dir: http.Dir(blobstoreRoot)}))

	// Create metrics server handler
	metricsServerEventBus := eventbus.NewEventBus(&eventbus.EventBusConfig{
		MulticastGroup: (net.UDPAddr{IP: net.ParseIP(eventBusAddr), Port: eventBusPort}),
		ServiceName:    "metrics-server",
	})
	go metricsServerEventBus.Start()
	metricsServer := &metrics.Server{EventBus: metricsServerEventBus}

	// Serve the API and frontend
	serverAddr := defaultServerAddr
	port := os.Getenv("PORT")
	if port != "" {
		serverAddr = ":" + port
	}
	router := mux.NewRouter()
	router.PathPrefix("/twirp/").Handler(api)
	router.PathPrefix("/receive_metrics").Handler(metricsServer)
	router.PathPrefix("/blobstore/").Handler(http.StripPrefix("/blobstore", blobstore))
	router.PathPrefix("/").Handler(spa)
	srv := &http.Server{
		Handler:      router,
		Addr:         serverAddr,
		WriteTimeout: 15 * time.Second,
		ReadTimeout:  15 * time.Second,
	}
	log.Println("Serving frontend and API at:", serverAddr)
	log.Fatal(srv.ListenAndServe())
}
