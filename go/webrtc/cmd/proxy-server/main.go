package main

import (
	"flag"
	"github.com/farm-ng/tractor/webrtc/cmd/conf"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/farm-ng/tractor/webrtc/internal/server"
	"github.com/gorilla/mux"
	"log"
	"net/http"
	"os"
	"path"
	"time"
)

const (
	rtpAddr = "127.0.0.1:5000"
	// Set this too low and we see packet loss in chrome://webrtc-internals, and on the network interface (`netstat -suna`)
	// But what should it be? `sysctl net.core.rmem_max`?
	rtpReadBufferSize  = 1024 * 1024 * 8
	maxRtpDatagramSize = 4096
	defaultServerAddr  = ":8585"
)

var farmNgRoot string
var blobstoreRoot string
var serverAddr string
var signalingEndpoint string

func init() {
	farmNgRoot = os.Getenv("FARM_NG_ROOT")
	if farmNgRoot == "" {
		log.Fatalln("FARM_NG_ROOT must be set.")
	}

	blobstoreRoot = os.Getenv("BLOBSTORE_ROOT")
	if blobstoreRoot == "" {
		log.Fatalln("BLOBSTORE_ROOT must be set.")
	}

	// TODO: Should configuration be set by command line args or environment variables?
	//       It should be simpler to use command line args only since
	//         1. security is not a concern
	//         2. there won't be a bunch of devops toolings changing runtime behaviors
	serverAddr = defaultServerAddr
	port := os.Getenv("PORT")
	if port != "" {
		serverAddr = ":" + port
	}

	// e.g. -signal=http://192.168.1.137:8586
	flag.StringVar(&signalingEndpoint, "signal",
		"",
		"If this is provided, this server opens up a datachannel to the signaling server.")
	flag.Parse()
}

func main() {
	// Start the proxy services participating in udp multicast
	proxyService := proxy.StartProxy(rtpAddr, rtpReadBufferSize, maxRtpDatagramSize)

	// TODO: Be able to handle connection failures and reconnects
	// Start a webrtc datachannel with the signaling server
	if signalingEndpoint != "" {
		log.Println("connecting to signaling server", "signal="+signalingEndpoint)
		signalingConn := &proxy.SignalingConn{
			Endpoint: signalingEndpoint,
			Proxy:    proxyService,
		}
		err := signalingConn.ConnectToSignal()
		if err != nil {
			log.Println("cannot connect to signaling server", err)
		} else {
			log.Println("connected to signaling server")
		}
	} else {
		log.Println("not configured to talk to a signaling server")
	}

	// Http server
	srv := &http.Server{
		Handler:      createRouter(proxyService),
		Addr:         serverAddr,
		WriteTimeout: 15 * time.Second,
		ReadTimeout:  15 * time.Second,
	}
	log.Println("Serving frontend and API at:", serverAddr)
	log.Fatal(srv.ListenAndServe())
}

func createRouter(proxy *proxy.Proxy) *mux.Router {
	router := mux.NewRouter()

	// twirp server
	twirp := server.CreateTwirpHandlerProxy(proxy)
	router.PathPrefix("/twirp/").Handler(*twirp)

	// resources route
	blobstore := server.CreateBlobstoreHandler(blobstoreRoot)
	router.PathPrefix("/blobstore/").Handler(http.StripPrefix("/blobstore", *blobstore))

	// spa
	spa := server.CreateSpaHandler(path.Join(farmNgRoot, conf.SpaDistRelPath))
	router.PathPrefix("/").Handler(*spa)

	return router
}
