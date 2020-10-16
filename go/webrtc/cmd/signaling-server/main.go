package main

import (
	"flag"
	"github.com/farm-ng/tractor/webrtc/cmd/conf"
	"github.com/farm-ng/tractor/webrtc/internal/server"
	"github.com/gorilla/mux"
	"log"
	"net/http"
	"os"
	"path"
	"time"
)

var farmNgRoot string
var serverAddr string
var serverPort string
var proxyEndpoint string

func init() {
	farmNgRoot = os.Getenv("FARM_NG_ROOT")
	if farmNgRoot == "" {
		log.Fatalln("FARM_NG_ROOT must be set.")
	}

	flag.StringVar(&serverPort, "port", "8586", "")
	flag.Parse()

	serverAddr = ":" + serverPort
}

func main() {
	s := &server.SignalingServer{
		SignalingServerWebrtcPort: conf.SignalingServerWebrtcPort,
	}

	srv := &http.Server{
		Handler:      createRouter(s),
		Addr:         serverAddr,
		WriteTimeout: 15 * time.Second,
		ReadTimeout:  15 * time.Second,
	}
	log.Println("Serving frontend and API at:", serverAddr)
	log.Fatal(srv.ListenAndServe())
}

func createRouter(s *server.SignalingServer) *mux.Router {
	router := mux.NewRouter()

	// signaling server
	signaling := server.CreateTwirpHandlerSignaling(s)
	router.PathPrefix("/twirp/").Handler(*signaling)

	// spa
	log.Println(path.Join(farmNgRoot, conf.SpaDistRelPath))
	spa := server.CreateSpaHandler(path.Join(farmNgRoot, conf.SpaDistRelPath))
	router.PathPrefix("/").Handler(*spa)

	return router
}
