package server

import (
	"github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/blobstore"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/farm-ng/tractor/webrtc/internal/spa"
	"github.com/rs/cors"
	"log"
	"net/http"
)

func CreateSpaHandler(path string) *spa.Handler {
	spa := spa.Handler{
		StaticPath: path,
		IndexPath:  "index.html",
	}
	return &spa
}

func CreateBlobstoreHandler(blobstoreRoot string) *http.Handler {
	blobstoreCorsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"}, // TODO: Security issue
		AllowedMethods: []string{"GET", "POST"},
		AllowedHeaders: []string{"Content-Type"},
	})
	log.Println("Serving blobstore from ", blobstoreRoot)
	blobstore := blobstoreCorsWrapper.Handler(blobstore.FileServer(&blobstore.RWDir{Dir: http.Dir(blobstoreRoot)}))
	return &blobstore
}

func CreateTwirpHandlerProxy(proxy *proxy.Proxy) *http.Handler {
	server := NewServer(proxy)
	twirpHandler := genproto.NewWebrtcApiServiceServer(server, nil)
	corsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"},
		AllowedMethods: []string{"POST"},
		AllowedHeaders: []string{"Content-Type"},
	})
	api := corsWrapper.Handler(twirpHandler)
	return &api
}

func CreateTwirpHandlerSignaling(server *SignalingServer) *http.Handler {
	twirpHandler := genproto.NewWebrtcApiServiceServer(server, nil)
	corsWrapper := cors.New(cors.Options{
		AllowedOrigins: []string{"*"},
		AllowedMethods: []string{"POST"},
		AllowedHeaders: []string{"Content-Type"},
	})
	api := corsWrapper.Handler(twirpHandler)
	return &api
}
