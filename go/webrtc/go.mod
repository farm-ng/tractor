module github.com/farm-ng/tractor/webrtc

go 1.14

require (
	github.com/farm-ng/tractor/genproto v0.0.0-00010101000000-000000000000
	github.com/golang/protobuf v1.4.2
	github.com/gorilla/mux v1.8.0
	github.com/pion/datachannel v1.4.19
	github.com/pion/rtp v1.6.0
	github.com/pion/webrtc/v3 v3.0.0-beta.1
	github.com/rs/cors v1.7.0
	github.com/twitchtv/twirp v7.1.0+incompatible
	golang.org/x/net v0.0.0-20200625001655-4c5254603344
	golang.org/x/sys v0.0.0-20200831180312-196b9ba8737a
	google.golang.org/protobuf v1.25.0
)

replace github.com/farm-ng/tractor/genproto => ../genproto
