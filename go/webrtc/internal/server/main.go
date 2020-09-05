package server

import (
	"context"
	"encoding/base64"
	"encoding/json"

	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/pion/webrtc/v3"
	"github.com/twitchtv/twirp"
)

// Server is a Twirp server that exposes a webRTC proxy
type server struct {
	proxy *proxy.Proxy
}

func NewServer(p *proxy.Proxy) *server {
	return &server{
		proxy: p,
	}
}

// InitiatePeerConnection starts the proxy and returns an SDP answer to the client
func (s *server) InitiatePeerConnection(ctx context.Context,
	req *pb.InitiatePeerConnectionRequest) (res *pb.InitiatePeerConnectionResponse, err error) {

	offer := webrtc.SessionDescription{}
	b, err := base64.StdEncoding.DecodeString(req.Sdp)
	if err != nil {
		return nil, twirp.NewError(twirp.InvalidArgument, "invalid base64")
	}
	err = json.Unmarshal(b, &offer)
	if err != nil {
		return nil, twirp.NewError(twirp.InvalidArgument, "invalid json")
	}

	answer := s.proxy.AddPeer(offer)

	b, err = json.Marshal(answer)
	if err != nil {
		return nil, twirp.NewError(twirp.Internal, "could not generate SDP")
	}
	return &pb.InitiatePeerConnectionResponse{
		Sdp: base64.StdEncoding.EncodeToString(b),
	}, nil
}
