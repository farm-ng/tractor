package api

import (
	"context"
	"encoding/base64"
	"encoding/json"

	pb "github.com/farm-ng/genproto/frontend"
	"github.com/farm-ng/tractor/webrtc/internal/proxy"
	"github.com/pion/webrtc/v3"
	"github.com/twitchtv/twirp"
)

// Server is a Twirp server that exposes a webRTC proxy
type Server struct {
	proxy *proxy.Proxy
}

// NewServer constructs a Server
func NewServer(p *proxy.Proxy) *Server {
	return &Server{
		proxy: p,
	}
}

// InitiatePeerConnection starts the proxy and returns an SDP answer to the client
func (s *Server) InitiatePeerConnection(ctx context.Context,
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

	answer, err := s.proxy.AddPeer(offer)
	if err != nil {
		return nil, twirp.NewError(twirp.Internal, "internal error")
	}

	b, err = json.Marshal(answer)
	if err != nil {
		return nil, twirp.NewError(twirp.Internal, "could not generate SDP")
	}
	return &pb.InitiatePeerConnectionResponse{
		Sdp: base64.StdEncoding.EncodeToString(b),
	}, nil
}
