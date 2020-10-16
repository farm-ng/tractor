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

// ProxyServer is a Twirp server that exposes a webRTC proxy
type ProxyServer struct {
	proxy *proxy.Proxy
}

// NewServer constructs a ProxyServer
func NewServer(p *proxy.Proxy) *ProxyServer {
	return &ProxyServer{
		proxy: p,
	}
}

// InitiatePeerConnection starts the proxy and returns an SDP answer to the client
func (s *ProxyServer) InitiatePeerConnection(ctx context.Context,
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

func (s *ProxyServer) InitiateSignalingConnection(
	ctx context.Context,
	req *pb.InitiateSignalingConnectionRequest,
) (res *pb.InitiateSignalingConnectionResponse, err error) {
	return nil, twirp.NewError(twirp.Internal, "not implemented; signaling server only server")
}

func (s *ProxyServer) Conns(ctx context.Context, req *pb.ConnsRequest) (res *pb.ConnsResponse, err error) {
	resp := &pb.ConnsResponse{
		Size:  int32(len(s.proxy.WebrtcConns)),
		Conns: s.proxy.WebrtcConns,
	}
	return resp, nil
}
