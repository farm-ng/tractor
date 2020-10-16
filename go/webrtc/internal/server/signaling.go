package server

import (
	"context"
	"errors"
	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/common"
	"github.com/golang/protobuf/ptypes"
	"github.com/pion/datachannel"
	"github.com/pion/webrtc/v3"
	"github.com/twitchtv/twirp"
	"google.golang.org/protobuf/proto"
	"log"
	"time"
)

const (
	SignalingMessageSizeMax = 1_000_000 // 1mb
)

/*
TODO: Allow the signaling server work for multiple backend proxy
      - InitiatePeerConnection and InitiateSignalingConnection need a new param to choose proxy
      - SignalingServer needs to maintain multiple proxy-signaling connections
      - Conns should show more details about proxy-signaling and proxy-client connections

SignalingServer only maintains a single proxy-signaling connection, the most recent connection.
See proxySignalingChan
*/
type SignalingServer struct {
	SignalingServerWebrtcPort uint16
	conns                     map[string]*pb.WebrtcPeerConn
	proxySignalingChan        *datachannel.ReadWriteCloser // Used to send msg to proxy
}

// API endpoint that is only used for debugging
func (s *SignalingServer) Conns(
	ctx context.Context,
	req *pb.ConnsRequest,
) (res *pb.ConnsResponse, err error) {
	resp := &pb.ConnsResponse{
		Size:  int32(len(s.conns)),
		Conns: s.conns,
	}
	return resp, nil
}

// InitiatePeerConnection starts the proxy and returns an SDP answer to the client
func (s *SignalingServer) InitiatePeerConnection(
	ctx context.Context,
	req *pb.InitiatePeerConnectionRequest,
) (*pb.InitiatePeerConnectionResponse, error) {
	peerId := common.UniquePeerID()

	// Send PeerConn msg to proxy
	err := s.sendPeerConnMsg(peerId, req)
	if err != nil {
		log.Println(err)
		return nil, twirp.NewError(twirp.Internal, err.Error())
	}

	// Wait for the peer connection msg, which comes back in the webrtc data channel.
	// The messages in the data channel are stored in s.conns
	sleepDuration := 300 * time.Millisecond
	reqWaitDuration := 10 * time.Second
	elapsed := 0 * time.Second
	for elapsed < reqWaitDuration {
		time.Sleep(300 * time.Millisecond)
		elapsed += sleepDuration
		if val, exists := s.conns[peerId]; exists {
			return &pb.InitiatePeerConnectionResponse{
				Sdp: val.ProxySdp,
			}, nil
		}
	}

	// Failure
	log.Println("cannot find a peer connection", s.conns)
	return nil, twirp.NewError(twirp.Internal, "cannot find a peer connection")
}

func (s *SignalingServer) InitiateSignalingConnection(
	ctx context.Context,
	req *pb.InitiateSignalingConnectionRequest,
) (res *pb.InitiateSignalingConnectionResponse, err error) {
	log.Println("connecting to proxy")

	offer, err := common.DeserializeSess(req.Sdp)
	if err != nil {
		twirpErr := twirp.NewError(twirp.InvalidArgument, "cannot deserialize sdp: "+req.Sdp)
		return nil, twirp.WrapError(twirpErr, err)
	}

	// TODO: This erases the previous proxy<->siganling connection, and establishes a new one.
	answer, err := s.connectToProxy(offer)
	if err != nil {
		return nil, twirp.NewError(twirp.Internal, err.Error())
	}
	answerSdp, err := common.SerializeSess(answer)
	if err != nil {
		return nil, twirp.NewError(twirp.Internal, "cannot serialize local session: "+err.Error())
	}

	resp := &pb.InitiateSignalingConnectionResponse{
		Sdp: answerSdp,
	}
	return resp, nil
}

func (s *SignalingServer) connectToProxy(offer *webrtc.SessionDescription) (*webrtc.SessionDescription, error) {
	settings := webrtc.SettingEngine{}
	settings.DetachDataChannels()

	// TODO: Understand both the inbound and outbound networking requirements
	//       to restrict network traffic setup for the cloud server (security)
	//settings.SetEphemeralUDPPortRange(s.SignalingServerWebrtcPort, s.SignalingServerWebrtcPort)

	webrtcApi := webrtc.NewAPI(
		webrtc.WithSettingEngine(settings))

	peerConnection, err := webrtcApi.NewPeerConnection(webrtc.Configuration{
		ICEServers: []webrtc.ICEServer{
			{
				URLs: []string{"stun:stun.l.google.com:19302"},
			},
		},
	})
	if err != nil {
		return nil, errors.New("cannot create peer connection")
	}

	// Set the handler for ICE connection state
	// This will notify you when the peer has connected/disconnected
	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		log.Printf("ICE Connection State has changed: %s\n", connectionState.String())
	})

	// Register data channel creation handling
	peerConnection.OnDataChannel(func(d *webrtc.DataChannel) {
		log.Printf("New DataChannel %s %d\n", d.Label(), d.ID())

		// Register channel opening handling
		d.OnOpen(func() {
			log.Printf("Data channel openned; label=%s id=%d  \n", d.Label(), d.ID())

			// Detach the data channel
			raw, dErr := d.Detach()
			if dErr != nil {
				log.Println("could not detach data channel", err)
				return
			}
			go s.startReadLoop(&raw)
		})
	})

	// Set the remote SessionDescription
	if err = peerConnection.SetRemoteDescription(*offer); err != nil {
		log.Println(err)
		return nil, err
	}
	// Create answer
	answer, err := peerConnection.CreateAnswer(nil)
	if err != nil {
		log.Println(err)
		return nil, err
	}

	// Create channel that is blocked until ICE Gathering is complete
	gatherComplete := webrtc.GatheringCompletePromise(peerConnection)

	// Set the LocalDescription
	if err = peerConnection.SetLocalDescription(answer); err != nil {
		return nil, err
	}

	// Block until ICE Gathering is complete, disabling trickle ICE
	// we do this because we only can exchange one signaling message
	// in a production application you should exchange ICE Candidates via OnICECandidate
	<-gatherComplete

	return peerConnection.LocalDescription(), nil
}

// Send a WebrtcPeerConn message to proxy
func (s *SignalingServer) sendPeerConnMsg(connId string, req *pb.InitiatePeerConnectionRequest) error {
	if s.proxySignalingChan == nil {
		return errors.New("signaling server has not been properly initialized")
	}

	event := &pb.WebrtcPeerConn{
		Stamp:     ptypes.TimestampNow(),
		ConnId:    connId,
		ClientSdp: req.Sdp,
	}
	eventBytes, err := proto.Marshal(event)
	_, err = (*s.proxySignalingChan).Write(eventBytes)

	return err
}

func (s *SignalingServer) startReadLoop(dc *datachannel.ReadWriteCloser) {
	// Erase previous connection
	s.proxySignalingChan = dc
	s.conns = make(map[string]*pb.WebrtcPeerConn)

	for {
		buffer := make([]byte, SignalingMessageSizeMax)

		// TODO: This #Read is left waiting forever if the channel is closed
		_, err := (*s.proxySignalingChan).Read(buffer)
		if err != nil {
			log.Println("Datachannel closed; Exit the readloop", err)
			return
		}

		fromProxy := &pb.WebrtcPeerConn{}
		proto.Unmarshal(buffer, fromProxy)
		s.conns[fromProxy.ConnId] = fromProxy
	}
}
