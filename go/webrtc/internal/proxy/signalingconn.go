package proxy

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/common"
	"github.com/golang/protobuf/ptypes"
	"github.com/pion/datachannel"
	"github.com/pion/webrtc/v3"
	"github.com/twitchtv/twirp"
	"google.golang.org/protobuf/proto"
	"log"
	"net/http"
)

const (
	SignalingChanName       = "signaling_data_channel"
	SignalingMessageSizeMax = 1_000_000 // 1 mb TODO: too high
)

type SignalingConn struct {
	Endpoint         string
	Proxy            *Proxy
	pc               *webrtc.PeerConnection
	proxySignalingDc *datachannel.ReadWriteCloser
}

func (s *SignalingConn) readLoop() {
	for {
		buffer := make([]byte, SignalingMessageSizeMax)
		_, err := (*s.proxySignalingDc).Read(buffer)
		if err != nil {
			log.Println("Datachannel closed; Exit the readloop:", err)
			return
		}
		fromClient := &pb.WebrtcPeerConn{}
		proto.Unmarshal(buffer, fromClient)

		s.createClientProxyDataChannel(fromClient)
	}
}

/**
1. Initialize a peer connection between client and proxy
2. Send back a WebrtcPeerConn to signaling server
*/
func (s *SignalingConn) createClientProxyDataChannel(fromClient *pb.WebrtcPeerConn) {
	// Setup a peer connection
	clientSess := webrtc.SessionDescription{}
	b, err := base64.StdEncoding.DecodeString(fromClient.ClientSdp)
	if err != nil {
		panic(err)
	}
	err = json.Unmarshal(b, &clientSess)
	if err != nil {
		panic(err)
	}
	proxySess, err := s.Proxy.AddPeer(clientSess) // client <-> proxy
	if err != nil {
		panic(err)
	}
	b, err = json.Marshal(proxySess)
	if err != nil {
		panic(err)
	}
	proxySessSdpBase64 := base64.StdEncoding.EncodeToString(b)

	// Make PeerConnEvent
	fromProxy := &pb.WebrtcPeerConn{
		Stamp:     ptypes.TimestampNow(),
		ConnId:    fromClient.ConnId,
		ClientSdp: fromClient.ClientSdp,
		ProxySdp:  proxySessSdpBase64,
	}

	// Send PeerConnEvent
	eventBytes, err := proto.Marshal(fromProxy)
	_, err = (*s.proxySignalingDc).Write(eventBytes)
	if err != nil {
		panic(err)
	}
}

// ------------------------------------------------
// Proxy to Signal
// ------------------------------------------------
// Open a data channel to signaling server
func (s *SignalingConn) ConnectToSignal() error {
	log.Println("signaling server endpoint", s.Endpoint)

	settings := webrtc.SettingEngine{}
	settings.DetachDataChannels()
	webrtcApi := webrtc.NewAPI(webrtc.WithSettingEngine(settings))

	peerConnection, err := webrtcApi.NewPeerConnection(webrtc.Configuration{
		ICEServers: []webrtc.ICEServer{
			{
				URLs: []string{"stun:stun.l.google.com:19302"},
			},
		},
	})
	if err != nil {
		return errors.New("cannot open a peer connection")
	}

	// Create a datachannel
	dataChannel, err := peerConnection.CreateDataChannel(SignalingChanName, nil)
	if err != nil {
		return errors.New("cannot create a data channel")
	}

	// Set the handler for ICE connection state
	// This will notify you when the peer has connected/disconnected
	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		log.Printf("ICE Connection State has changed: %s\n", connectionState.String())
	})

	// Register channel opening handling
	dataChannel.OnOpen(func() {
		log.Println("Data channel opened", dataChannel.Label(), dataChannel.ID())
		dc, dErr := dataChannel.Detach()
		if dErr != nil {
			log.Println("cannot detach data channel", dErr)
			return
		}
		s.proxySignalingDc = &dc
		go s.readLoop()
	})

	// Create an offer to send to the browser
	localSess, err := peerConnection.CreateOffer(nil)
	if err != nil {
		return err
	}

	// Create channel that is blocked until ICE Gathering is complete
	gatherComplete := webrtc.GatheringCompletePromise(peerConnection)

	// Sets the LocalDescription, and starts our UDP listeners
	err = peerConnection.SetLocalDescription(localSess)
	if err != nil {
		return err
	}

	// Block until ICE Gathering is complete, disabling trickle ICE
	// we do this because we only can exchange one signaling message
	// in a production application you should exchange ICE Candidates via OnICECandidate
	<-gatherComplete

	// Communicate with the signaling server
	remoteSess, err := findSignalingPeer(&localSess, s.Endpoint)
	if err != nil {
		return err
	}

	// Apply the answer as the remote description
	err = peerConnection.SetRemoteDescription(*remoteSess)
	if err != nil {
		return err
	}

	log.Println("finished setting up datachannel")
	return nil
}

func findSignalingPeer(
	localSess *webrtc.SessionDescription,
	endpoint string,
) (*webrtc.SessionDescription, error) {
	sdp, err := json.Marshal(localSess)
	if err != nil {
		log.Fatal(twirp.NewError(twirp.Internal, "could not generate SDP"))
	}

	// Twirp req
	req := &pb.InitiateSignalingConnectionRequest{
		Sdp: base64.StdEncoding.EncodeToString(sdp),
	}
	client := pb.NewWebrtcApiServiceJSONClient(endpoint, &http.Client{})
	resp, err := client.InitiateSignalingConnection(context.Background(), req)
	if resp == nil || err != nil {
		log.Println("twirp request (InitiateSignalingConnection)", endpoint)
		log.Println("cannot complete the twirp req", req, req, err)
		return nil, err
	}

	// Deserialize
	remoteSess, err := common.DeserializeSess(resp.Sdp)
	if err != nil {
		return nil, err
	}
	return remoteSess, nil
}
