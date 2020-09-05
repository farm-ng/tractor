package proxy

import (
	"fmt"
	"log"
	"net"
	"sync"
	"time"

	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
	"github.com/pion/rtp"
	"github.com/pion/webrtc/v3"
	"google.golang.org/protobuf/proto"
)

func uniqueId() int64 {
	return time.Now().UnixNano() / 1e6
}

const maxRtpDatagramSize = 4096

type eventCallback func([]byte) error
type rtpCallback func(*rtp.Packet) error

// Proxy is a webRTC proxy that proxies EventBus events to/from a webRTC data channel and
// RTP packets to a webRTC video channel.
type Proxy struct {
	eventBus            *eventbus.EventBus
	eventSource         chan *pb.Event
	eventCallbacks      map[string]eventCallback
	eventCallbacksMutex *sync.Mutex
	ssrc                uint32
	rtpListener         *net.UDPConn
	rtpCallbacks        map[string]rtpCallback
	rtpCallbacksMutex   *sync.Mutex
}

func NewProxy(eventBus *eventbus.EventBus, eventSource chan *pb.Event, ssrc uint32, rtpListener *net.UDPConn) *Proxy {
	return &Proxy{
		eventBus:            eventBus,
		eventSource:         eventSource,
		eventCallbacks:      make(map[string]eventCallback),
		eventCallbacksMutex: &sync.Mutex{},
		ssrc:                ssrc,
		rtpListener:         rtpListener,
		rtpCallbacks:        make(map[string]rtpCallback),
		rtpCallbacksMutex:   &sync.Mutex{},
	}
}

// Start begins reading continously from the EventBus and RTP stream
func (p *Proxy) Start() {
	// Continuously read from the event bus and publish to all registered event callbacks
	go func() {
		for {
			select {
			case e := <-p.eventSource:
				eventBytes, err := proto.Marshal(e)
				if err != nil {
					log.Println("Could not marshal event: ", e)
					continue
				}
				p.eventCallbacksMutex.Lock()
				for id, cb := range p.eventCallbacks {
					err := cb(eventBytes)
					if err != nil {
						log.Printf("Ending eventbus->datachannel forwarding [%s]\n", id)
						delete(p.eventCallbacks, id)
					}
				}
				p.eventCallbacksMutex.Unlock()
			}
		}
	}()

	// Continuously read from the RTP stream and publish to all registered RTP callbacks
	inboundRTPPacket := make([]byte, maxRtpDatagramSize)
	go func() {
		for {
			n, _, err := p.rtpListener.ReadFrom(inboundRTPPacket)
			if err != nil {
				log.Printf("error during read: %s", err)
				panic(err)
			}

			packet := &rtp.Packet{}
			if err := packet.Unmarshal(inboundRTPPacket[:n]); err != nil {
				panic(err)
			}

			p.rtpCallbacksMutex.Lock()
			for id, cb := range p.rtpCallbacks {
				err := cb(packet)
				if err != nil {
					log.Printf("Ending rtp->videoTrack forwarding [%s]\n", id)
					delete(p.rtpCallbacks, id)
				}
			}
			p.rtpCallbacksMutex.Unlock()
		}
	}()
}

func (p *Proxy) registerEventCallback(id string, cb eventCallback) {
	p.eventCallbacksMutex.Lock()
	p.eventCallbacks[id] = cb
	p.eventCallbacksMutex.Unlock()
}

func (p *Proxy) registerRTPCallback(id string, cb rtpCallback) {
	p.rtpCallbacksMutex.Lock()
	p.rtpCallbacks[id] = cb
	p.rtpCallbacksMutex.Unlock()
}

// AddPeer accepts an offer SDP from a peer, registers callbacks for RTP and EventBus events, and returns an
// answer SDP.
// TODO: Return errors rather than panic
// TODO: Clean up logging
// TODO: Support graceful shutdown
func (p *Proxy) AddPeer(offer webrtc.SessionDescription) webrtc.SessionDescription {
	peerId := fmt.Sprint(uniqueId())
	log.Printf("New peer [%s]\n", peerId)

	// We make our own mediaEngine so we can place the offerer's codecs in it.  This because we must use the
	// dynamic media type from the offerer in our answer. This is not required if we are the offerer
	mediaEngine := webrtc.MediaEngine{}
	err := mediaEngine.PopulateFromSDP(offer)
	if err != nil {
		panic(err)
	}

	// Search for H264 Payload type. If the offer doesn't support H264, exit,
	// since they won't be able to decode anything we send them
	var payloadType uint8
	for _, videoCodec := range mediaEngine.GetCodecsByKind(webrtc.RTPCodecTypeVideo) {
		if videoCodec.Name == "H264" {
			payloadType = videoCodec.PayloadType
			break
		}
	}
	if payloadType == 0 {
		panic("Remote peer does not support H264")
	}

	// Create a SettingEngine and enable Detach.
	// Detach allows us to use a more idiomatic API to read from and write to the data channel.
	// It must be explicitly enabled as a setting since it diverges from the WebRTC API
	// https://github.com/pion/webrtc/blob/master/examples/data-channels-detach/main.go
	settingEngine := webrtc.SettingEngine{}
	settingEngine.DetachDataChannels()

	// Without a persistent signaling channel, the connection will disconnect after 5s and fail after 30s.
	// TODO: Remove when persistent signaling is enabled
	// See https://godoc.org/github.com/pions/webrtc#SettingEngine.SetICETimeouts
	settingEngine.SetICETimeouts(60*time.Minute, 60*time.Second, 2*time.Second)

	// Create a new RTCPeerConnection
	api := webrtc.NewAPI(webrtc.WithSettingEngine(settingEngine), webrtc.WithMediaEngine(mediaEngine))
	peerConnection, err := api.NewPeerConnection(webrtc.Configuration{
		// No STUN servers for now, to ensure candidate pair that's selected operates solely over LAN
		ICEServers: []webrtc.ICEServer{
			// {
			// 	URLs: []string{"stun:stun.l.google.com:19302"},
			// },
		},
	})
	if err != nil {
		panic(err)
	}

	// Create a video track, using the payloadType of the offer and the SSRC of the RTP stream
	videoTrack, err := peerConnection.NewTrack(payloadType, p.ssrc, "video", "pion")
	if err != nil {
		panic(err)
	}
	if _, err = peerConnection.AddTrack(videoTrack); err != nil {
		panic(err)
	}

	// Register a new consumer with the RTP stream and service the received packets
	cb := func(packet *rtp.Packet) error {
		packet.Header.PayloadType = payloadType
		err := videoTrack.WriteRTP(packet)
		if err != nil {
			log.Println("Could not write packet to videoTrack: ", err)
		}
		return err
	}
	p.registerRTPCallback(peerId, cb)

	// Register data channel creation handling
	peerConnection.OnDataChannel(func(d *webrtc.DataChannel) {
		log.Printf("New DataChannel %s %d\n", d.Label(), d.ID())

		// Register channel opening handling
		d.OnOpen(func() {
			log.Printf("Data channel '%s'-'%d' open.\n", d.Label(), d.ID())

			// Detach the data channel
			raw, dErr := d.Detach()
			if dErr != nil {
				panic(dErr)
			}

			// datachannel -> eventbus
			const messageSize = 1024
			go func() {
				log.Printf("Starting datachannel->eventbus forwarding [%s]\n", peerId)
				for {
					buffer := make([]byte, messageSize)
					n, err := raw.Read(buffer)
					if err != nil {
						log.Println("Datachannel closed:", err)
						break
					}

					event := &pb.Event{}
					err = proto.Unmarshal(buffer[:n], event)
					if err != nil {
						log.Println("Received invalid event on the data channel:", err)
						continue
					}

					p.eventBus.SendEvent(event)
				}
				log.Println("Ending datachannel->eventbus forwarding.")
			}()

			// eventbus -> datachannel
			cb := func(eventBytes []byte) error {
				_, err = raw.Write(eventBytes)
				if err != nil {
					log.Println("Could not write event to datachannel : ", err)
				}
				return err
			}
			log.Printf("Starting eventbus->datachannel forwarding [%s]\n", peerId)
			p.registerEventCallback(peerId, cb)
		})
	})

	// Set the handler for ICE connection state
	// This will notify you when the peer has connected/disconnected
	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		log.Printf("Connection State has changed %s \n", connectionState.String())
	})

	// Set the remote SessionDescription
	if err = peerConnection.SetRemoteDescription(offer); err != nil {
		panic(err)
	}

	// Create answer
	answer, err := peerConnection.CreateAnswer(nil)
	if err != nil {
		panic(err)
	}

	// Create channel that is blocked until ICE Gathering is complete
	gatherComplete := webrtc.GatheringCompletePromise(peerConnection)

	// Set the LocalDescription
	if err = peerConnection.SetLocalDescription(answer); err != nil {
		panic(err)
	}

	// Block until ICE Gathering is complete, disabling trickle ICE
	// we do this because we only can exchange one signaling message
	// in a production application you should exchange ICE Candidates via OnICECandidate
	<-gatherComplete

	return *peerConnection.LocalDescription()
}
