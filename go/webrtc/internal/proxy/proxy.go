package proxy

// TODO: Clean up logging
// TODO: Support graceful shutdown

import (
	"errors"
	"fmt"
	"github.com/farm-ng/tractor/webrtc/cmd/conf"
	"github.com/farm-ng/tractor/webrtc/internal/common"
	"log"
	"net"
	"sync"
	"time"

	pb "github.com/farm-ng/tractor/genproto"
	"github.com/farm-ng/tractor/webrtc/internal/eventbus"
	"github.com/golang/protobuf/ptypes"
	"github.com/pion/rtp"
	"github.com/pion/webrtc/v3"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"
)

func uniqueID() uint32 {
	return uint32(time.Now().UnixNano() / 1e6)
}

type rtpCallback func(*rtp.Packet) error

// RtpProxy proxies RTP traffic to registered callbacks
type RtpProxy struct {
	config         RtpProxyConfig
	callbacks      map[string]rtpCallback
	callbacksMutex *sync.Mutex
	// Use a consistent SSRC, even if multicast RTP traffic is from changing sources
	SSRC            uint32
	packetsReceived uint32
}

// RtpProxyConfig configures an RtpProxy
type RtpProxyConfig struct {
	ListenAddr      string
	ReadBufferSize  int
	MaxDatagramSize int
}

// NewRtpProxy constructs an RtpProxy
func NewRtpProxy(config *RtpProxyConfig) *RtpProxy {
	return &RtpProxy{
		config:         *config,
		callbacks:      make(map[string]rtpCallback),
		callbacksMutex: &sync.Mutex{},
		SSRC:           uniqueID(),
	}
}

func (p *RtpProxy) start() error {
	// Open a UDP Listener for RTP Packets
	resolvedListenAddr, err := net.ResolveUDPAddr("udp", p.config.ListenAddr)
	if err != nil {
		return err
	}
	listener, err := net.ListenUDP("udp", resolvedListenAddr)
	if err != nil {
		return err
	}
	listener.SetReadBuffer(p.config.ReadBufferSize)
	log.Println("RTP proxy waiting for RTP Packets at: ", p.config.ListenAddr)

	// Periodically log our state
	t := time.NewTicker(10 * time.Second)
	go func() {
		for {
			select {
			case <-t.C:
				p.callbacksMutex.Lock()
				numCallbacks := len(p.callbacks)
				p.callbacksMutex.Unlock()
				log.Printf("[RtpProxy] ssrc=%d, packetsReceived/10s=%d, # callbacks=%d", p.SSRC, p.packetsReceived, numCallbacks)
				p.packetsReceived = 0
			}
		}
	}()

	// Continuously read from the RTP stream and publish to all registered callbacks
	inboundRTPPacket := make([]byte, p.config.MaxDatagramSize)
	for {
		n, _, err := listener.ReadFrom(inboundRTPPacket)
		if err != nil {
			log.Println("RTP proxy error during read: ", err)
			continue
		}

		packet := &rtp.Packet{}
		if err := packet.Unmarshal(inboundRTPPacket[:n]); err != nil {
			log.Println("RTP proxy error during unmarshal: ", err)
			continue
		}
		p.packetsReceived++

		p.callbacksMutex.Lock()
		for id, cb := range p.callbacks {
			err := cb(packet)
			if err != nil {
				log.Printf("[%s] Error proxying RTP: %s \n", id, err)
				p.unregisterCallbackThreadUnsafe(id)
			}
		}
		p.callbacksMutex.Unlock()
	}
}

func (p *RtpProxy) registerCallback(id string, cb rtpCallback) {
	p.callbacksMutex.Lock()
	p.registerCallbackThreadUnsafe(id, cb)
	p.callbacksMutex.Unlock()
}

func (p *RtpProxy) registerCallbackThreadUnsafe(id string, cb rtpCallback) {
	log.Printf("[%s] Registering RTP callback \n", id)
	p.callbacks[id] = cb
}

func (p *RtpProxy) unregisterCallback(id string) {
	p.callbacksMutex.Lock()
	p.unregisterCallbackThreadUnsafe(id)
	p.callbacksMutex.Unlock()
}

func (p *RtpProxy) unregisterCallbackThreadUnsafe(id string) {
	log.Printf("[%s] Unregistering RTP callback \n", id)
	delete(p.callbacks, id)
}

type eventCallback func([]byte) error

// EventBusProxy proxies EventBus traffic to registered callbacks
type EventBusProxy struct {
	config         EventBusProxyConfig
	callbacks      map[string]eventCallback
	callbacksMutex *sync.Mutex
}

// EventBusProxyConfig configures an EventBusProxy
type EventBusProxyConfig struct {
	EventBus    *eventbus.EventBus
	EventSource chan *pb.Event
}

// NewEventBusProxy constructs an EventBusProxy
func NewEventBusProxy(config *EventBusProxyConfig) *EventBusProxy {
	return &EventBusProxy{
		config:         *config,
		callbacks:      make(map[string]eventCallback),
		callbacksMutex: &sync.Mutex{},
	}
}

func (p *EventBusProxy) registerCallback(id string, cb eventCallback) {
	p.callbacksMutex.Lock()
	p.registerCallbackThreadUnsafe(id, cb)
	p.callbacksMutex.Unlock()
}

func (p *EventBusProxy) registerCallbackThreadUnsafe(id string, cb eventCallback) {
	log.Printf("[%s] Registering EventBus callback \n", id)
	p.callbacks[id] = cb
}

func (p *EventBusProxy) unregisterCallback(id string) {
	p.callbacksMutex.Lock()
	p.unregisterCallbackThreadUnsafe(id)
	p.callbacksMutex.Unlock()
}

func (p *EventBusProxy) unregisterCallbackThreadUnsafe(id string) {
	log.Printf("[%s] Unregistering EventBus callback \n", id)
	delete(p.callbacks, id)
}

func (p *EventBusProxy) start() {
	go p.config.EventBus.Start()

	// Continuously read from the event bus and publish to all registered event callbacks
	for {
		select {
		case e := <-p.config.EventSource:
			eventBytes, err := proto.Marshal(e)
			if err != nil {
				log.Println("Could not marshal event: ", e)
				continue
			}
			p.callbacksMutex.Lock()
			for id, cb := range p.callbacks {
				err := cb(eventBytes)
				if err != nil {
					log.Printf("[%s] Error proxying eventbus->datachannel: %s \n", id, err)
					p.unregisterCallbackThreadUnsafe(id)
				}
			}
			p.callbacksMutex.Unlock()
		}
	}
}

func (p *EventBusProxy) sendBytes(bytes []byte) {
	p.config.EventBus.SendBytes(bytes)
}

// Proxy proxies EventBus events to/from a webRTC data channel and RTP packets to a webRTC video channel.
type Proxy struct {
	eventBusProxy *EventBusProxy
	rtpProxy      *RtpProxy
	WebrtcConns   map[string]*pb.WebrtcPeerConn
}

// NewProxy constructs a Proxy
func NewProxy(eventBusProxy *EventBusProxy, rtpProxy *RtpProxy) *Proxy {
	return &Proxy{
		eventBusProxy: eventBusProxy,
		rtpProxy:      rtpProxy,
		WebrtcConns:   make(map[string]*pb.WebrtcPeerConn),
	}
}

// Start begins reading / writing continously from / to the EventBus and RTP stream
func (p *Proxy) Start() {
	// TODO: Use the errgroup pattern (https://stackoverflow.com/a/55122595) to break if either errors
	go p.eventBusProxy.start()
	go p.rtpProxy.start()
}

// TODO: Be able to clean up the webrtc connection after they are completed/closed/failed
// AddPeer accepts an offer SDP from a peer, registers callbacks for RTP and EventBus events, and returns an
// answer SDP.
func (p *Proxy) AddPeer(offer webrtc.SessionDescription) (*webrtc.SessionDescription, error) {
	peerID := fmt.Sprint(uniqueID())
	log.Printf("[%s] Added as new peer\n", peerID)

	// We make our own mediaEngine so we can place the offerer's codecs in it.  This because we must use the
	// dynamic media type from the offerer in our answer. This is not required if we are the offerer
	mediaEngine := webrtc.MediaEngine{}
	err := mediaEngine.PopulateFromSDP(offer)
	if err != nil {
		return nil, err
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
		return nil, errors.New("remote peer does not support H264")
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
		ICEServers: []webrtc.ICEServer{
			{
				URLs: []string{"stun:stun.l.google.com:19302"},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	// Create a video track, using the payloadType of the offer and the SSRC of the RTP proxy
	ssrc := p.rtpProxy.SSRC
	videoTrack, err := peerConnection.NewTrack(payloadType, ssrc, "video", "pion")
	if err != nil {
		return nil, err
	}
	if _, err = peerConnection.AddTrack(videoTrack); err != nil {
		return nil, err
	}

	// Service received RTP packets
	cb := func(packet *rtp.Packet) error {
		packet.Header.PayloadType = payloadType

		// Rewrite the SSRC so it looks like one continuous stream to the peer
		packet.SSRC = p.rtpProxy.SSRC
		return videoTrack.WriteRTP(packet)
	}
	p.rtpProxy.registerCallback(peerID, cb)

	// Handle data channel creation
	peerConnection.OnDataChannel(func(d *webrtc.DataChannel) {
		log.Printf("[%s] New DataChannel %s %d\n", peerID, d.Label(), d.ID())

		d.OnOpen(func() {
			log.Printf("[%s] Data channel '%s'-'%d' open\n", peerID, d.Label(), d.ID())

			// Detach the data channel
			raw, dErr := d.Detach()
			if dErr != nil {
				log.Printf("[%s] Could not detach data channel: %s\n", peerID, err)
				return
			}

			// Service received data channel events, forwarding them to the event bus
			const messageSize = 1024
			go func() {
				log.Printf("[%s] Starting datachannel->eventbus forwarding\n", peerID)
				for {
					buffer := make([]byte, messageSize)
					n, err := raw.Read(buffer) // TODO: This blocks forever when if the connection is closed
					if err != nil {
						log.Printf("[%s] Datachannel closed: %s\n", peerID, err)
						break
					}
					// TODO: A major security hole
					p.eventBusProxy.sendBytes(buffer[:n])
				}
				log.Printf("[%s] Ending datachannel->eventbus forwarding", peerID)

				// This is the most reliable signal I've been able to find so far that a peer
				// has disappeared. End RTP forwarding as well.
				log.Printf("[%s] Ending rtp->videostream forwarding", peerID)
				p.rtpProxy.unregisterCallback(peerID)
			}()

			// Service received EventBus events, forwarding them to the data channel
			cb := func(eventBytes []byte) error {
				_, err = raw.Write(eventBytes)
				return err
			}
			log.Printf("[%s] Starting eventbus->datachannel forwarding\n", peerID)
			p.eventBusProxy.registerCallback(peerID, cb)

			// Immediately announce ourselves to fill the data channel's write buffers
			announceSelf := func(t *timestamppb.Timestamp) error {
				event := &pb.Event{}
				event.RecvStamp = t
				event.Stamp = t
				event.Name = "ipc/announcement/webrtc-proxy"
				event.Data, err = ptypes.MarshalAny(&pb.Announce{
					Service: "webrtc-proxy",
					Stamp:   t,
				})

				eventBytes, err := proto.Marshal(event)
				_, err = raw.Write(eventBytes)
				return err
			}
			for i := 0; i < 100; i++ {
				err := announceSelf(ptypes.TimestampNow())
				if err != nil {
					break
				}
			}

			// And continue announcing periodically
			t := time.NewTicker(1 * time.Second)
			go func() {
				for {
					select {
					case <-t.C:
						err := announceSelf(ptypes.TimestampNow())
						if err != nil {
							return
						}
					}
				}
			}()
		})
	})

	// Handle notifications that peer has connected/disconnected
	peerConnection.OnICEConnectionStateChange(func(s webrtc.ICEConnectionState) {
		log.Printf("[%s] Connection State has changed %s \n", peerID, s.String())
		if s == webrtc.ICEConnectionStateClosed {
			p.eventBusProxy.unregisterCallback(peerID)
			p.rtpProxy.unregisterCallback(peerID)
		}
	})

	// Set the remote SessionDescription
	if err = peerConnection.SetRemoteDescription(offer); err != nil {
		return nil, err
	}

	// Create answer
	answer, err := peerConnection.CreateAnswer(nil)
	if err != nil {
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

	localSess := peerConnection.LocalDescription()
	p.storeConn(peerID, &offer, localSess)
	return localSess, nil
}

func StartProxy(rtpAddr string, rtpReadBufferSize int, maxRtpDatagramSize int) *Proxy {
	// Create EventBus proxy
	eventChan := make(chan *pb.Event)
	eventBus := eventbus.NewEventBus(&eventbus.EventBusConfig{
		MulticastGroup: net.UDPAddr{
			IP:   net.ParseIP(conf.EventBusAddr),
			Port: conf.EventBusPort,
		},
		ServiceName: "webrtc-proxy",
	}).WithEventChannel(&eventbus.EventChannelConfig{
		Channel:              eventChan,
		PublishAnnouncements: true,
	})
	eventBusProxy := NewEventBusProxy(&EventBusProxyConfig{
		EventBus:    eventBus,
		EventSource: eventChan,
	})

	// Create Rtp proxy
	rtpProxy := NewRtpProxy(&RtpProxyConfig{
		ListenAddr:      rtpAddr,
		ReadBufferSize:  rtpReadBufferSize,
		MaxDatagramSize: maxRtpDatagramSize,
	})

	// Start webRTC proxy
	p := NewProxy(eventBusProxy, rtpProxy)
	p.Start()

	return p
}

func (p *Proxy) storeConn(peerId string, clientSess, proxySession *webrtc.SessionDescription) {
	clientSdp, _ := common.SerializeSess(clientSess)
	proxySdp, _ := common.SerializeSess(proxySession)

	p.WebrtcConns[peerId] = &pb.WebrtcPeerConn{
		Stamp:     ptypes.TimestampNow(),
		ConnId:    peerId,
		ClientSdp: clientSdp,
		ProxySdp:  proxySdp,
	}
}
