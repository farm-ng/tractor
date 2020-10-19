package eventbus

import (
	"context"
	"fmt"
	"log"
	"net"
	"regexp"
	"sync"
	"syscall"
	"time"

	pb "github.com/farm-ng/tractor/genproto"
	"github.com/golang/protobuf/ptypes"
	"golang.org/x/net/ipv4"
	"golang.org/x/sys/unix"
	"google.golang.org/protobuf/proto"
)

const (
	maxDatagramSize = 65507
)

// EventBus ...
type EventBus struct {
	multicastGroup       net.UDPAddr
	serviceName          string
	Announcements        map[string]*pb.Announce
	announcementsMutex   *sync.Mutex
	State                map[string]*pb.Event
	eventChan            chan<- *pb.Event
	publishAnnouncements bool
	receiveConn          *net.UDPConn
	sendConn             *net.UDPConn
	subscriptions        []pb.Subscription
}

// EventBusConfig ...
type EventBusConfig struct {
	MulticastGroup net.UDPAddr
	ServiceName    string
}

// NewEventBus returns a new EventBus.
func NewEventBus(config *EventBusConfig) *EventBus {
	return &EventBus{
		multicastGroup:     config.MulticastGroup,
		serviceName:        config.ServiceName,
		Announcements:      make(map[string]*pb.Announce),
		announcementsMutex: &sync.Mutex{},
		State:              make(map[string]*pb.Event),
	}
}

// EventChannelConfig ...
type EventChannelConfig struct {
	Channel chan<- *pb.Event
	// If true, the channel will receive announcement events too.
	PublishAnnouncements bool
}

// WithEventChannel allows the caller to provide a channel that will receive all events.
// This channel must be serviced, or the bus will hang.
func (bus *EventBus) WithEventChannel(config *EventChannelConfig) *EventBus {
	bus.eventChan = config.Channel
	bus.publishAnnouncements = config.PublishAnnouncements
	return bus
}

// AddSubscriptions ...
func (bus *EventBus) AddSubscriptions(names []string) {
	for _, name := range names {
		bus.subscriptions = append(bus.subscriptions, pb.Subscription{Name: name})
	}
}

// Start ...
func (bus *EventBus) Start() {
	// Shared socket configuration
	socketConfig := net.ListenConfig{
		Control: func(network, address string, c syscall.RawConn) error {
			var err error
			c.Control(func(fd uintptr) {
				err = unix.SetsockoptInt(int(fd), unix.SOL_SOCKET, unix.SO_REUSEADDR, 1)
				if err != nil {
					return
				}

				err = unix.SetsockoptInt(int(fd), unix.SOL_SOCKET, unix.SO_REUSEPORT, 1)
				if err != nil {
					return
				}
			})
			return err
		},
	}

	// Configure receive connection
	receiveConn, err := socketConfig.ListenPacket(
		context.Background(),
		"udp4",
		fmt.Sprintf(":%d", bus.multicastGroup.Port),
	)
	if err != nil {
		log.Fatalf("could not create receiveConn: %v", err)
	}
	defer receiveConn.Close()
	bus.receiveConn = receiveConn.(*net.UDPConn)
	bus.receiveConn.SetReadBuffer(maxDatagramSize)

	// Additional receive connection configuration requires ipv4 wrapper
	receiveConnWrapper := ipv4.NewPacketConn(bus.receiveConn)

	lo, err := net.InterfaceByName("lo")
	if err != nil {
		log.Fatalf("could not find lo interface: %v", err)
	}
	err = receiveConnWrapper.JoinGroup(lo, &net.UDPAddr{IP: bus.multicastGroup.IP})
	if err != nil {
		log.Fatalf("receiveConn could not join group: %v", err)
	}
	defer receiveConnWrapper.LeaveGroup(lo, &net.UDPAddr{IP: bus.multicastGroup.IP})

	// Configure send connection
	sendConn, err := socketConfig.ListenPacket(context.Background(), "udp4", ":0")
	if err != nil {
		log.Fatalf("could not create sendConn: %v", err)
	}
	defer sendConn.Close()
	bus.sendConn = sendConn.(*net.UDPConn)

	// Additional send connection configuration requires ipv4 wrapper
	// Set the time-to-live for messages to 0 so they do not leave localhost.
	sendConnWrapper := ipv4.NewPacketConn(bus.sendConn)
	err = sendConnWrapper.SetMulticastTTL(0)
	if err != nil {
		log.Fatalf("sendConn could not set multicast TTL: %v", err)
	}

	log.Println("Starting eventbus")
	go bus.announce()
	go bus.handleAnnouncements()
	go bus.handleEvents()
	select {}
}

var compiled = make(map[string]*regexp.Regexp)

func compileRegex(s string) *regexp.Regexp {
	if _, ok := compiled[s]; !ok {
		var err error
		compiled[s], err = regexp.Compile(s)
		if err != nil {
			log.Fatalf("could not compile regex %v -- %v:", s, err)
		}
	}
	return compiled[s]
}

func (bus *EventBus) recipients(e *pb.Event) []*pb.Announce {
	var recipientsList []*pb.Announce
	bus.announcementsMutex.Lock()
	defer bus.announcementsMutex.Unlock()
	for _, a := range bus.Announcements {
		for _, s := range a.Subscriptions {
			re := compileRegex(s.Name)
			if re.Match([]byte(e.Name)) {
				recipientsList = append(recipientsList, a)
				continue
			}
		}
	}
	return recipientsList
}

// SendEvent serializes an event, then sends it on the eventbus
func (bus *EventBus) SendEvent(e *pb.Event) {
	recipientsList := bus.recipients(e)
	if len(recipientsList) == 0 {
		return
	}
	bytes, err := proto.Marshal(e)
	if err != nil {
		log.Fatalln("Could not marshal event: ", e)
	}
	for _, a := range recipientsList {
		bus.sendConn.WriteToUDP(bytes, &net.UDPAddr{
			IP:   net.ParseIP(a.Host),
			Port: int(a.Port),
		})
	}
}

func (bus *EventBus) announce() {
	// For now, only announce our local address
	host := "127.0.0.1"
	announce := &pb.Announce{
		Host:    host,
		Port:    int32(bus.sendConn.LocalAddr().(*net.UDPAddr).Port),
		Service: bus.serviceName,
		Stamp:   ptypes.TimestampNow(),
	}
	for i := range bus.subscriptions {
		announce.Subscriptions = append(announce.Subscriptions, &bus.subscriptions[i])
	}
	announceBytes, err := proto.Marshal(announce)
	if err != nil {
		log.Fatalln("announcement encoding failed: ", err)
	}

	for {
		// log.Println("announcing to: ", bus.multicastGroup.IP, bus.multicastGroup.Port)
		bus.sendConn.WriteToUDP(announceBytes, &bus.multicastGroup)

		// Clear stale announcements
		bus.announcementsMutex.Lock()
		for key, a := range bus.Announcements {
			receiveTime, err := ptypes.Timestamp(a.RecvStamp)
			if err != nil {
				log.Fatalln("invalid receive timestamp: ", err)
			}
			if time.Now().Sub(receiveTime) > time.Second*10 {
				log.Println("deleting stale: ", key)
				delete(bus.Announcements, key)
				continue
			}
		}
		bus.announcementsMutex.Unlock()

		time.Sleep(1 * time.Second)
	}
}

func (bus *EventBus) handleAnnouncements() {
	for {
		buf := make([]byte, maxDatagramSize)
		n, src, err := bus.receiveConn.ReadFromUDP(buf)
		if err != nil {
			log.Fatalln("handleAnnouncements ReadFromUDP failed:", err)
		}
		srcIP, srcPort := src.IP.String(), src.Port

		// Ignore self-announcements
		if srcPort == bus.sendConn.LocalAddr().(*net.UDPAddr).Port {
			continue
		}

		// Ignore non-local announcements
		if !isHostLocal(srcIP) {
			log.Println("ignoring non-local announcement: ", srcIP, srcPort)
			continue
		}

		announce := &pb.Announce{}
		err = proto.Unmarshal(buf[:n], announce)
		if err != nil {
			log.Fatalln("announcement parsing failed:", err, announce)
		}

		// Ignore faulty announcements
		if srcPort != int(announce.Port) {
			log.Printf("sender port (%v) does not match announcement: %v", srcPort, announce)
			continue
		}

		// Store the announcement
		announce.RecvStamp = ptypes.TimestampNow()
		log.Println("received announcement: ", announce)
		bus.announcementsMutex.Lock()
		bus.Announcements[src.String()] = announce
		bus.announcementsMutex.Unlock()

		if bus.eventChan != nil && bus.publishAnnouncements {
			event := &pb.Event{}
			event.RecvStamp = announce.RecvStamp
			event.Stamp = announce.Stamp
			event.Name = "ipc/announcement/" + announce.Service
			event.Data, err = ptypes.MarshalAny(announce)
			if err != nil {
				log.Fatalln("marshalling announcement to Any failed: ", err, announce)
			}
			bus.eventChan <- event
		}
	}
}

func (bus *EventBus) handleEvents() {
	for {
		buf := make([]byte, maxDatagramSize)
		n, _, err := bus.sendConn.ReadFrom(buf)
		if err != nil {
			log.Fatalln("handleEvents ReadFrom failed:", err)
		}
		event := &pb.Event{}
		err = proto.Unmarshal(buf[:n], event)
		if err != nil {
			log.Fatalln("event parsing failed:", err, event)
		}
		bus.State[event.Name] = event
		if bus.eventChan != nil {
			bus.eventChan <- event
		}
	}
}

func isHostLocal(host string) bool {
	ifaces, err := net.Interfaces()
	if err != nil {
		log.Fatalln("could not get interfaces:", err)
	}
	localAddresses := []string{"localhost", "0.0.0.0"}
	for _, i := range ifaces {
		addrs, err := i.Addrs()
		if err != nil {
			log.Fatalln("could not get interface address:", err)
		}
		for _, addr := range addrs {
			var ip net.IP
			switch v := addr.(type) {
			case *net.IPNet:
				ip = v.IP
			case *net.IPAddr:
				ip = v.IP
			}
			localAddresses = append(localAddresses, ip.String())
		}
	}

	for _, a := range localAddresses {
		if host == a {
			return true
		}
	}
	return false
}
