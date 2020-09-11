package eventbus

import (
	"context"
	"fmt"
	"log"
	"net"
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
	maxDatagramSize = 1024
)

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
}

// NewEventBus returns a new EventBus.
//
// A channel may be provided for event callbacks. This channel must be serviced, or the bus will hang.
// If a channel is provided and `publishAnnouncement` is true, the channel will receive announcement events too.
func NewEventBus(multicastGroup net.UDPAddr, serviceName string, eventChan chan<- *pb.Event, publishAnnouncements bool) *EventBus {
	return &EventBus{
		multicastGroup:       multicastGroup,
		serviceName:          serviceName,
		Announcements:        make(map[string]*pb.Announce),
		announcementsMutex:   &sync.Mutex{},
		State:                make(map[string]*pb.Event),
		eventChan:            eventChan,
		publishAnnouncements: publishAnnouncements,
	}
}

func (bus *EventBus) Start() {
	listenConfig := net.ListenConfig{
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

	c, err := listenConfig.ListenPacket(context.Background(), "udp4", fmt.Sprintf(":%d", bus.multicastGroup.Port))
	if err != nil {
		log.Fatalf("could not create receiveConn: %v", err)
	}
	defer c.Close()
	bus.receiveConn = c.(*net.UDPConn)
	bus.receiveConn.SetReadBuffer(maxDatagramSize)

	// https://godoc.org/golang.org/x/net/ipv4#PacketConn.JoinGroup
	// JoinGroup uses the system assigned multicast interface when ifi is nil,
	// although this is not recommended...
	p := ipv4.NewPacketConn(bus.receiveConn)
	err = p.JoinGroup(nil, &net.UDPAddr{IP: bus.multicastGroup.IP})
	if err != nil {
		log.Fatalf("receiveConn could not join group: %v", err)
	}
	// TODO: LeaveGroup?
	// defer p.LeaveGroup(nil, &net.UDPAddr{IP: bus.multicastGroup.IP})

	c, err = listenConfig.ListenPacket(context.Background(), "udp4", ":0")
	if err != nil {
		log.Fatalf("could not create sendConn: %v", err)
	}
	defer c.Close()
	bus.sendConn = c.(*net.UDPConn)

	// Set the time-to-live for messages to 1 so they do not go past the local network segment.
	p = ipv4.NewPacketConn(bus.sendConn)
	err = p.SetMulticastTTL(1)
	if err != nil {
		log.Fatalf("sendConn could not set multicast TTL: %v", err)
	}

	log.Println("Starting eventbus")
	go bus.announce()
	go bus.handleAnnouncements()
	go bus.handleEvents()
	select {}
}

func (bus *EventBus) SendEvent(e *pb.Event) {
	event_bytes, err := proto.Marshal(e)
	if err != nil {
		log.Fatalln("Could not marshal event: ", e)
	}

	bus.announcementsMutex.Lock()
	for _, a := range bus.Announcements {
		bus.sendConn.WriteTo(event_bytes, &net.UDPAddr{
			IP:   []byte(a.Host),
			Port: int(a.Port),
		})
	}
	bus.announcementsMutex.Unlock()
}

func (bus *EventBus) announce() {
	// TODO: What's the expected behavior here?
	// host, err := net.LookupAddr(bus.sendConn.LocalAddr().(*net.UDPAddr).IP.String())
	// if err != nil {
	// 	log.Fatalln("lookup of sendConn's local addr failed: ", err)
	// }
	host := bus.sendConn.LocalAddr().(*net.UDPAddr).IP.String()
	announce := &pb.Announce{
		Host:    host,
		Port:    int32(bus.sendConn.LocalAddr().(*net.UDPAddr).Port),
		Service: bus.serviceName,
		Stamp:   ptypes.TimestampNow(),
	}
	announceBytes, err := proto.Marshal(announce)
	if err != nil {
		log.Fatalln("announcement encoding failed: ", err)
	}

	for {
		// log.Println("announcing: ", announce)
		bus.sendConn.WriteToUDP(announceBytes, &bus.multicastGroup)
		// log.Println("announcing to: ", bus.multicastGroup.IP, bus.multicastGroup.Port)

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
			bus.sendConn.WriteToUDP(announceBytes, &net.UDPAddr{
				IP:   []byte(a.Host),
				Port: bus.multicastGroup.Port,
			})
			// log.Println("announcing to: ", a.Host, bus.multicastGroup.Port)
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
		srcIP := src.IP.String()
		srcPort := src.Port

		// Ignore self-announcements
		if srcPort == bus.sendConn.LocalAddr().(*net.UDPAddr).Port && isHostLocal(srcIP) {
			// log.Println("ignoring self-announcement: ", srcIP, srcPort)
			continue
		}

		announce := &pb.Announce{}
		now := ptypes.TimestampNow()
		err = proto.Unmarshal(buf[:n], announce)
		if err != nil {
			log.Fatalln("announcement parsing failed:", err, announce)
		}

		if srcPort != int(announce.Port) {
			log.Printf("sender port (%v) does not match announcement: %v", srcPort, announce)
		}
		announce.Host = srcIP
		announce.Port = int32(srcPort)
		announce.RecvStamp = now
		// log.Println("received announcement: ", announce)
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
