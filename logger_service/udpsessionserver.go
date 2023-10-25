package main

import (
	"fmt"
	"log"
	"net"

	"github.com/google/uuid"
)

func udpsessionservermain() {
	// listen to incoming udp packets
	pc, err := net.ListenPacket("udp", ":51338")
	if err != nil {
		log.Fatal(err)
	}
	defer pc.Close()
	fmt.Println("udp-session-server running on 51338")

	for {
		buf := make([]byte, 4096)
		n, addr, err := pc.ReadFrom(buf)
		if err != nil {
			continue
		}
		go handleUDPSessionRequest(pc, addr, buf[:n])
	}

}

func handleUDPSessionRequest(pc net.PacketConn, addr net.Addr, buf []byte) {
	randomUUID := uuid.New().String()
	pc.WriteTo(append([]byte(randomUUID), 0), addr)
	fmt.Printf("Gave session id {%s}\n", randomUUID)
}
