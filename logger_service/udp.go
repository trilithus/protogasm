package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net"
	"os"
)

func udpmain() {
	// listen to incoming udp packets
	pc, err := net.ListenPacket("udp", ":51337")
	if err != nil {
		log.Fatal(err)
	}
	defer pc.Close()

	for {
		buf := make([]byte, 4096)
		n, addr, err := pc.ReadFrom(buf)
		if err != nil {
			continue
		}
		go handleUDPLog(pc, addr, buf[:n])
	}

}

func handleUDPLog(pc net.PacketConn, addr net.Addr, buf []byte) {
	pc.WriteTo([]byte{1}, addr)

	d := map[string]any{}
	if err := json.Unmarshal(buf, &d); err != nil {
		fmt.Println("error unmarshalling struct")
		return
	}

	uuid := d["session"].(string)
	filePath := "./data/" + uuid + ".txt"

	fileIsNew := false
	if info, err := os.Stat(filePath); err != nil || info.IsDir() {
		fileIsNew = true
	}

	file, err := os.OpenFile(filePath, os.O_WRONLY|os.O_APPEND|os.O_CREATE, 0644)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	if fileIsNew {
		file.WriteString("time,motorspeed,current_pressure,avg_pressure,delta_pressure,limit,cooldown\r\n")
	}

	// Perform any necessary processing on the request body here.
	file.WriteString(fmt.Sprintf("%d,%d,%d,%d,%d,%d,%d\r\n", d["time"],
		d["param1"], d["param2"], d["param3"],
		d["param4"], d["param5"], d["param6"]))
}
