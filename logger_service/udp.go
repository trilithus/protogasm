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
	fmt.Println("udp-server running on 51337")

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
		file.WriteString("time_s,motorspeed,current_pressure,avg_pressure,delta_pressure,limit,cooldown,var,var_target\r\n")
	}

	timestampMS := d["time"].(float64)
	timestampS := timestampMS / 1000.0

	// Perform any necessary processing on the request body here.
	file.WriteString(fmt.Sprintf("%5.3f,%v,%v,%v,%v,%v,%v,%v,%v\r\n", timestampS,
		d["param1"], d["param2"], d["param3"],
		d["param4"], d["param5"], d["param6"],
		d["param7"], d["param8"]))
}
