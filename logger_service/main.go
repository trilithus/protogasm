package main

import "time"

func main() {
	go httpmain()
	go udpmain()
	go udpsessionservermain()

	for {
		time.Sleep(1000)
	}
}
