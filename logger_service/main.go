package main

import (
	"net/http"
	_ "net/http/pprof"
	"time"
)

func main() {
	go httpmain()
	go udpmain()
	go udpsessionservermain()
	go http.ListenAndServe("0.0.0.0:6060", nil)

	for {
		time.Sleep(time.Millisecond * 1000)
	}
}
