package main

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"

	"github.com/google/uuid"
	"github.com/gorilla/mux"
)

type LogStruct struct {
	Logs []struct {
		Fields []string `json:"fields"`
	} `json:"logs"`
}

func httpmain() {
	if info, err := os.Stat("data"); err != nil || !info.IsDir() {
		os.Mkdir("data", os.ModePerm)
	}

	// Create a new Gorilla Mux router.
	r := mux.NewRouter()

	// Define the GET /session endpoint.
	r.HandleFunc("/session", getSession).Methods("GET")

	// Define the PUT /session/{uuid}/log endpoint.
	r.HandleFunc("/session/{uuid}/log", putSessionLog).Methods("POST")

	// Create a server using the router and bind it to port 51337.
	server := &http.Server{
		Addr:     ":51337",
		Handler:  r,
		ErrorLog: log.Default(),
	}

	// Start the server.
	fmt.Println("HTTP-Server is listening on port 51337...")
	if err := server.ListenAndServe(); err != nil {
		log.Fatal(err)
	}
}

func getSession(w http.ResponseWriter, r *http.Request) {
	// Respond with "hello world" as the response body.
	w.WriteHeader(http.StatusOK)

	randomUUID := uuid.New().String()
	w.Write([]byte(randomUUID))

	fmt.Printf("Gave session id {%s}\n", randomUUID)
}

func putSessionLog(w http.ResponseWriter, r *http.Request) {
	// Parse the UUID from the URL path (you can use it if needed).
	vars := mux.Vars(r)
	uuid := vars["uuid"]

	// Read the request body as text (you can process it if needed).
	body, err := io.ReadAll(r.Body)
	if err != nil {
		http.Error(w, "Failed to read request body", http.StatusBadRequest)
		return
	}

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
	d := map[string]int64{}
	if err := json.Unmarshal(body, &d); err == nil {
		file.WriteString(fmt.Sprintf("%v,%v,%v,%v,%v,%v,%v\r\n", d["time"],
			d["param1"], d["param2"], d["param3"],
			d["param4"], d["param5"], d["param6"]))
	} else {
		fmt.Println(err.Error())
	}

	// Respond with a 200 OK status.
	w.WriteHeader(http.StatusOK)
}
