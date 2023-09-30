package main

import (
	"fmt"
	"io"
	"log"
	"net/http"

	"github.com/google/uuid"
	"github.com/gorilla/mux"
)

type LogStruct struct {
	Logs []struct {
		Fields []string `json:"fields"`
	} `json:"logs"`
}

func main() {
	// Create a new Gorilla Mux router.
	r := mux.NewRouter()

	// Define the GET /session endpoint.
	r.HandleFunc("/session", getSession).Methods("GET")

	// Define the PUT /session/{uuid}/log endpoint.
	r.HandleFunc("/session/{uuid}/log", putSessionLog).Methods("POST")

	// Create a server using the router and bind it to port 51337.
	server := &http.Server{
		Addr:    ":51337",
		Handler: r,
	}

	// Start the server.
	fmt.Println("Server is listening on port 51337...")
	if err := server.ListenAndServe(); err != nil {
		log.Fatal(err)
	}
}

func getSession(w http.ResponseWriter, r *http.Request) {
	// Respond with "hello world" as the response body.
	w.WriteHeader(http.StatusOK)

	randomUUID := uuid.New().String()
	w.Write([]byte(randomUUID))
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

	// Perform any necessary processing on the request body here.
	fmt.Println(string(uuid))
	fmt.Println(string(body))

	// Respond with a 200 OK status.
	w.WriteHeader(http.StatusOK)
}
