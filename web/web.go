package web

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/rustyoz/rustygps/gps"
	"github.com/rustyoz/rustygps/implement"
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true // Be careful with this in production
	},
}

type Client struct {
	conn *websocket.Conn
	mu   sync.Mutex
}

var clients = make(map[*Client]bool)
var clientsMu sync.Mutex

// TractorControls represents the control input from the web interface
type TractorControls struct {
	Speed         float64 `json:"speed"`
	SteeringAngle float64 `json:"steeringAngle"`
}

var currentPosition gps.Position

func handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		http.Error(w, "Could not upgrade connection", http.StatusInternalServerError)
		return
	}

	client := &Client{conn: conn}

	clientsMu.Lock()
	clients[client] = true
	clientsMu.Unlock()

	defer func() {
		conn.Close()
		clientsMu.Lock()
		delete(clients, client)
		clientsMu.Unlock()
	}()

	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			break
		}

		// Handle control messages from client
		var control struct {
			Type          string  `json:"type"`
			Speed         float64 `json:"speed"`
			SteeringAngle float64 `json:"steeringAngle"`
		}

		if err := json.Unmarshal(message, &control); err != nil {
			continue
		}

		if control.Type == "control" {
			// Convert speed from km/h to m/s
			speedMS := control.Speed / 3.6
			// Convert steering angle to radians
			steeringRad := control.SteeringAngle * (3.14159 / 180.0)

			// Update the tractor simulation
			gps.UpdateTractorControls(speedMS, steeringRad)
		}
	}
}

// BroadcastPosition sends position updates to all connected clients
func BroadcastPosition(pos *gps.Position) {
	// Get implement position
	implPos := implement.GetPosition()

	message := struct {
		Type             string  `json:"type"`
		Lat              float64 `json:"Lat"`
		Lon              float64 `json:"Lon"`
		Heading          float64 `json:"Heading"`
		Speed            float64 `json:"Speed"`
		Time             string  `json:"Time"`
		ImplementLat     float64 `json:"ImplementLat"`
		ImplementLon     float64 `json:"ImplementLon"`
		ImplementHeading float64 `json:"ImplementHeading"`
	}{
		Type:             "position",
		Lat:              pos.Lat,
		Lon:              pos.Lon,
		Heading:          pos.Heading,
		Speed:            pos.Speed,
		Time:             pos.Time.Format(time.RFC3339),
		ImplementLat:     implPos.Lat,
		ImplementLon:     implPos.Lon,
		ImplementHeading: implPos.Heading,
	}

	data, err := json.Marshal(message)
	if err != nil {
		return
	}

	clientsMu.Lock()
	for client := range clients {
		client.mu.Lock()
		err := client.conn.WriteMessage(websocket.TextMessage, data)
		client.mu.Unlock()
		if err != nil {
			// Handle error or remove client
			delete(clients, client)
		}
	}
	clientsMu.Unlock()
}

func Run(port string) {

	// print the port
	//	log.Println("Web server running on port", port)

	// serve the tractor location at "/gps/position"
	http.HandleFunc("/gps/position", func(w http.ResponseWriter, r *http.Request) {

		// get the gps position
		position := gps.GetPosition()

		// serve the position as json
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(position)
	})

	// handle tractor steering controls
	http.HandleFunc("/tractor/steer", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
			return
		}

		// Read the request body
		body, err := io.ReadAll(r.Body)
		if err != nil {
			http.Error(w, "Error reading request body", http.StatusBadRequest)
			return
		}

		// Parse the controls
		var controls TractorControls
		if err := json.Unmarshal(body, &controls); err != nil {
			http.Error(w, "Error parsing JSON", http.StatusBadRequest)
			return
		}

		// Convert speed from km/h to m/s
		speedMS := controls.Speed / 3.6
		// Convert steering angle to radians
		steeringRad := controls.SteeringAngle * (3.14159 / 180.0)

		// Update the tractor simulation
		gps.UpdateTractorControls(speedMS, steeringRad)

		// Send success response
		w.WriteHeader(http.StatusOK)
	})

	// serve the static files
	http.Handle("/web/static/", http.StripPrefix("/web/static/", http.FileServer(http.Dir("web/static/"))))

	// serve the web interface
	http.HandleFunc("/ws", handleWebSocket)

	// serve position updates to websocket at 10hz
	go func() {
		for {
			currentPosition = gps.GetPosition()
			BroadcastPosition(&currentPosition)
			time.Sleep(100 * time.Millisecond)
		}
	}()

	err := http.ListenAndServe(port, nil)
	fmt.Println("Serving static files from web/static/")

	if err != nil {
		log.Println("Error starting web server: ", err)
	} else {
		log.Println("Web server started on port ", port)
	}

}
