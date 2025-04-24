package web

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/tractor"
	"github.com/rustyoz/rustygps/types"
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

var currentTractorPosition types.Position
var currentImplementPosition types.Position

var theTractor *tractor.Tractor
var theImplement *implement.Implement

func SetTractor(tractor *tractor.Tractor) {
	theTractor = tractor
}

func SetImplement(implement *implement.Implement) {
	theImplement = implement
}

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

		// Handle messages from client
		var msg struct {
			Type    string `json:"type"`
			Tractor struct {
				Wheelbase   float64 `json:"wheelbase"`
				HitchOffset float64 `json:"hitchOffset"`
			} `json:"tractor"`
			Implement struct {
				Length float64 `json:"length"`
				Width  float64 `json:"width"`
			} `json:"implement"`
			Speed         float64 `json:"speed"`
			SteeringAngle float64 `json:"steeringAngle"`
		}

		if err := json.Unmarshal(message, &msg); err != nil {
			continue
		}

		switch msg.Type {
		case "config":
			// Update tractor configuration
			theTractor.UpdateConfiguration(msg.Tractor.Wheelbase, msg.Tractor.HitchOffset)
			// Update implement configuration
			theImplement.UpdateConfiguration(msg.Implement.Length, msg.Implement.Width)

		case "control":
			// Convert speed from km/h to m/s
			speedMS := msg.Speed / 3.6
			// Convert steering angle to radians with positive being anticlockwise
			steeringRad := msg.SteeringAngle * (-3.14159 / 180.0)

			// Update the tractor simulation
			theTractor.UpdateTractorControls(speedMS, steeringRad)
		}
	}
}

// BroadcastPosition sends position updates to all connected clients
func BroadcastPosition() {

	// Get tractor position
	tractorPos := theTractor.GetPosition()
	tractorWorldPos := theTractor.GetWorldPosition()
	implementPos := theImplement.GetPosition()
	implementWorldPos := theImplement.GetWorldPosition()
	message := struct {
		Type              string              `json:"type"`
		TractorPos        types.Position      `json:"tractorPos"`
		TractorWorldPos   types.WorldPosition `json:"tractorWorldPos"`
		ImplementPos      types.Position      `json:"implementPos"`
		ImplementWorldPos types.WorldPosition `json:"implementWorldPos"`
	}{
		Type:              "position",
		TractorPos:        tractorPos,
		TractorWorldPos:   tractorWorldPos,
		ImplementPos:      implementPos,
		ImplementWorldPos: implementWorldPos,
	}

	//fmt.Printf("tractor heading: %f implement heading: %f articulation angle: %f\n", tractorWorldPos.Heading, implementWorldPos.Heading, implementWorldPos.Heading-tractorWorldPos.Heading)

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

	// serve the static files
	http.Handle("/web/static/", http.StripPrefix("/web/static/", http.FileServer(http.Dir("web/static/"))))

	// serve the web interface
	http.HandleFunc("/ws", handleWebSocket)

	// serve position updates to websocket at 10hz
	go func() {
		for {
			currentTractorPosition = theTractor.GetPosition()
			BroadcastPosition()
			time.Sleep(1000 / 50 * time.Millisecond)
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
