package web

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/rustyoz/rustygps/guidance"
	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/planner"
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
var thePlanner *planner.ABLinePlanner
var theField planner.Field
var thePath []planner.Point
var theGuidance *guidance.Guidance

func init() {
	// Create a rectangular field 400m x 800m
	boundary := []planner.Point{
		{X: 0, Y: 0},
		{X: 100, Y: 0},
		{X: 100, Y: 30},
		{X: 0, Y: 30},
	}

	// Create default AB line from bottom to top of field
	abLine := planner.FindABLine(boundary)

	theField = planner.NewField(boundary, *abLine)
	thePlanner = planner.NewABLinePlanner()
	theGuidance = guidance.NewGuidance(&thePath)
}

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
		_, p, err := conn.ReadMessage()
		if err != nil {
			return
		}

		var message map[string]interface{}
		if err := json.Unmarshal(p, &message); err != nil {
			continue
		}

		switch message["type"] {
		case "getField":
			//fmt.Println("getField")
			response := map[string]interface{}{
				"type":     "field",
				"boundary": theField.Boundary,
				"abLine":   theField.ABLine,
			}
			data, err := json.Marshal(response)
			if err != nil {
				continue
			}
			client.mu.Lock()
			err = client.conn.WriteMessage(websocket.TextMessage, data)
			client.mu.Unlock()
			if err != nil {
				return
			}

		case "calculatePath":
			//fmt.Println("calculatePath")
			config := planner.PlannerConfig{
				ImplementWidth:      6.0, // Default 6m implement
				TractorTurnRadius:   8.0,
				ImplementTurnRadius: 10.0,
			}

			// Update config from message if provided
			if cfg, ok := message["config"].(map[string]interface{}); ok {
				if width, ok := cfg["implementWidth"].(float64); ok {
					config.ImplementWidth = width
				}
			}

			path, err := thePlanner.GeneratePath(theField, config)
			thePath = path
			if err != nil {
				fmt.Println("error generating path", err)
				continue
			}

			response := map[string]interface{}{
				"type":   "path",
				"points": path,
			}
			data, err := json.Marshal(response)
			if err != nil {
				continue
			}
			client.mu.Lock()
			err = client.conn.WriteMessage(websocket.TextMessage, data)
			client.mu.Unlock()
			if err != nil {
				return
			}

		case "config":
			// Update tractor configuration
			theTractor.UpdateConfiguration(message["tractor"].(map[string]interface{})["wheelbase"].(float64), message["tractor"].(map[string]interface{})["hitchOffset"].(float64))
			// Update implement configuration
			theImplement.UpdateConfiguration(message["implement"].(map[string]interface{})["length"].(float64), message["implement"].(map[string]interface{})["width"].(float64))

		case "control":
			// Convert speed from km/h to m/s
			speedMS := message["speed"].(float64) / 3.6
			// Convert steering angle to radians with positive being anticlockwise
			steeringRad := message["steeringAngle"].(float64) * (-3.14159 / 180.0)

			// Update the tractor simulation
			theTractor.UpdateTractorControls(speedMS, steeringRad)

		case "guidanceParams":
			theGuidance.UpdateParameters(message["lookAheadDistance"].(float64), message["deadzone"].(float64), message["planningDistance"].(float64))

		case "reset":
			// Reset tractor position and controls
			theTractor.Reset()
			theImplement.Reset()

			// Recalculate path
			path, err := thePlanner.GeneratePath(theField, planner.PlannerConfig{
				ImplementWidth:      theImplement.WorkingWidth,
				TractorTurnRadius:   8.0,
				ImplementTurnRadius: 10.0,
			})
			if err != nil {
				fmt.Println("error generating path during reset:", err)
				continue
			}
			thePath = path

			if theGuidance.Path != &thePath {
				panic("path not updated")
			}

			// Send updated path to client
			response := map[string]interface{}{
				"type":   "path",
				"points": path,
			}
			data, err := json.Marshal(response)
			if err != nil {
				continue
			}
			client.mu.Lock()
			err = client.conn.WriteMessage(websocket.TextMessage, data)
			client.mu.Unlock()
			if err != nil {
				return
			}
		}
	}
}

func BroadcastPosition() {
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

	data, err := json.Marshal(message)
	if err != nil {
		return
	}

	clientsMu.Lock()
	for client := range clients {
		client.mu.Lock()
		if err := client.conn.WriteMessage(websocket.TextMessage, data); err != nil {
			client.mu.Unlock()
			delete(clients, client)
			continue
		}
		client.mu.Unlock()
	}
	clientsMu.Unlock()
}

func BroadcastGuidance() {
	theGuidance.Update()
	message := struct {
		Type          string        `json:"type"`
		TargetPoint   planner.Point `json:"targetPoint"`
		SteeringAngle float64       `json:"steeringAngle"`
	}{
		Type:          "guidance",
		TargetPoint:   theGuidance.TargetPoint,
		SteeringAngle: theGuidance.TargetSteeringAngle,
	}

	data, err := json.Marshal(message)
	if err != nil {
		return
	}

	clientsMu.Lock()
	for client := range clients {
		client.mu.Lock()
		if err := client.conn.WriteMessage(websocket.TextMessage, data); err != nil {
			client.mu.Unlock()
			delete(clients, client)
			continue
		}
		client.mu.Unlock()
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

	theGuidance.Path = &thePath

	// serve guidance updates to websocket at 1hz
	go func() {
		for {
			theGuidance.SetTractorState(guidance.TractorState{
				Position:      theTractor.GetPosition(),
				WorldPos:      theTractor.GetWorldPosition(),
				SteeringAngle: theTractor.SteeringAngle,
				Throttle:      theTractor.Throttle,
			})
			theGuidance.Update()
			BroadcastGuidance()
			time.Sleep(1000 / 5 * time.Millisecond)
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
