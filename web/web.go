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

var theTractor *tractor.Tractor
var theImplement *implement.Implement
var thePlanner *planner.ABLinePlanner
var theField planner.Field
var thePath []planner.Point
var theGuidance *guidance.Guidance

func init() {
	// Create a rectangular field 400m x 800m
	boundary := []planner.Point{
		{X: -10, Y: -10},
		{X: -10, Y: 200},
		{X: 200, Y: 200},
		{X: 200, Y: -10},
	}

	//boundary = planner.RotateBoundary(boundary, math.Pi/2)

	theField = planner.NewField(boundary)
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
			// Calculate inner boundary with 10m offset (or whatever offset you prefer)
			//path, err := thePlanner.GeneratePath(&theField, theTractor, theImplement)
			response := map[string]interface{}{
				"type":          "field",
				"boundary":      theField.Boundary,
				"abLine":        theField.ABLine,
				"innerBoundary": theField.WorkingArea,
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

			// Update config from message if provided
			if cfg, ok := message["config"].(map[string]interface{}); ok {
				if width, ok := cfg["implementWidth"].(float64); ok {
					theImplement.WorkingWidth = width
				}
			}

			path, controlPath, err := thePlanner.GeneratePath(&theField, theTractor, theImplement)
			thePath = path
			if err != nil {
				fmt.Println("error generating path", err)
				continue
			}

			response := map[string]interface{}{
				"type":          "path",
				"points":        path,
				"controlPoints": controlPath,
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
			fmt.Println("resetting tractor")
			theTractor.Reset()
			fmt.Println("resetting implement")
			theImplement.Reset()

			// Recalculate pathreset
			path, controlPath, err := thePlanner.GeneratePath(&theField, theTractor, theImplement)
			if err != nil {
				fmt.Println("error generating path during reset:", err)
				continue
			}
			thePath = path
			theGuidance.Reset()

			if theGuidance.Path != &thePath {
				panic("path not updated")
			}

			// Send updated path to client
			response := map[string]interface{}{
				"type":          "path",
				"points":        path,
				"controlPoints": controlPath,
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
		Type                  string                `json:"type"`
		TractorPos            types.Position        `json:"tractorPos"`
		TractorWorldPos       types.WorldPosition   `json:"tractorWorldPos"`
		ImplementPos          types.Position        `json:"implementPos"`
		ImplementWorldPos     types.WorldPosition   `json:"implementWorldPos"`
		ImplementCoverageLine []types.WorldPosition `json:"implementCoverageLine"`
	}{
		Type:                  "position",
		TractorPos:            tractorPos,
		TractorWorldPos:       tractorWorldPos,
		ImplementPos:          implementPos,
		ImplementWorldPos:     implementWorldPos,
		ImplementCoverageLine: theImplement.CoverageLine,
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
	// serve the web interface
	http.HandleFunc("/ws", handleWebSocket)

	http.Handle("/", http.StripPrefix("/", http.FileServer(http.Dir("web/static/"))))

	// serve position updates to websocket at 10hz
	go func() {
		for {

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
