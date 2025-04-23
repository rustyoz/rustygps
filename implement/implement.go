package implement

import (
	"math"
	"sync"
	"time"

	"github.com/rustyoz/rustygps/gps"
)

type Position struct {
	Lat     float64
	Lon     float64
	Heading float64
	Time    time.Time
}

type Implement struct {
	Position  Position
	Length    float64  // Distance from hitch point in meters
	LastHitch Position // Previous hitch point for heading calculation
}

var simulatedImplement *Implement
var currentPosition Position
var mu sync.RWMutex

// NewImplement creates a new implement with default values
func NewImplement() *Implement {
	return &Implement{
		Length: 3.0, // 3 meters behind hitch point
		Position: Position{
			Lat:     -36.8013,
			Lon:     142.3142,
			Heading: 0.0,
			Time:    time.Now(),
		},
	}
}

// GetPosition returns the current implement position
func GetPosition() Position {
	mu.RLock()
	defer mu.RUnlock()
	return currentPosition
}

// updatePosition calculates new implement position based on tractor position
func (impl *Implement) updatePosition(tractorPos gps.Position) {
	// Store current tractor position as hitch point
	hitchPos := Position{
		Lat:     tractorPos.Lat,
		Lon:     tractorPos.Lon,
		Heading: tractorPos.Heading,
		Time:    tractorPos.Time,
	}

	// Convert tractor heading to radians
	headingRad := tractorPos.Heading * math.Pi / 180.0

	// Calculate implement position behind tractor
	// Using the implement length and tractor heading
	// Convert implement length to degrees (approximately)
	latChange := -(impl.Length * math.Cos(headingRad)) / 111000.0
	lonChange := -(impl.Length * math.Sin(headingRad)) / (111000.0 * math.Cos(tractorPos.Lat*math.Pi/180.0))

	// Update implement position
	impl.Position.Lat = tractorPos.Lat + latChange
	impl.Position.Lon = tractorPos.Lon + lonChange
	impl.Position.Time = time.Now()

	// Calculate implement heading based on line from previous to current position
	if impl.LastHitch.Time.Before(hitchPos.Time) {
		// Calculate heading from implement position to hitch point
		deltaLon := hitchPos.Lon - impl.Position.Lon
		deltaLat := hitchPos.Lat - impl.Position.Lat

		// Calculate heading in degrees
		heading := math.Atan2(deltaLon, deltaLat) * 180.0 / math.Pi
		// Normalize to 0-360 degrees
		if heading < 0 {
			heading += 360
		}
		impl.Position.Heading = heading

		// Store current hitch position for next update
		impl.LastHitch = hitchPos
	}
}

// RunSimulation starts the implement simulation
func RunSimulation() {
	simulatedImplement = NewImplement()

	// Update at 10Hz
	ticker := time.NewTicker(100 * time.Millisecond)

	go func() {
		for range ticker.C {
			// Get current tractor position
			tractorPos := gps.GetPosition()

			// Update implement position
			simulatedImplement.updatePosition(tractorPos)

			// Update the current position
			mu.Lock()
			currentPosition = simulatedImplement.Position
			mu.Unlock()
		}
	}()
}

func Run(port string, baud string) {
	RunSimulation()
}
