package implement

import (
	"fmt"
	"log"
	"math"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/adrianmo/go-nmea"
	"github.com/jacobsa/go-serial/serial"
	"github.com/rustyoz/rustygps/tractor"
	"github.com/rustyoz/rustygps/types"
)

type Implement struct {
	Position     types.Position          // GPS position
	WorldPos     types.WorldPosition     // Position in world coordinates
	Length       float64                 // Distance from hitch point to implement axle in meters
	WheelBase    float64                 // Distance between axles (for bicycle model)
	LastHitchPos types.WorldPosition     // Previous hitch point in world coordinates
	LastTime     time.Time               // Time of last update for delta time calculation
	CoordSys     *types.CoordinateSystem // Reference to coordinate system
	WorkingWidth float64                 // Width of the implement in meters
	mu           sync.RWMutex
}

// NewImplement creates a new implement with default values
func NewImplement() *Implement {
	// Use same coordinate system as tractor
	coordSys := tractor.NewCoordinateSystem(-36.8013, 142.3142)

	return &Implement{
		Length:    3.0, // 3 meters from hitch point to implement axle
		WheelBase: 2.0, // 2 meters between virtual axles for bicycle model
		Position: types.Position{
			Lat:     coordSys.OriginLat,
			Lon:     coordSys.OriginLon,
			Heading: 0.0,
			Time:    time.Now(),
		},
		WorldPos: types.WorldPosition{
			X:       0.0,
			Y:       0.0,
			Heading: 0,
		},
		LastTime: time.Now(),
		CoordSys: coordSys,
	}
}

// GetPosition returns the current implement position
func (impl *Implement) GetPosition() types.Position {
	impl.mu.RLock()
	defer impl.mu.RUnlock()
	return impl.Position
}

func (impl *Implement) GetWorldPosition() types.WorldPosition {
	impl.mu.RLock()
	defer impl.mu.RUnlock()
	return impl.WorldPos
}

// UpdateConfiguration updates the physical configuration of the implement
func (impl *Implement) UpdateConfiguration(length float64, width float64) {
	impl.mu.Lock()
	defer impl.mu.Unlock()

	impl.Length = length
	impl.WorkingWidth = width // Store width in WorkingWidth field
	fmt.Printf("Implement configuration updated: length = %.2f m, width = %.2f m\n", length, width)
}

// updatePosition calculates new implement position based on tractor position and world coordinates
// using a bicycle model for more accurate trailer dynamics
func (impl *Implement) UpdatePosition(tractorHitchPos types.WorldPosition) {
	// Calculate time delta since last update
	now := time.Now()
	dt := now.Sub(impl.LastTime).Seconds()
	impl.LastTime = now

	// Calculate tractor movement since last update
	hitchDeltaX := tractorHitchPos.X - impl.LastHitchPos.X
	hitchDeltaY := tractorHitchPos.Y - impl.LastHitchPos.Y

	// Calculate tractor velocity
	tractorVelocity := math.Sqrt(hitchDeltaX*hitchDeltaX+hitchDeltaY*hitchDeltaY) / dt

	if tractorVelocity == 0 {
		// If nearly stationary, maintain current position
		impl.LastHitchPos = tractorHitchPos
		return
	}

	// Calculate the angle between the tractor and implement (articulation angle)
	articulationAngle := tractorHitchPos.Heading - impl.WorldPos.Heading

	// Normalize articulation angle to [-pi, pi]
	for articulationAngle > math.Pi {
		articulationAngle -= 2 * math.Pi
	}
	for articulationAngle < -math.Pi {
		articulationAngle += 2 * math.Pi
	}

	// calculate the turn radis of the implement
	turnRadius := impl.Length / math.Tan(articulationAngle)

	// calculate the implement's angular velocity
	angularVelocity := tractorVelocity * math.Cos(articulationAngle) / turnRadius

	// filter to

	// update the implement's heading
	impl.WorldPos.Heading += angularVelocity * dt

	// Normalize heading to [-pi, pi]
	for impl.WorldPos.Heading > math.Pi {
		impl.WorldPos.Heading -= 2 * math.Pi
	}
	for impl.WorldPos.Heading < -math.Pi {
		impl.WorldPos.Heading += 2 * math.Pi
	}

	// Update implement position based on new heading and hitch point
	impl.WorldPos.X = tractorHitchPos.X - impl.Length*math.Cos(impl.WorldPos.Heading)
	impl.WorldPos.Y = tractorHitchPos.Y - impl.Length*math.Sin(impl.WorldPos.Heading)

	// Store current hitch position for next update
	impl.LastHitchPos = tractorHitchPos

	// Convert world position to GPS coordinates
	latChange := impl.WorldPos.Y / impl.CoordSys.MetersPerLat
	lonChange := impl.WorldPos.X / impl.CoordSys.MetersPerLon

	// Update GPS position
	impl.Position.Lat = impl.CoordSys.OriginLat + latChange
	impl.Position.Lon = impl.CoordSys.OriginLon + lonChange
	impl.Position.Heading = impl.WorldPos.Heading * 180.0 / math.Pi // Convert to degrees
	impl.Position.Time = time.Now()
}

func Run(gpsPort string, gpsBaud string) {
	log.Println("implement GPS port: ", gpsPort)
	log.Println("implement GPS baud: ", gpsBaud)

	// Convert baud string to uint64
	baud, err := strconv.ParseUint(gpsBaud, 10, 32)
	if err != nil {
		log.Println("Error parsing baud rate:", err)
		return
	}

	// open the gps port
	port, err := serial.Open(serial.OpenOptions{
		PortName:        gpsPort,
		BaudRate:        uint(baud),
		DataBits:        8,
		StopBits:        1,
		ParityMode:      serial.PARITY_NONE,
		MinimumReadSize: 1,
	})
	if err != nil {
		log.Println("Error opening GPS port: ", err)
		return
	}

	// read the gps data
	buffer := make([]byte, 1024)
	for {
		n, err := port.Read(buffer)
		if err != nil {
			log.Println("Error reading GPS data: ", err)
			return
		}

		// print the gps data
		log.Println(string(buffer[:n]))

		// read the buffer for NMEA sentences
		sentence := strings.Split(string(buffer[:n]), "$")
		if len(sentence) > 1 {
			log.Println("NMEA sentence: ", sentence[1])
		}

		// parse the NMEA sentence
		msg, err := nmea.Parse(sentence[1])
		if err != nil {
			log.Println("Error parsing NMEA sentence: ", err)
			return
		}

		// print the parsed message
		log.Println("Parsed message: ", msg)

	}

}

// Reset resets the implement to its initial state
func (i *Implement) Reset() {
	i.Position = types.Position{
		Lat:     0,
		Lon:     0,
		Heading: 0,
		Speed:   0,
		Time:    time.Now(),
	}
	i.WorldPos = types.WorldPosition{
		X:       0,
		Y:       0,
		Heading: 0,
	}
}
