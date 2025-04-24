package tractor

import (
	"fmt"
	"log"
	"math"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/jacobsa/go-serial/serial"

	nmea "github.com/adrianmo/go-nmea"
	"github.com/rustyoz/rustygps/types"
)

// NewCoordinateSystem creates a new coordinate system with the given origin
func NewCoordinateSystem(originLat, originLon float64) *types.CoordinateSystem {
	// At the origin latitude, calculate meters per degree
	metersPerLat := 111132.92 - 559.82*math.Cos(2*originLat*math.Pi/180) +
		1.175*math.Cos(4*originLat*math.Pi/180)
	metersPerLon := 111412.84*math.Cos(originLat*math.Pi/180) -
		93.5*math.Cos(3*originLat*math.Pi/180)

	return &types.CoordinateSystem{
		OriginLat:    originLat,
		OriginLon:    originLon,
		MetersPerLat: metersPerLat,
		MetersPerLon: metersPerLon,
	}
}

// Tractor represents the physical properties and state of the tractor
type Tractor struct {
	WheelBase     float64                 // Length between front and rear axles in meters
	SteeringAngle float64                 // Current steering angle in radians
	SteeringType  types.TractorType       // Type of steering mechanism
	Speed         float64                 // Current speed in meters per second
	Position      types.Position          // Current GPS position
	WorldPos      types.WorldPosition     // Tractor Position in world coordinates (centre on rear axle)
	CoordSys      *types.CoordinateSystem // Coordinate system for transformations
	HitchOffset   float64                 // Distance from hitch point in meters
	HitchPos      types.WorldPosition     // Position of hitch point in world coordinates
	mu            sync.RWMutex
}

// NewTractor creates a new tractor with default values
func NewTractor() *Tractor {
	// Create coordinate system with initial position as origin
	coordSys := NewCoordinateSystem(-36.8013, 142.3142)

	return &Tractor{
		WheelBase:     2.5,
		SteeringAngle: 0.0,
		SteeringType:  types.Ackerman,
		Speed:         0.0,
		Position: types.Position{
			Lat:     coordSys.OriginLat,
			Lon:     coordSys.OriginLon,
			Heading: 90.0,
			Speed:   0.0,
			Time:    time.Now(),
		},
		WorldPos: types.WorldPosition{
			X:       0.0,
			Y:       0.0,
			Heading: 0.0,
		},

		HitchPos: types.WorldPosition{
			X:       0.0,
			Y:       0.0,
			Heading: 0.0,
		},
		HitchOffset: 0.5,
		CoordSys:    coordSys,
	}
}

// UpdatePosition calculates new position based on vehicle kinematics
func (t *Tractor) UpdatePosition(stepSize float64) {
	// Calculate movement in world coordinates
	if t.SteeringType == types.Ackerman {
		// Calculate turn radius based on wheelbase and steering angle
		turnRadius := t.WheelBase / math.Tan(t.SteeringAngle)

		// Calculate change in heading
		deltaHeading := (t.Speed * stepSize) / turnRadius
		if math.Abs(t.SteeringAngle) < 0.001 {
			deltaHeading = 0
		}

		// Update world heading
		t.WorldPos.Heading += deltaHeading
		// Normalize heading [-pi, pi]
		if t.WorldPos.Heading > math.Pi {
			t.WorldPos.Heading -= 2 * math.Pi
		}
		if t.WorldPos.Heading < -math.Pi {
			t.WorldPos.Heading += 2 * math.Pi
		}
	}

	// Calculate distance traveled in this step
	distance := t.Speed * stepSize

	// Update world position

	t.WorldPos.X += distance * math.Cos(t.WorldPos.Heading)
	t.WorldPos.Y += distance * math.Sin(t.WorldPos.Heading)

	// Update hitch position
	t.HitchPos.X = t.WorldPos.X - t.HitchOffset*math.Cos(t.WorldPos.Heading)
	t.HitchPos.Y = t.WorldPos.Y - t.HitchOffset*math.Sin(t.WorldPos.Heading)
	t.HitchPos.Heading = t.WorldPos.Heading

	// Convert world position to GPS coordinates
	latChange := t.WorldPos.Y / t.CoordSys.MetersPerLat
	lonChange := t.WorldPos.X / t.CoordSys.MetersPerLon

	// convert from world heading 0 radians = east to normal heading 0 degrees = north
	t.Position.Heading = -t.WorldPos.Heading*180.0/math.Pi + 90.0 + 360.0
	t.Position.Heading = math.Mod(t.Position.Heading, 360.0)

	// Update GPS position based on the coordinate system
	t.Position.Lat = t.CoordSys.OriginLat + latChange
	t.Position.Lon = t.CoordSys.OriginLon + lonChange
	t.Position.Speed = t.Speed * 3.6 // Convert m/s to km/h
	t.Position.Time = time.Now()
}

// GetPosition returns the current simulated position
func (t *Tractor) GetPosition() types.Position {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.Position
}

// UpdateTractorControls updates the speed and steering angle of the simulated tractor
func (t *Tractor) UpdateTractorControls(speed float64, steeringAngle float64) {
	t.mu.Lock()
	defer t.mu.Unlock()

	t.Speed = speed
	t.SteeringAngle = steeringAngle
	fmt.Println("Tractor controls updated: speed = ", t.Speed, " steering angle = ", t.SteeringAngle)
}

// UpdateConfiguration updates the physical configuration of the tractor
func (t *Tractor) UpdateConfiguration(wheelbase float64, hitchOffset float64) {
	t.mu.Lock()
	defer t.mu.Unlock()

	t.WheelBase = wheelbase
	t.HitchOffset = hitchOffset
	fmt.Printf("Tractor configuration updated: wheelbase = %.2f m, hitch offset = %.2f m\n", wheelbase, hitchOffset)
}

func Run(gpsPort string, gpsBaud string) {
	log.Println("GPS port: ", gpsPort)
	log.Println("GPS baud: ", gpsBaud)

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

// GetWorldPosition returns the current world position of the simulated tractor
func (t *Tractor) GetWorldPosition() types.WorldPosition {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.WorldPos
}

func (t *Tractor) GetWorldHitchPosition() types.WorldPosition {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.HitchPos
}
