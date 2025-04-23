package gps

import (
	"log"
	"math"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/jacobsa/go-serial/serial"

	nmea "github.com/adrianmo/go-nmea"
)

type Position struct {
	Lat     float64
	Lon     float64
	Heading float64
	Speed   float64
	Time    time.Time
}

// TractorType represents the steering mechanism of the tractor
type TractorType int

const (
	Ackerman TractorType = iota
	Articulated
)

// Tractor represents the physical properties and state of the tractor
type Tractor struct {
	WheelBase     float64     // Length between front and rear axles in meters
	SteeringAngle float64     // Current steering angle in radians
	SteeringType  TractorType // Type of steering mechanism
	Speed         float64     // Current speed in meters per second
	Position      Position    // Current GPS position
}

// NewTractor creates a new tractor with default values
func NewTractor() *Tractor {
	return &Tractor{
		WheelBase:     2.5, // 2.5 meters wheelbase
		SteeringAngle: 0.0, // 0 radians (straight)
		SteeringType:  Ackerman,
		Speed:         0.0, // 0 m/s initial speed
		Position: Position{
			Lat:     -36.8013, // Initial position (same as in web interface)
			Lon:     142.3142,
			Heading: 0.0,
			Speed:   0.0,
			Time:    time.Now(),
		},
	}
}

var simulatedTractor *Tractor
var currentPosition Position
var mu sync.RWMutex

// updatePosition calculates new position based on vehicle kinematics
func (t *Tractor) updatePosition(stepSize float64) {
	// Convert heading to radians for calculations
	headingRad := t.Position.Heading * math.Pi / 180.0

	// For Ackerman steering, use bicycle model
	// Change in heading based on steering geometry
	if t.SteeringType == Ackerman {
		// Calculate turn radius based on wheelbase and steering angle
		turnRadius := t.WheelBase / math.Tan(t.SteeringAngle)

		// Calculate change in heading
		deltaHeading := (t.Speed * stepSize) / turnRadius
		if math.Abs(t.SteeringAngle) < 0.001 {
			deltaHeading = 0
		}

		// Update heading
		headingRad += deltaHeading
		// normalize the heading to be between 0 and 360
		headingRad = math.Mod(headingRad, 2*math.Pi)
	}

	// Calculate distance traveled in this step
	distance := t.Speed * stepSize

	// Calculate position changes in meters
	dx := distance * math.Cos(headingRad)
	dy := distance * math.Sin(headingRad)

	// Convert to approximate lat/lon changes
	// At these latitudes, 1 degree is approximately 111km
	latChange := dy / 111000.0
	lonChange := dx / (111000.0 * math.Cos(t.Position.Lat*math.Pi/180.0))

	// Update position
	t.Position.Lat += latChange
	t.Position.Lon += lonChange
	t.Position.Heading = headingRad * 180.0 / math.Pi // Convert back to degrees
	t.Position.Speed = t.Speed * 3.6                  // Convert m/s to km/h
	t.Position.Time = time.Now()
}

// GetPosition returns the current simulated position
func GetPosition() Position {
	mu.RLock()
	defer mu.RUnlock()
	return currentPosition
}

// UpdateTractorControls updates the speed and steering angle of the simulated tractor
func UpdateTractorControls(speed float64, steeringAngle float64) {
	mu.Lock()
	defer mu.Unlock()

	if simulatedTractor != nil {
		simulatedTractor.Speed = speed
		simulatedTractor.SteeringAngle = steeringAngle
	}
}

// RunSimulation starts the GPS simulation
func RunSimulation() {
	simulatedTractor = NewTractor()

	// Update at 10Hz
	ticker := time.NewTicker(100 * time.Millisecond)

	go func() {
		for range ticker.C {
			// Update position with 0.1 second step size
			simulatedTractor.updatePosition(0.1)

			// Update the current position
			mu.Lock()
			currentPosition = simulatedTractor.Position
			mu.Unlock()
		}
	}()
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

// implement a RunSimulation fuction. This will produce a gps position by maintaining a simulation of a tractor. The tractor properties will be defined in a struct, including wheel base length, steering angle, steering type (akerman, articulated), speed. Create a simulation engine that can take a step size (in seconds) to update the tractors position based on speed and steering angle.
