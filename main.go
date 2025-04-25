package main

import (
	"flag"
	"time"

	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/tractor"
	"github.com/rustyoz/rustygps/web"
)

var theTractor *tractor.Tractor
var theImplement *implement.Implement

func main() {
	// Define flags with default values
	simulationMode := flag.Bool("simulate", true, "Enable simulation mode (no serial connections)")
	gpsPort := flag.String("gps-port", "/dev/ttyUSB0", "GPS serial port")
	gpsBaud := flag.String("gps-baud", "9600", "GPS baud rate")
	implementPort := flag.String("impl-port", "/dev/ttyUSB1", "Implement serial port")
	implementBaud := flag.String("impl-baud", "9600", "Implement baud rate")

	// Parse flags
	flag.Parse()

	if !*simulationMode {
		// start the gps process with real serial connection
		go tractor.Run(*gpsPort, *gpsBaud)
		// start the implement process with real serial connection
		go implement.Run(*implementPort, *implementBaud)
	} else {
		theTractor = tractor.NewTractor()
		theImplement = implement.NewImplement()

		web.SetTractor(theTractor)
		web.SetImplement(theImplement)

		RunSimulation()
	}

	// start the web process
	port := ":8080"

	web.Run(port)

}

// RunSimulation starts the GPS simulation
func RunSimulation() {

	// Update at 50Hz
	rate := 25.0 // Hz
	millis := 1000 / rate

	ticker := time.NewTicker(time.Duration(millis) * time.Millisecond)

	go func() {
		for range ticker.C {
			// Update position with 0.1 second step size
			theTractor.UpdatePosition(1 / rate)
			theImplement.UpdatePosition(theTractor.GetWorldHitchPosition())
		}
	}()
}
