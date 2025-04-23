package main

import (
	"flag"

	"github.com/rustyoz/rustygps/gps"
	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/web"
)

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
		go gps.Run(*gpsPort, *gpsBaud)
		// start the implement process with real serial connection
		go implement.Run(*implementPort, *implementBaud)
	} else {
		// start the gps process in simulation mode
		go gps.RunSimulation()
		// start the implement process in simulation mode
		go implement.RunSimulation()
	}

	// start the web process
	port := ":8080"

	web.Run(port)

}
