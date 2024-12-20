package main

import (
	"flag"

	"github.com/rustyoz/rustygps/gps"
	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/web"
)

func main() {
	// Define flags with default values
	gpsPort := flag.String("gps-port", "/dev/ttyUSB0", "GPS serial port")
	gpsBaud := flag.String("gps-baud", "9600", "GPS baud rate")
	implementPort := flag.String("impl-port", "/dev/ttyUSB1", "Implement serial port")
	implementBaud := flag.String("impl-baud", "9600", "Implement baud rate")

	// Parse flags
	flag.Parse()

	// start the gps process
	go gps.Run(*gpsPort, *gpsBaud)

	// start the implement process
	go implement.Run(*implementPort, *implementBaud)

	// start the web process
	web.Run(":8080")
}
