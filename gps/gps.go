package gps

import (
	"log"
	"strconv"
	"strings"
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

func GetPosition() Position {
	return Position{
		Lat:     0.0,
		Lon:     0.0,
		Heading: 0.0,
		Speed:   0.0,
		Time:    time.Now(),
	}
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
