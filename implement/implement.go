package implement

import (
	"log"
	"strconv"

	"github.com/jacobsa/go-serial/serial"
)

func Run(implementPort string, implementBaud string) {
	log.Println("Implement port: ", implementPort)
	log.Println("Implement baud: ", implementBaud)

	// Convert baud string to uint64
	baud, err := strconv.ParseUint(implementBaud, 10, 32)
	if err != nil {
		log.Println("Error parsing baud rate:", err)
		return
	}

	// open the gps port
	port, err := serial.Open(serial.OpenOptions{
		PortName:        implementPort,
		BaudRate:        uint(baud),
		DataBits:        8,
		StopBits:        1,
		ParityMode:      serial.PARITY_NONE,
		MinimumReadSize: 1,
	})
	if err != nil {
		log.Println("Error opening Implement port: ", err)
		return
	}

	// read the gps data
	buffer := make([]byte, 1024)
	for {
		n, err := port.Read(buffer)
		if err != nil {
			log.Println("Error reading Implement data: ", err)
			return
		}

		// print the implement data
		log.Println(string(buffer[:n]))

	}

}
