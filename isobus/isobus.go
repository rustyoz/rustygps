package isobus

import (
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/jacobsa/go-serial/serial"
)

// ISOBUS constants
const (
	DefaultBaudRate = 250000 // Standard ISOBUS baud rate is 250 kbps
	DefaultDataBits = 8
	DefaultStopBits = 1
	DefaultParity   = serial.PARITY_NONE
)

// Message represents an ISOBUS message
type Message struct {
	PGN       uint32    // Parameter Group Number
	Priority  uint8     // Message priority (0-7)
	Source    uint8     // Source address
	Dest      uint8     // Destination address
	Data      []byte    // Message data
	Timestamp time.Time // Time when message was received
}

// Device represents an ISOBUS device
type Device struct {
	Address      uint8             // Device address on the bus
	Name         string            // Device name
	Manufacturer string            // Manufacturer name
	Functions    map[string]string // Device functions
}

// ISOBUS represents the ISOBUS interface
type ISOBUS struct {
	port         serial.SerialPort
	portName     string
	baudRate     uint
	isConnected  bool
	devices      map[uint8]*Device // Map of connected devices by address
	messageQueue chan Message      // Channel for message processing
	mu           sync.RWMutex
}

// NewISOBUS creates a new ISOBUS interface
func NewISOBUS(portName string) *ISOBUS {
	return &ISOBUS{
		portName:     portName,
		baudRate:     DefaultBaudRate,
		isConnected:  false,
		devices:      make(map[uint8]*Device),
		messageQueue: make(chan Message, 100),
	}
}

// Connect establishes a connection to the ISOBUS
func (i *ISOBUS) Connect() error {
	options := serial.OpenOptions{
		PortName:        i.portName,
		BaudRate:        i.baudRate,
		DataBits:        DefaultDataBits,
		StopBits:        DefaultStopBits,
		ParityMode:      DefaultParity,
		MinimumReadSize: 1,
	}

	port, err := serial.Open(options)
	if err != nil {
		return fmt.Errorf("failed to open ISOBUS port: %v", err)
	}

	i.mu.Lock()
	i.port = port
	i.isConnected = true
	i.mu.Unlock()

	// Start message processing goroutine
	go i.processMessages()

	// Start device discovery
	go i.discoverDevices()

	log.Printf("Connected to ISOBUS on port %s at %d baud", i.portName, i.baudRate)
	return nil
}

// Disconnect closes the ISOBUS connection
func (i *ISOBUS) Disconnect() error {
	i.mu.Lock()
	defer i.mu.Unlock()

	if !i.isConnected {
		return nil
	}

	err := i.port.Close()
	if err != nil {
		return fmt.Errorf("failed to close ISOBUS port: %v", err)
	}

	i.isConnected = false
	log.Printf("Disconnected from ISOBUS on port %s", i.portName)
	return nil
}

// SendMessage sends a message to the ISOBUS
func (i *ISOBUS) SendMessage(msg Message) error {
	i.mu.RLock()
	defer i.mu.RUnlock()

	if !i.isConnected {
		return fmt.Errorf("not connected to ISOBUS")
	}

	// Encode the message according to ISOBUS protocol
	encodedMsg := encodeMessage(msg)

	_, err := i.port.Write(encodedMsg)
	if err != nil {
		return fmt.Errorf("failed to send message: %v", err)
	}

	return nil
}

// encodeMessage encodes an ISOBUS message into bytes
func encodeMessage(msg Message) []byte {
	// This is a simplified implementation
	// Real implementation would follow ISO 11783 standard

	// Basic format: [Priority + PGN (3 bytes)][Source][Destination][Data Length][Data][Checksum]
	result := make([]byte, 5+len(msg.Data)+1)

	// Priority and PGN
	pgn := msg.PGN & 0x3FFFF // 18 bits for PGN
	result[0] = byte((msg.Priority << 5) | ((pgn >> 16) & 0x7))
	result[1] = byte((pgn >> 8) & 0xFF)
	result[2] = byte(pgn & 0xFF)

	// Source and destination
	result[3] = msg.Source
	result[4] = msg.Dest

	// Data length
	result[5] = byte(len(msg.Data))

	// Data
	copy(result[6:], msg.Data)

	// Simple checksum (sum of all bytes)
	checksum := byte(0)
	for i := 0; i < len(result)-1; i++ {
		checksum += result[i]
	}
	result[len(result)-1] = checksum

	return result
}

// processMessages reads and processes incoming ISOBUS messages
func (i *ISOBUS) processMessages() {
	buffer := make([]byte, 1024)

	for {
		i.mu.RLock()
		if !i.isConnected {
			i.mu.RUnlock()
			return
		}
		port := i.port
		i.mu.RUnlock()

		n, err := port.Read(buffer)
		if err != nil {
			log.Printf("Error reading from ISOBUS: %v", err)
			continue
		}

		if n > 0 {
			// Process received data
			messages := parseMessages(buffer[:n])
			for _, msg := range messages {
				i.messageQueue <- msg
			}
		}
	}
}

// parseMessages parses raw bytes into ISOBUS messages
func parseMessages(data []byte) []Message {
	// This is a simplified implementation
	// Real implementation would follow ISO 11783 standard

	var messages []Message

	// Simple parsing logic - in reality this would be more complex
	// and handle message framing, checksums, etc.
	if len(data) < 7 { // Minimum message size
		return messages
	}

	// Extract a single message for demonstration
	priority := (data[0] >> 5) & 0x7
	pgn := uint32((uint32(data[0]&0x7) << 16) | (uint32(data[1]) << 8) | uint32(data[2]))
	source := data[3]
	dest := data[4]
	dataLen := int(data[5])

	if 6+dataLen+1 <= len(data) {
		msgData := make([]byte, dataLen)
		copy(msgData, data[6:6+dataLen])

		messages = append(messages, Message{
			PGN:       pgn,
			Priority:  priority,
			Source:    source,
			Dest:      dest,
			Data:      msgData,
			Timestamp: time.Now(),
		})
	}

	return messages
}

// discoverDevices sends address claim messages and builds a map of connected devices
func (i *ISOBUS) discoverDevices() {
	// In a real implementation, this would:
	// 1. Send an address claim request
	// 2. Listen for responses
	// 3. Build a map of connected devices

	// For demonstration, we'll just log that we're looking for devices
	log.Println("Scanning for ISOBUS devices...")

	// In a real implementation, we would send a request and process responses
	// For now, we'll just simulate finding a device
	i.mu.Lock()
	i.devices[0x80] = &Device{
		Address:      0x80,
		Name:         "Virtual ISOBUS Device",
		Manufacturer: "Example Manufacturer",
		Functions:    map[string]string{"function1": "Steering Control"},
	}
	i.mu.Unlock()

	log.Println("Found 1 ISOBUS device")
}

// GetDevices returns a list of discovered ISOBUS devices
func (i *ISOBUS) GetDevices() []*Device {
	i.mu.RLock()
	defer i.mu.RUnlock()

	devices := make([]*Device, 0, len(i.devices))
	for _, device := range i.devices {
		devices = append(devices, device)
	}

	return devices
}

// SetBaudRate changes the baud rate for the ISOBUS connection
func (i *ISOBUS) SetBaudRate(baudRate uint) {
	i.mu.Lock()
	defer i.mu.Unlock()

	i.baudRate = baudRate
	log.Printf("ISOBUS baud rate set to %d", baudRate)
}

// IsConnected returns the connection status
func (i *ISOBUS) IsConnected() bool {
	i.mu.RLock()
	defer i.mu.RUnlock()

	return i.isConnected
}
