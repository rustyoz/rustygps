package types

import "time"

type Position struct {
	Lat     float64   `json:"Lat"`
	Lon     float64   `json:"Lon"`
	Heading float64   `json:"Heading"` // degrees north
	Speed   float64   `json:"Speed"`
	Time    time.Time `json:"Time"`
}

// TractorType represents the steering mechanism of the tractor
type TractorType int

const (
	Ackerman TractorType = iota
	Articulated
)

// WorldPosition represents position in meters from origin point
type WorldPosition struct {
	X       float64 `json:"X"`       // meters east from origin
	Y       float64 `json:"Y"`       // meters north from origin
	Heading float64 `json:"Heading"` // radians, 0 = east, clockwise positive
}

// CoordinateSystem defines the transformation between world and GPS coordinates
type CoordinateSystem struct {
	OriginLat float64 `json:"OriginLat"` // latitude of world origin (0,0)
	OriginLon float64 `json:"OriginLon"` // longitude of world origin (0,0)
	// Conversion factors at the origin point
	MetersPerLat float64 `json:"MetersPerLat"` // meters per degree latitude
	MetersPerLon float64 `json:"MetersPerLon"` // meters per degree longitude
}
