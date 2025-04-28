package planner

import "math"

// Vector represents a 2D vector
type Vector struct {
	X float64
	Y float64
}

// vecLength returns the length of a vector
func vecLength(v Vector) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

// vecUnit returns a unit vector in the same direction as the given vector
func vecUnit(v Vector) Vector {
	len := vecLength(v)
	if len == 0 {
		return Vector{X: 0, Y: 0}
	}
	return Vector{X: v.X / len, Y: v.Y / len}
}

// vecMul multiplies a vector by a scalar
func vecMul(v Vector, scalar float64) Vector {
	return Vector{X: v.X * scalar, Y: v.Y * scalar}
}

// vecDot returns the dot product of two vectors
func vecDot(v1, v2 Vector) float64 {
	return v1.X*v2.X + v1.Y*v2.Y
}

// vecRot90CW rotates a vector 90 degrees clockwise
func vecRot90CW(v Vector) Vector {
	return Vector{X: v.Y, Y: -v.X}
}

// vecRot90CCW rotates a vector 90 degrees counterclockwise
func vecRot90CCW(v Vector) Vector {
	return Vector{X: -v.Y, Y: v.X}
}

// intersect finds the intersection point of two line segments defined by four points
func intersect(line1 [2]Point, line2 [2]Point) Point {
	a1 := line1[1].X - line1[0].X
	b1 := line2[0].X - line2[1].X
	c1 := line2[0].X - line1[0].X

	a2 := line1[1].Y - line1[0].Y
	b2 := line2[0].Y - line2[1].Y
	c2 := line2[0].Y - line1[0].Y

	// Calculate the parameter t for the intersection point on line1
	// If the lines are parallel, this will divide by zero
	denominator := a2*b1 - a1*b2
	if denominator == 0 {
		// Lines are parallel, return midpoint of first line as fallback
		return Point{
			X: (line1[0].X + line1[1].X) / 2,
			Y: (line1[0].Y + line1[1].Y) / 2,
		}
	}

	t := (b1*c2 - b2*c1) / denominator

	// Calculate the intersection point using the parameter t
	return Point{
		X: line1[0].X + t*(line1[1].X-line1[0].X),
		Y: line1[0].Y + t*(line1[1].Y-line1[0].Y),
	}
}

// polyIsCw determines if a polygon is in clockwise order
func polyIsCw(poly []Point) bool {
	if len(poly) < 3 {
		return false
	}

	// Calculate sum of (x2 - x1)(y2 + y1) for all edges
	// Positive result means clockwise, negative means counterclockwise
	sum := 0.0
	for i := 0; i < len(poly); i++ {
		j := (i + 1) % len(poly)
		sum += (poly[j].X - poly[i].X) * (poly[j].Y + poly[i].Y)
	}

	return sum >= 0
}
