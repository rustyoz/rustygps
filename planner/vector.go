package planner

import (
	"fmt"
	"math"
)

// Vector represents a 2D vector
type Vector struct {
	X float64
	Y float64
}

// convert Point to Vector
func PointToVector(p Point) Vector {
	return Vector{X: p.X, Y: p.Y}
}

func VectorToPoint(v Vector) Point {
	return Point{X: v.X, Y: v.Y}
}

type VectorInterface interface {
	Length() float64
	Unit() Vector
	Mul(scalar float64) Vector
	Dot(v Vector) float64
	Rot90CW() Vector
	Rot90CCW() Vector
	Intersect(line1 [2]Point, line2 [2]Point) Point
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
func intersect(line1 [2]Point, line2 [2]Point) *Point {
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
		// Lines are parallel, return nil
		return nil
	}

	t := (b1*c2 - b2*c1) / denominator

	// Calculate the intersection point using the parameter t
	return &Point{
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

func Direction(a, b, c Point) int {
	ab := PointToVector(b).Sub(PointToVector(a))
	bc := PointToVector(c).Sub(PointToVector(b))

	angle1 := math.Atan2(ab.Y, ab.X)
	angle2 := math.Atan2(bc.Y, bc.X)

	angleDiff := angle2 - angle1

	fmt.Println(angle1, angle2, angleDiff)

	if angleDiff < 0 {
		return -1
	}
	if angleDiff > 0 {
		if angleDiff > math.Pi {
			return -1
		}
		return 1
	}

	return 0
}

func (v Vector) Sub(v2 Vector) Vector {
	return Vector{X: v.X - v2.X, Y: v.Y - v2.Y}
}

// Angle returns the angle of a vector in radians normalized to 0-2pi
func Angle(v Vector) float64 {
	angle := math.Atan2(v.Y, v.X)
	if angle < 0 {
		angle += 2 * math.Pi
	}
	return angle
}

func (v Vector) Rotate(angle float64) Vector {

	// specially handle the case where the angle is multiple of 90 degrees
	if angle == math.Pi/2 || angle == 3*math.Pi/2 {
		return Vector{X: -v.Y, Y: v.X}
	}

	if angle == math.Pi || angle == 2*math.Pi {
		return Vector{X: -v.X, Y: -v.Y}
	}

	v2 := Vector{
		X: v.X*math.Cos(angle) - v.Y*math.Sin(angle),
		Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle),
	}

	return v2
}

func (v Vector) Add(v2 Vector) Vector {
	return Vector{X: v.X + v2.X, Y: v.Y + v2.Y}
}

func (v Vector) ToPoint() Point {
	return Point{X: v.X, Y: v.Y}
}

func (v Vector) Reverse() Vector {
	return Vector{X: -v.X, Y: -v.Y}
}

func (v Vector) Dot(v2 Vector) float64 {
	return v.X*v2.X + v.Y*v2.Y
}

func (v Vector) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y)
}

func (v Vector) SetLength(length float64) Vector {
	return Vector{X: v.X * length / v.Length(), Y: v.Y * length / v.Length()}
}

func (v Vector) Unit() Vector {
	len := v.Length()
	if len == 0 {
		return Vector{X: 0, Y: 0}
	}
	return Vector{X: v.X / len, Y: v.Y / len}
}

func (v Vector) Scale(scalar float64) Vector {
	return Vector{X: v.X * scalar, Y: v.Y * scalar}
}

func (v Vector) Project(v2 Vector) Vector {
	unit := v2.Unit()
	return unit.Scale(v.Dot(unit))
}

func (v Vector) Middle(v2 Vector) Vector {
	return Vector{
		X: (v.X + v2.X) / 2,
		Y: (v.Y + v2.Y) / 2,
	}
}

func RotateBoundary(boundary []Point, angle float64) []Point {
	for i := range boundary {
		boundary[i] = PointToVector(boundary[i]).Rotate(angle).ToPoint()
	}
	return boundary
}

func (v Vector) Perpendicular() Vector {
	return Vector{X: -v.Y, Y: v.X}
}

func VectorsToPoints(vectors []Vector) []Point {
	points := make([]Point, len(vectors))
	for i := range vectors {
		points[i] = vectors[i].ToPoint()
	}
	return points
}

func (p Point) Distance(a Point) float64 {

	// distance from point p to point a

	return math.Sqrt(math.Pow(p.X-a.X, 2) + math.Pow(p.Y-a.Y, 2))
}
