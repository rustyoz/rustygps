package planner

// BezierCurve represents a Bezier curve defined by control points
type BezierCurve struct {
	ControlPoints []Point
}

// NewBezierCurve creates a new Bezier curve with the given control points
func NewBezierCurve(points ...Point) *BezierCurve {
	return &BezierCurve{
		ControlPoints: points,
	}
}

// EvaluateAt calculates a point on the Bezier curve at parameter t (between 0 and 1)
func (b *BezierCurve) EvaluateAt(t float64) Point {
	if len(b.ControlPoints) == 0 {
		return Point{}
	}

	if t <= 0 {
		return b.ControlPoints[0]
	}

	if t >= 1 {
		return b.ControlPoints[len(b.ControlPoints)-1]
	}

	// De Casteljau's algorithm
	points := make([]Point, len(b.ControlPoints))
	copy(points, b.ControlPoints)

	for r := 1; r < len(b.ControlPoints); r++ {
		for i := 0; i < len(b.ControlPoints)-r; i++ {
			points[i] = Point{
				X: (1-t)*points[i].X + t*points[i+1].X,
				Y: (1-t)*points[i].Y + t*points[i+1].Y,
			}
		}
	}

	return points[0]
}

// GeneratePoints generates n points along the Bezier curve
func (b *BezierCurve) GeneratePoints(numPoints int) []Point {
	if numPoints <= 0 {
		return nil
	}

	points := make([]Point, numPoints)
	for i := 0; i < numPoints; i++ {
		t := float64(i) / float64(numPoints-1)
		points[i] = b.EvaluateAt(t)
		points[i].Color = "red"
	}

	return points
}

// QuadraticBezier creates a quadratic Bezier curve with 3 control points
func QuadraticBezier(p0, p1, p2 Point) *BezierCurve {
	return NewBezierCurve(p0, p1, p2)
}

// CubicBezier creates a cubic Bezier curve with 4 control points
func CubicBezier(p0, p1, p2, p3 Point) *BezierCurve {
	return NewBezierCurve(p0, p1, p2, p3)
}
