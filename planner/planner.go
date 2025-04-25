package planner

import (
	"errors"
	"math"
)

// PlannerConfig holds configuration parameters for path planning
type PlannerConfig struct {
	ImplementWidth      float64 // Width of the implement in meters
	TractorTurnRadius   float64 // Minimum turning radius of the tractor in meters
	ImplementTurnRadius float64 // Minimum turning radius of the implement in meters
}

// Point represents a 2D point in the field
type Point struct {
	X       float64
	Y       float64
	Visited bool
}

// ABLine represents a line defined by two points
type ABLine struct {
	PointA Point
	PointB Point
}

// Field represents the field boundary and other field-specific data
type Field struct {
	Boundary []Point // Field boundary points in clockwise order
	ABLine   ABLine  // Reference AB line for parallel tracking
}

// PathPlanner interface defines methods that all path planning algorithms must implement
type PathPlanner interface {
	GeneratePath(field Field, config PlannerConfig) ([]Point, error)
}

// ABLinePlanner implements parallel path planning based on AB lines
type ABLinePlanner struct{}

// NewABLinePlanner creates a new AB line path planner
func NewABLinePlanner() *ABLinePlanner {
	return &ABLinePlanner{}
}

// GeneratePath implements the PathPlanner interface for AB line planning
func (p *ABLinePlanner) GeneratePath(field Field, config PlannerConfig) ([]Point, error) {

	//fmt.Println("Generating path")

	if config.ImplementWidth <= 0 {
		return nil, errors.New("implement width must be positive")
	}

	// Calculate field dimensions from boundary
	minX, maxX, minY, maxY := getBoundingBox(field.Boundary)
	fieldWidth := maxX - minX
	fieldLength := maxY - minY

	//fmt.Println("fieldWidth: ", fieldWidth, "fieldLength: ", fieldLength)

	// print ab line
	//fmt.Printf("abLine: %v\n", field.ABLine)

	// Calculate AB line direction vector
	dx := field.ABLine.PointB.X - field.ABLine.PointA.X
	dy := field.ABLine.PointB.Y - field.ABLine.PointA.Y
	length := math.Sqrt(dx*dx + dy*dy)

	if length == 0 {
		return nil, errors.New("invalid AB line: points A and B are the same")
	}

	// Normalize direction vector
	dx /= length
	dy /= length

	// Calculate perpendicular vector for parallel lines
	perpX := dy
	perpY := -dx

	// print perpendicular vector
	//fmt.Printf("perpX: %v, perpY: %v\n", perpX, perpY)

	// find the centre of the field
	centre := FindCentreOfField(field.Boundary)

	// print centre
	//fmt.Printf("centre: %v\n", centre)

	// find the side of the ab line
	side := DetermineSideOfABLine(field.ABLine, centre)

	switch side {
	case 1:
		//fmt.Println("left")
	case -1:
		//fmt.rintln("right")
	case 0:
		//fmt.Println("on line")
	}

	// if side is 0, fatal error
	if side == 0 {
		panic("centre is on the ab line")
	}

	// invert the perpendicular vector if the point is on the left side of the AB line
	if side == -1 {
		perpX = -perpX
		perpY = -perpY
	}

	// Calculate number of passes needed to cover field width
	projectionLength := math.Abs(fieldWidth*perpX + fieldLength*perpY)
	numPasses := int(math.Ceil(projectionLength / config.ImplementWidth))

	const intervalDistance = 1.0 // Distance between points in meters

	// Generate parallel lines with fixed interval points
	var path []Point
	var lastEndPoint *Point
	for i := 0; i < numPasses; i++ {
		offset := float64(i) * config.ImplementWidth

		// Calculate start and end points for this pass
		startPoint := Point{
			X: field.ABLine.PointA.X + perpX*offset,
			Y: field.ABLine.PointA.Y + perpY*offset,
		}
		endPoint := Point{
			X: field.ABLine.PointB.X + perpX*offset,
			Y: field.ABLine.PointB.Y + perpY*offset,
		}

		// reverse every second pass
		if i%2 == 1 {
			startPoint, endPoint = endPoint, startPoint
		}

		if lastEndPoint != nil {
			points := GenerateHeadlandCurve(*lastEndPoint, startPoint, side-(i%2)*2*side, 0.5, config.ImplementWidth/2)
			path = append(path, points...)
		}

		lastEndPoint = &endPoint

		// Calculate number of intervals needed
		lineLength := math.Sqrt(
			math.Pow(endPoint.X-startPoint.X, 2) +
				math.Pow(endPoint.Y-startPoint.Y, 2),
		)
		numIntervals := int(math.Ceil(lineLength / intervalDistance))

		// Generate points along the line at fixed intervals

		for j := 0; j <= numIntervals; j++ {
			t := float64(j) / float64(numIntervals)
			path = append(path, Point{
				X: startPoint.X + (endPoint.X-startPoint.X)*t,
				Y: startPoint.Y + (endPoint.Y-startPoint.Y)*t,
			})
		}

	}

	return path, nil
}

// getBoundingBox calculates the minimum and maximum coordinates of the field boundary
func getBoundingBox(boundary []Point) (minX, maxX, minY, maxY float64) {
	if len(boundary) == 0 {
		return 0, 0, 0, 0
	}

	minX, maxX = boundary[0].X, boundary[0].X
	minY, maxY = boundary[0].Y, boundary[0].Y

	for _, p := range boundary {
		if p.X < minX {
			minX = p.X
		}
		if p.X > maxX {
			maxX = p.X
		}
		if p.Y < minY {
			minY = p.Y
		}
		if p.Y > maxY {
			maxY = p.Y
		}
	}
	return
}

// CreateABLine creates an AB line from two points
func CreateABLine(pointA, pointB Point) ABLine {
	return ABLine{
		PointA: pointA,
		PointB: pointB,
	}
}

// Find AB Line, find the longest line in the boundary and return it
func FindABLine(boundary []Point) *ABLine {
	if len(boundary) < 2 {
		return nil // Return empty AB line if boundary has fewer than 2 points
	}

	var maxDistance float64
	var longestLine ABLine

	// Check each pair of consecutive points
	for i := 0; i < len(boundary); i++ {
		// Get current point and next point (wrapping around to start)
		current := boundary[i]
		next := boundary[(i+1)%len(boundary)]

		// Calculate distance between points
		distance := math.Sqrt(
			math.Pow(next.X-current.X, 2) +
				math.Pow(next.Y-current.Y, 2),
		)

		// Update longest line if this distance is greater
		if distance > maxDistance {
			maxDistance = distance
			longestLine = ABLine{
				PointA: current,
				PointB: next,
			}
		}
	}

	//fmt.Printf("longestLine: %v\n", longestLine)

	return &longestLine
}

// NewField creates a new field with the given boundary and AB line
func NewField(boundary []Point, abLine ABLine) Field {
	return Field{
		Boundary: boundary,
		ABLine:   abLine,
	}
}

// IsPointInField determines whether a point lies inside the field boundary
// using the ray casting algorithm
func IsPointInField(boundary []Point, point Point) bool {
	if len(boundary) < 3 {
		return false // A field must have at least 3 points to form a polygon
	}

	inside := false
	j := len(boundary) - 1

	for i := 0; i < len(boundary); i++ {
		// Check if point is on boundary line
		if (boundary[i].Y > point.Y) != (boundary[j].Y > point.Y) {
			// Calculate intersection point
			if point.X < (boundary[j].X-boundary[i].X)*(point.Y-boundary[i].Y)/
				(boundary[j].Y-boundary[i].Y)+boundary[i].X {
				inside = !inside
			}
		}
		j = i
	}

	return inside
}

// IsPointInFieldWinding determines whether a point lies inside the field boundary
// using the winding number algorithm
func IsPointInFieldWinding(boundary []Point, point Point) bool {
	if len(boundary) < 3 {
		return false
	}

	wn := 0 // winding number counter

	// Loop through all edges of the polygon
	for i := 0; i < len(boundary)-1; i++ {
		if boundary[i].Y <= point.Y {
			if boundary[i+1].Y > point.Y {
				// Upward crossing
				if isLeft(boundary[i], boundary[i+1], point) > 0 {
					wn++
				}
			}
		} else {
			if boundary[i+1].Y <= point.Y {
				// Downward crossing
				if isLeft(boundary[i], boundary[i+1], point) < 0 {
					wn--
				}
			}
		}
	}

	return wn != 0
}

// isLeft tests if point P is left of the line through points A and B
func isLeft(a, b, p Point) float64 {
	return (b.X-a.X)*(p.Y-a.Y) - (p.X-a.X)*(b.Y-a.Y)
}

// IsPointInFieldAngle determines whether a point lies inside the field boundary
// using the angle summation method
func IsPointInFieldAngle(boundary []Point, point Point) bool {
	if len(boundary) < 3 {
		return false
	}

	angle := 0.0

	for i := 0; i < len(boundary); i++ {
		p1 := boundary[i]
		p2 := boundary[(i+1)%len(boundary)]

		// Calculate vectors from point to vertices
		dx1 := p1.X - point.X
		dy1 := p1.Y - point.Y
		dx2 := p2.X - point.X
		dy2 := p2.Y - point.Y

		// Calculate angle using atan2
		angle += math.Atan2(
			dx1*dy2-dy1*dx2,
			dx1*dx2+dy1*dy2,
		)
	}

	// Point is inside if absolute angle is close to 2π
	return math.Abs(angle) > 6.0 // approximately 2π - small epsilon
}

// GenerateInnerBoundary creates an inner boundary offset from the field boundary
// offset: distance to offset inward in meters (positive value)
func GenerateInnerBoundary(boundary []Point, offset float64) []Point {
	if len(boundary) < 3 || offset <= 0 {
		return nil
	}

	var innerBoundary []Point

	// For each vertex in the boundary
	for i := 0; i < len(boundary); i++ {
		// Get current point and neighboring points
		prev := boundary[(i+len(boundary)-1)%len(boundary)]
		curr := boundary[i]
		next := boundary[(i+1)%len(boundary)]

		// Calculate vectors for the two edges
		v1x := curr.X - prev.X
		v1y := curr.Y - prev.Y
		v2x := next.X - curr.X
		v2y := next.Y - curr.Y

		// Normalize vectors
		len1 := math.Sqrt(v1x*v1x + v1y*v1y)
		len2 := math.Sqrt(v2x*v2x + v2y*v2y)

		v1x /= len1
		v1y /= len1
		v2x /= len2
		v2y /= len2

		// Calculate perpendicular vectors (inward pointing)
		n1x := -v1y
		n1y := v1x
		n2x := -v2y
		n2y := v2x

		// Calculate angle between segments
		angle := math.Acos(v1x*v2x + v1y*v2y)

		// Calculate offset distance based on angle
		offsetDist := offset
		if angle < math.Pi {
			// For internal angles, adjust offset to maintain consistent distance
			offsetDist = offset / math.Sin(angle/2)
		}

		// Calculate bisector vector
		bisectorX := (n1x + n2x) / 2
		bisectorY := (n1y + n2y) / 2

		// Normalize bisector
		bisectorLen := math.Sqrt(bisectorX*bisectorX + bisectorY*bisectorY)
		if bisectorLen > 0 {
			bisectorX /= bisectorLen
			bisectorY /= bisectorLen
		}

		// Calculate new point position
		newPoint := Point{
			X: curr.X + bisectorX*offsetDist,
			Y: curr.Y + bisectorY*offsetDist,
		}

		innerBoundary = append(innerBoundary, newPoint)
	}

	return innerBoundary
}

// GetHeadlandArea returns the points that make up the headland area
// (area between the original boundary and inner boundary)
func GetHeadlandArea(outerBoundary, innerBoundary []Point) []Point {
	// Create a slice to hold all points that define the headland area
	headlandArea := make([]Point, 0, len(outerBoundary)+len(innerBoundary))

	// Add outer boundary points
	headlandArea = append(headlandArea, outerBoundary...)

	// Add inner boundary points in reverse order to create a closed loop
	for i := len(innerBoundary) - 1; i >= 0; i-- {
		headlandArea = append(headlandArea, innerBoundary[i])
	}

	return headlandArea
}

// this function takes two points, the end and start of the two parrallel lines and generates an arc between them
func GenerateHeadlandCurve(start Point, end Point, direction int, intervalDistance float64, headlandoffset float64) []Point {

	// print parameters to console, format to 1 decimal place
	//fmt.Printf("headland curve start: x: %.1f, y: %.1f, end: x: %.1f, y: %.1f, direction: %v, intervalDistance: %.1f, headlandoffset: %.1f\n", start.X, start.Y, end.X, end.Y, direction, intervalDistance, headlandoffset)

	// find the center of the arc
	center := Point{
		X: (start.X + end.X) / 2,
		Y: (start.Y + end.Y) / 2,
	}

	// shift the center of the arc by the headland offset
	// first find the vector from the start to the end
	vector := Point{
		X: end.X - start.X,
		Y: end.Y - start.Y,
	}
	// normalize vector
	vectorLen := math.Sqrt(vector.X*vector.X + vector.Y*vector.Y)
	vector.X /= vectorLen
	vector.Y /= vectorLen

	//fmt.Printf("direction: %v\n", direction)
	if direction == 1 {
		//	fmt.Println("left turn")
	} else {
		//	fmt.Println("right turn")
	}

	// rotate the vector by 90 degrees
	if direction == 1 {
		// rotate 90 degrees clockwise
		vector.X, vector.Y = vector.Y, -vector.X
	} else {
		// rotate 90 degrees counterclockwise
		vector.X, vector.Y = -vector.Y, vector.X
	}

	//fmt.Printf("vector after rotation: x: %.1f, y: %.1f\n", vector.X, vector.Y)

	// shift the center by the headland offset
	center.X += vector.X * headlandoffset
	center.Y += vector.Y * headlandoffset

	//fmt.Printf("arc center: x: %.1f, y: %.1f\n", center.X, center.Y)

	// find the radius of the arc
	radius := math.Sqrt(
		math.Pow(start.X-center.X, 2) +
			math.Pow(start.Y-center.Y, 2),
	)

	angleOffset := math.Atan2(start.Y-center.Y, start.X-center.X)

	// find the angle of the arc
	angle1 := math.Atan2(start.Y-center.Y, start.X-center.X)
	// normalize the angle to be between 0 and 2pi
	angle1 = math.Mod(angle1+2*math.Pi, 2*math.Pi)
	// normalize the angle to be between 0 and 2pi
	angle2 := math.Atan2(end.Y-center.Y, end.X-center.X)
	angle2 = math.Mod(angle2+2*math.Pi, 2*math.Pi)
	var angle float64
	// if direction is -1, invert the angle calculation
	if direction == 1 {
		angle = angle2 - angle1
		if angle < 0 {
			angle += 2 * math.Pi
		}
	} else {
		angle = angle1 - angle2
		if angle < 0 {
			angle += 2 * math.Pi
		}
	}

	//fmt.Printf("angle1: %v, angle2: %v, angle: %v\n", angle1, angle2, angle)

	// find the arclength
	arclength := radius * angle
	// find the number of points in the arc
	numPoints := int(math.Ceil(arclength / intervalDistance))

	var points []Point

	//fmt.Printf("centre: %v, radius: %v, angle: %v, arclength: %v, numPoints: %v, angleoffset: %v\n", center, radius, angle, arclength, numPoints, angleOffset)

	// shift the centre again by the headland offset
	center.X += vector.X * headlandoffset
	center.Y += vector.Y * headlandoffset

	// generate the points in the arc, starting at the start point and going in the direction of the arc, but don't include the start and end points
	for i := 0; i < numPoints; i++ {
		theta := float64(i) * angle / float64(numPoints)
		x := center.X + radius*math.Cos(theta*float64(direction)+angleOffset)
		y := center.Y + radius*math.Sin(theta*float64(direction)+angleOffset)
		points = append(points, Point{X: x, Y: y})
	}

	return points
}

// Find centre of field
func FindCentreOfField(boundary []Point) Point {
	// find the center of the field by averaging the x and y coordinates of the boundary points
	center := Point{
		X: 0,
		Y: 0,
	}
	count := len(boundary)

	// find the center of the field by averaging the x and y coordinates of the boundary points
	for _, p := range boundary {
		center.X += p.X / float64(count)
		center.Y += p.Y / float64(count)
	}
	return center
}

// determine which side of the AB line a point lies on
// returns 1 if the point is on the left side, -1 if the point is on the right side, 0 if the point is on the line
func DetermineSideOfABLine(abLine ABLine, point Point) int {
	side := (point.X-abLine.PointA.X)*(abLine.PointB.Y-abLine.PointA.Y) - (point.Y-abLine.PointA.Y)*(abLine.PointB.X-abLine.PointA.X)
	if side > 0 {
		return 1
	} else if side < 0 {
		return -1
	} else {
		return 0
	}
}
