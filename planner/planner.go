package planner

import (
	"errors"
	"fmt"
	"math"

	"github.com/rustyoz/rustygps/implement"
	"github.com/rustyoz/rustygps/tractor"
)

// Point represents a 2D point in the field
type Point struct {
	X       float64
	Y       float64
	Visited bool
	End     bool
}

// ABLine represents a line defined by two points
type ABLine struct {
	PointA Point
	PointB Point
}

// Field represents the field boundary and other field-specific data
type Field struct {
	Boundary    []Point // Field boundary points in clockwise order
	WorkingArea []Point // Inner working area boundary points in clockwise order
	ABLine      ABLine  // Reference AB line for parallel tracking
}

// PathPlanner interface defines methods that all path planning algorithms must implement
type PathPlanner interface {
	GeneratePath(field Field, tractor tractor.Tractor, implement implement.Implement) ([]Point, error)
}

// ABLinePlanner implements parallel path planning based on AB lines
type ABLinePlanner struct{}

// NewABLinePlanner creates a new AB line path planner
func NewABLinePlanner() *ABLinePlanner {
	return &ABLinePlanner{}
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

// GenerateInnerBoundary creates an inner boundary offset from the field boundary
// using an intersection algorithm for more accurate results
// offset: distance to offset inward in meters (positive value)
func GenerateInnerBoundary(boundary []Point, offset float64) []Point {
	if len(boundary) < 3 || offset <= 0 {
		return nil
	}

	// Create a new slice to hold the inner boundary points
	innerBoundary := make([]Point, 0, len(boundary))

	// Determine if the polygon is clockwise
	isCw := polyIsCw(boundary)

	// Choose the appropriate rotation function based on polygon orientation
	// For a clockwise polygon, we need to rotate inward (counterclockwise)
	// For a counterclockwise polygon, we need to rotate inward (clockwise)
	rotFunc := vecRot90CCW
	if isCw {
		rotFunc = vecRot90CW
	}

	// For each vertex in the boundary
	for i := 0; i < len(boundary); i++ {
		// Get current point and neighboring points
		prev := boundary[(i+len(boundary)-1)%len(boundary)]
		curr := boundary[i]
		next := boundary[(i+1)%len(boundary)]

		// Calculate vectors for the two edges
		v01 := Vector{X: curr.X - prev.X, Y: curr.Y - prev.Y}
		v12 := Vector{X: next.X - curr.X, Y: next.Y - curr.Y}

		// Calculate perpendicular offset vectors scaled by the offset distance
		d01 := vecMul(vecUnit(rotFunc(v01)), offset)
		d12 := vecMul(vecUnit(rotFunc(v12)), offset)

		// Calculate points on the offset lines
		ptx0 := Point{X: prev.X + d01.X, Y: prev.Y + d01.Y}
		ptx10 := Point{X: curr.X + d01.X, Y: curr.Y + d01.Y}
		ptx12 := Point{X: curr.X + d12.X, Y: curr.Y + d12.Y}
		ptx2 := Point{X: next.X + d12.X, Y: next.Y + d12.Y}

		// Find the intersection of the two offset lines
		line1 := [2]Point{ptx0, ptx10}
		line2 := [2]Point{ptx12, ptx2}
		intersection := intersect(line1, line2)

		// Add the intersection point to the inner boundary
		innerBoundary = append(innerBoundary, intersection)
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

// GenerateHeadlandCurve generates a smooth U-turn between two parallel lines
func GenerateHeadlandCurve(start Point, end Point, heading float64, direction int, intervalDistance float64, headlandOffset float64) []Point {

	//fmt.Println("Generating headland curve")
	// Calculate the centers for a three-arc turn with smooth transitions
	entryArcCenter, mainArcCenter, exitArcCenter, minorRadius, mainRadius, entryArcLength := CalculateThreeArcCenters(start, end, direction, headlandOffset)

	var points []Point

	// Calculate entry arc points
	numEntryPoints := 9

	// Starting angle
	//entryStartAngle := heading

	//fmt.Printf("entryStartAngle: %v\n", entryStartAngle*180/math.Pi)
	//fmt.Printf("entryArcLength: %v\n", entryArcLength*180/math.Pi)
	//	fmt.Printf("start point: %v\n", start)
	//	fmt.Printf("end point: %v\n", end)
	//	fmt.Printf("entry arc centre: %v\n", entryArcCenter)
	//	fmt.Printf("exit arc centre: %v\n", exitArcCenter)
	//	fmt.Printf("direction: %v\n", direction)
	// Calculate points for entry arc
	var currentAngle float64
	for i := 0; i < numEntryPoints; i++ {
		t := float64(i) / float64(numEntryPoints)
		currentAngle = math.Pi/2 - t*entryArcLength*float64(direction) // for left headland, first turn right
		///		fmt.Printf("angle: %v\n", currentAngle*180/math.Pi)
		x := minorRadius * math.Cos(currentAngle)
		y := minorRadius * math.Sin(currentAngle)
		p := Point{X: x + entryArcCenter.X, Y: y + entryArcCenter.Y}
		//	fmt.Printf("entry point: %v\n", p)

		points = append(points, p)
	}

	// Calculate main arc angles
	mainStartAngle := math.Pi - (currentAngle - math.Pi/2)
	// Normalize to be between 0 and 2π
	mainStartAngle = math.Mod(mainStartAngle+2*math.Pi, 2*math.Pi)

	// Adjust for direction if needed
	mainArcLength := math.Pi + 2*entryArcLength // 270 degrees

	//fmt.Printf("mainstartAngle: %v mainArcLength: %v entrystartAngle: %v\n", mainStartAngle*180/math.Pi, mainArcLength*180/math.Pi, entryArcLength*180/math.Pi)
	numMainPoints := 27

	for i := 0; i < numMainPoints; i++ {
		t := float64(i) / float64(numMainPoints)
		currentAngle = mainStartAngle - t*mainArcLength*float64(direction)
		//		fmt.Println("main agnle:", currentAngle*180/math.Pi)
		x := mainArcCenter.X + mainRadius*math.Sin(currentAngle)
		y := mainArcCenter.Y + mainRadius*math.Cos(currentAngle)
		//		fmt.Printf("main point: %v\n", Point{X: x, Y: y})
		points = append(points, Point{X: x, Y: y})
	}

	// Calculate exit arc points
	exitStartAngle := currentAngle
	exitArcLength := entryArcLength
	numExitPoints := 9
	//fmt.Println("currentAngle: ", currentAngle*180/math.Pi)
	//fmt.Println("exitStartAngle: ", exitStartAngle*180/math.Pi)
	//fmt.Println("exitArcLength: ", exitArcLength*180/math.Pi)
	for i := 0; i < numExitPoints; i++ {
		t := float64(i) / float64(numExitPoints)
		currentAngle = exitStartAngle + t*exitArcLength*float64(direction)
		x := exitArcCenter.X - minorRadius*math.Sin(currentAngle)
		y := exitArcCenter.Y - minorRadius*math.Cos(currentAngle)

		//		fmt.Printf("exit point: %v\n", Point{X: x, Y: y})
		points = append(points, Point{X: x, Y: y})
	}

	//fmt.Println("currentAngle: ", currentAngle*180/math.Pi)

	return points
}

// CalculateThreeArcCenters calculates the centers of three arcs for a smooth headland turn
// Returns the center points and radii for entry, main, and exit arcs
func CalculateThreeArcCenters(start Point, end Point, direction int, headlandOffset float64) (Point, Point, Point, float64, float64, float64) {
	// Calculate vector from start to end
	vector := Point{
		X: end.X - start.X,
		Y: end.Y - start.Y,
	}

	midpoint := Point{
		X: (start.X + end.X) / 2,
		Y: (start.Y + end.Y) / 2,
	}
	//fmt.Printf("midpoint: %v\n", midpoint)

	// Calculate distance between start and end points
	distance := math.Sqrt(vector.X*vector.X + vector.Y*vector.Y)
	//fmt.Printf("distance: %v\n", distance)

	// Normalize vector
	vector.X /= distance
	vector.Y /= distance

	// Calculate perpendicular vector based on direction
	var perpVector Point
	if direction == 1 {
		// Rotate 90 degrees clockwise
		perpVector = Point{X: vector.Y, Y: -vector.X}
	} else {
		// Rotate 90 degrees counterclockwise
		perpVector = Point{X: -vector.Y, Y: vector.X}
	}

	// push start and end points outwards by headland offset
	//start = Point{X: start.X + perpVector.X*headlandOffset, Y: start.Y + perpVector.Y*headlandOffset}
	//end = Point{X: end.X + perpVector.X*headlandOffset, Y: end.Y + perpVector.Y*headlandOffset}

	// Main arc radius
	mainRadius := distance
	minorRadius := mainRadius

	x := distance / 2

	// Calculate main arc center offset
	mH := (mainRadius + minorRadius) * (mainRadius + minorRadius) // hypotenuse
	mA := (minorRadius + x) * (minorRadius + x)                   // adjacent
	mainCentreOffset := math.Sqrt(mH - mA)                        // oposite

	//fmt.Printf("mH: %v, mA: %v, mainCentreOffset: %v\n", mH, mA, mainCentreOffset)

	angle := math.Acos((minorRadius + x) / (mainRadius + minorRadius))
	//fmt.Printf("Acos(mA/mH): %v\n", angle*180/math.Pi)

	mainCenter := Point{
		X: midpoint.X + perpVector.X*mainCentreOffset,
		Y: midpoint.Y + perpVector.Y*mainCentreOffset,
	}

	// Calculate entry arc center
	//fmt.Println("entry arc")
	//fmt.Printf("start: %v\n", start)
	//fmt.Printf("vector: %v\n", vector)
	//fmt.Printf("perpVector: %v\n", perpVector)
	//fmt.Printf("minorRadius: %v\n", minorRadius)
	// entry centre is start point minus vector times entry radius
	entryCenter := Point{
		X: start.X - vector.X*minorRadius,
		Y: start.Y - vector.Y*minorRadius,
	}

	// Calculate exit arc center
	// exit centre is end point plus vector times exit radius
	exitCenter := Point{
		X: end.X + vector.X*minorRadius,
		Y: end.Y + vector.Y*minorRadius,
	}

	return entryCenter, mainCenter, exitCenter, minorRadius, mainRadius, angle
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

// GeneratePath implements the PathPlanner interface for AB line planning
func (p *ABLinePlanner) GeneratePath(field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, error) {

	fmt.Printf("Generating path: Implement width: %.1f\n", implement.WorkingWidth)

	if implement.WorkingWidth <= 0 {
		return nil, errors.New("implement width must be positive")
	}

	// Calculate field dimensions from boundary
	minX, maxX, minY, maxY := getBoundingBox(field.Boundary)
	fmt.Printf("minX: %v, maxX: %v, minY: %v, maxY: %v\n", minX, maxX, minY, maxY)

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

	// find the point furthest from the ab line in the direction of perpX and perpY
	maxDistance := 0.0
	for _, p := range field.Boundary {
		distance := math.Abs(p.X*perpX + p.Y*perpY)
		if distance > maxDistance {
			maxDistance = distance
		}
	}

	// maximum length of a pass
	numPasses := int(math.Ceil(maxDistance / implement.WorkingWidth))
	fmt.Printf("numPasses: %v\n", numPasses)

	const intervalDistance = 1.0 // Distance between points in meters

	// Generate parallel lines with fixed interval points
	var path []Point
	for i := 0; i < numPasses; i++ {
		//fmt.Printf("pass: %v\n", i)
		offset := float64(i) * implement.WorkingWidth

		// Calculate start and end points for this pass
		startPoint := Point{
			X: field.ABLine.PointA.X + perpX*offset,
			Y: field.ABLine.PointA.Y + perpY*offset,
		}
		endPoint := Point{
			X: field.ABLine.PointA.X + dx*maxDistance + perpX*offset,
			Y: field.ABLine.PointA.Y + dy*maxDistance + perpY*offset,
		}

		// reverse every second pass
		if i%2 == 1 {
			startPoint, endPoint = endPoint, startPoint
		}

		// Calculate number of intervals needed
		lineLength := math.Sqrt(
			math.Pow(endPoint.X-startPoint.X, 2) +
				math.Pow(endPoint.Y-startPoint.Y, 2),
		)
		numIntervals := int(math.Ceil(lineLength / intervalDistance))

		// Generate points along the line at fixed intervals

		//fmt.Printf("start point: %v\n", startPoint)
		for j := 0; j <= numIntervals; j++ {
			t := float64(j) / float64(numIntervals)
			path = append(path, Point{
				X: startPoint.X + (endPoint.X-startPoint.X)*t,
				Y: startPoint.Y + (endPoint.Y-startPoint.Y)*t,
			})
		}
		// append end point
		//fmt.Printf("end point: %v\n", endPoint)
		endPoint.End = true
		path = append(path, endPoint)

	}

	field.WorkingArea = GenerateInnerBoundary(field.Boundary, implement.WorkingWidth/2)

	path = ClipPath(path, field.WorkingArea)

	_, err := GenerateHeadlandPath(path, field, tractor, implement)
	if err != nil {
		return nil, err
	}

	return path, nil
}

func ClipPath(path []Point, boundary []Point) []Point {

	var newPath []Point

	var lastGoodPoint *Point
	for _, p := range path {
		if IsPointInField(boundary, p) {
			newPath = append(newPath, p)

			lastGoodPoint = &p
		} else {
			//fmt.Printf("point %v is outside the field boundary\n", p)
			if lastGoodPoint != nil {
				lastGoodPoint.End = true
				// derefernce lastGoodPoint
				lastGoodPoint = nil
			}
		}
	}

	return newPath
}

func GenerateHeadlandPath(path []Point, field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, error) {

	// clip path
	clippedPath := ClipPath(path, field.WorkingArea)

	headlandPath := []Point{}
	// scan clip path point by point
	headlandPath = append(headlandPath, clippedPath[0])
	for i := 1; i < len(clippedPath); i++ {
		if clippedPath[i-1].End {
			headlandCurve := GenerateHeadlandCurve(clippedPath[i-1], clippedPath[i], 0, 1, 0.5, implement.WorkingWidth)
			headlandPath = append(headlandPath, headlandCurve...)

		}
		headlandPath = append(headlandPath, clippedPath[i])
	}

	return headlandPath, nil
}
