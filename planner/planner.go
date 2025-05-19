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
	Color   string
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
	GeneratePath(field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, []Point, error)
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
		if distance >= maxDistance {
			maxDistance = distance
			longestLine = ABLine{
				PointA: next,
				PointB: current,
			}
		}
	}

	fmt.Printf("longestLine: %v\n", longestLine)

	return &longestLine
}

// NewField creates a new field with the given boundary and AB line
func NewField(boundary []Point) Field {

	return Field{
		Boundary: boundary,
	}
}

// check if point lies between the start and end point, return true if it does
func IsPointOnLine(start Point, end Point, point Point) bool {
	// Calculate vectors
	lineVec := Point{X: end.X - start.X, Y: end.Y - start.Y}
	pointVec := Point{X: point.X - start.X, Y: point.Y - start.Y}

	// Calculate cross product to check if point is collinear with line
	crossProduct := lineVec.X*pointVec.Y - lineVec.Y*pointVec.X

	// If cross product is not close to zero, point is not on the line
	const epsilon = 0.0001
	if math.Abs(crossProduct) > epsilon {
		return false
	}

	// Check if point is between start and end using dot product
	dotProduct := lineVec.X*pointVec.X + lineVec.Y*pointVec.Y

	// If dot product is negative, point is before start
	if dotProduct < 0 {
		return false
	}

	// If dot product is greater than squared length of line, point is after end
	squaredLength := lineVec.X*lineVec.X + lineVec.Y*lineVec.Y
	if dotProduct > squaredLength {
		return false
	}

	// Point is on the line segment
	return true
}

// check if a points is on the boundary line, return true if it is
func IsPointOnBoundary(boundary []Point, point Point) bool {
	for i := 0; i < len(boundary)-1; i++ {
		if IsPointOnLine(boundary[i], boundary[i+1], point) {
			return true
		}
	}
	return false
}

// IsPointInField determines whether a point lies on or nside the field boundary
// using the ray casting algorithm
func IsPointInField(boundary []Point, point Point) bool {

	// check if points is equal to any boundary point
	for _, p := range boundary {
		if p.X == point.X && p.Y == point.Y {
			return true
		}
	}

	if IsPointOnBoundary(boundary, point) {
		return true
	}

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

		if intersection != nil {
			innerBoundary = append(innerBoundary, *intersection)
		} else {
			return nil
		}

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
func GenerateHeadlandCurve(previousPoint Point, start Point, end Point, intervalDistance float64, headlandOffset float64) []Point {

	var points []Point

	// Calculate entry arc points
	numEntryPoints := 9

	// Starting angle
	entryStartAngle := math.Atan2(start.Y-previousPoint.Y, start.X-previousPoint.X)

	currentAngle := entryStartAngle

	// determine direction of turn
	direction := Direction(previousPoint, start, end)

	//fmt.Println("Generating headland curve")
	// Calculate the centers for a three-arc turn with smooth transitions
	entryArcCenter, mainArcCenter, exitArcCenter, minorRadius, mainRadius, entryArcLength := CalculateThreeArcCenters(Vector{X: start.X, Y: start.Y}, Vector{X: end.X, Y: end.Y}, direction, headlandOffset)

	//fmt.Printf("entryStartAngle: %v\n", entryStartAngle*180/math.Pi)
	//fmt.Printf("entryArcLength: %v\n", entryArcLength*180/math.Pi)
	//	fmt.Printf("start point: %v\n", start)
	//	fmt.Printf("end point: %v\n", end)
	//	fmt.Printf("entry arc centre: %v\n", entryArcCenter)
	//	fmt.Printf("exit arc centre: %v\n", exitArcCenter)
	//fmt.Printf("direction: %v\n", direction)
	// Calculate points for entry arc
	for i := 0; i < numEntryPoints; i++ {
		t := float64(i) / float64(numEntryPoints)
		currentAngle = -math.Pi/2 + entryStartAngle - t*entryArcLength*float64(direction) // for left headland, first turn right
		///		fmt.Printf("angle: %v\n", currentAngle*180/math.Pi)
		if direction == 1 {
			currentAngle = currentAngle - math.Pi
		}

		x := minorRadius * math.Cos(currentAngle)
		y := minorRadius * math.Sin(currentAngle)
		p := Point{X: entryArcCenter.X + x, Y: entryArcCenter.Y + y, Color: "red"}
		//	fmt.Printf("entry point: %v\n", p)

		points = append(points, p)
	}

	// Calculate main arc angles
	mainStartAngle := entryStartAngle + (math.Pi/2 + entryArcLength)

	//fmt.Printf("mainStartAngle: %v currentAngle: %v\n", mainStartAngle*180/math.Pi, currentAngle*180/math.Pi)
	// Adjust for direction if needed
	mainArcLength := math.Pi + 2*entryArcLength // 270 degrees

	//fmt.Printf("mainArcLength: %v\n", mainArcLength*180/math.Pi)
	//fmt.Printf("mainstartAngle: %v mainArcLength: %v entrystartAngle: %v\n", mainStartAngle*180/math.Pi, mainArcLength*180/math.Pi, entryArcLength*180/math.Pi)
	numMainPoints := 27

	for i := 0; i < numMainPoints; i++ {
		t := float64(i) / float64(numMainPoints)
		currentAngle = mainStartAngle*float64(-direction) + t*mainArcLength*float64(direction)

		//		fmt.Println("main agnle:", currentAngle*180/math.Pi)
		x := mainArcCenter.X + mainRadius*math.Cos(currentAngle)
		y := mainArcCenter.Y + mainRadius*math.Sin(currentAngle)
		//		fmt.Printf("main point: %v\n", Point{X: x, Y: y})
		points = append(points, Point{X: x, Y: y, Color: "blue"})
	}

	// Calculate exit arc points
	exitStartAngle := entryStartAngle - entryArcLength + math.Pi/2
	exitArcLength := entryArcLength
	numExitPoints := 9
	//fmt.Println("currentAngle: ", currentAngle*180/math.Pi)
	//fmt.Println("exitStartAngle: ", exitStartcurrentAngleAngle*180/math.Pi)
	//fmt.Println("exitArcLength: ", exitArcLength*180/math.Pi)
	for i := 0; i < numExitPoints; i++ {
		t := float64(i) / float64(numExitPoints)
		currentAngle = exitStartAngle*float64(-direction) - t*exitArcLength*float64(direction)

		x := exitArcCenter.X + minorRadius*math.Cos(currentAngle)
		y := exitArcCenter.Y + minorRadius*math.Sin(currentAngle)

		//fmt.Printf("exit point: %v\n", Point{X: x, Y: y})
		points = append(points, Point{X: x, Y: y, Color: "green"})
	}

	//fmt.Println("currentAngle: ", currentAngle*180/math.Pi)

	return points
}

func GenerateHeadlandBezierCurve(previousPoint Point, start Point, end Point, intervalDistance float64, implement *implement.Implement) ([]Point, []Point) {

	startVector := PointToVector(start)
	endVector := PointToVector(end)

	// vector from previous point to start point
	vector := Vector{X: start.X - previousPoint.X, Y: start.Y - previousPoint.Y}

	turnVector := Vector{X: end.X - start.X, Y: end.Y - start.Y}

	vector = vector.Unit()

	perpVector := vector.Rotate(math.Pi / 2)

	direction := Direction(previousPoint, start, end)

	if direction == -1 {
		perpVector = perpVector.Reverse()
	}

	vectors := []Vector{
		startVector,
		startVector.Add(vector.SetLength(implement.Length)),
		startVector.Add(vector.SetLength(implement.Length * 2)),
		startVector.Add(vector.SetLength(implement.Length * 4)).Add(perpVector.SetLength(3).Reverse()),
		startVector.Add(vector.SetLength(implement.Length * 4)),

		startVector.Add(vector.SetLength(implement.Length * 4)).Add(turnVector.Scale(1.0 / 2.0)),

		endVector.Add(vector.SetLength(implement.Length * 4)).Add(perpVector.SetLength(3)),

		//startVector.Add(vector.SetLength(headlandOffset * 2)).Add(turnVector.SetLength(headlandOffset)),

		//startVector.Add(perpVector.SetLength(projection / 2)).Add(vector.SetLength(headlandOffset)),
		endVector.Add(vector.SetLength(implement.Length * 3)).Add(perpVector.SetLength(3)),

		//endVector.Add(vector.SetLength(implement.Length * 2)),
		endVector.Add(vector.SetLength(implement.Length)),
		endVector,
	}

	//fmt.Printf("firstControlPoint: %v\n", vectors[0].Sub(startVector))
	//fmt.Printf("secondControlPoint: %v\n", vectors[1].Sub(startVector))
	//fmt.Printf("thirdControlPoint: %v\n", vectors[2].Sub(startVector))
	//fmt.Printf("forthControlPoint: %v\n", vectors[3].Sub(startVector))

	// Vec

	// split the curve into two curves
	bezierCurve1 := BezierCurve{
		ControlPoints: VectorsToPoints(vectors[:6]),
	}
	bezierCurve2 := BezierCurve{
		ControlPoints: VectorsToPoints(vectors[5:]),
	}

	bezierCurve1Points := bezierCurve1.GeneratePoints(100)
	bezierCurve2Points := bezierCurve2.GeneratePoints(100)

	points := append(bezierCurve1Points, bezierCurve2Points...)

	return points, VectorsToPoints(vectors)
}

// CalculateThreeArcCenters calculates the centers of three arcs for a smooth headland turn
// Returns the center points and radii for entry, main, and exit arcs
func CalculateThreeArcCenters(start Vector, end Vector, direction int, headlandOffset float64) (Point, Point, Point, float64, float64, float64) {
	//fmt.Println("CalculateThreeArcCenters")

	//fmt.Printf("start: %v\n", start)
	//fmt.Printf("end: %v\n", end)
	//fmt.Printf("direction: %v\n", direction)
	//fmt.Printf("headlandOffset: %v\n", headlandOffset)

	vector := end.Sub(start)

	//fmt.Printf("vector: %v\n", vector)

	midpoint := start.Middle(end)
	//fmt.Printf("midpoint: %v\n", midpoint)

	// Calculate distance between start and end points
	distance := vector.Length()
	//fmt.Printf("distance: %v\n", distance)

	// Normalize vector
	vector = vector.Unit()

	// Calculate perpendicular vector based on direction
	var perpVector Vector
	if direction == 1 {
		perpVector = vector.Rotate(math.Pi / 2)
	} else {
		perpVector = vector.Rotate(-math.Pi / 2)
	}

	//fmt.Printf("perpVector: %v\n", perpVector)

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
		X: midpoint.X - perpVector.X*mainCentreOffset,
		Y: midpoint.Y - perpVector.Y*mainCentreOffset,
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

	//fmt.Printf("entryCenter: %v\n", entryCenter)
	//fmt.Printf("mainCenter: %v\n", mainCenter)
	//fmt.Printf("exitCenter: %v\n", exitCenter)
	//fmt.Printf("minorRadius: %v\n", minorRadius)
	//fmt.Printf("mainRadius: %v\n", mainRadius)
	//fmt.Printf("angle: %v\n", angle*180/math.Pi)
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
func (p *ABLinePlanner) GeneratePath(field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, []Point, error) {

	//fmt.Printf("Generating path: Implement width: %.1f\n", implement.WorkingWidth)

	if implement.WorkingWidth <= 0 {
		return nil, nil, errors.New("implement width must be positive")
	}

	//field.WorkingArea = GenerateInnerBoundary(field.Boundary, implement.WorkingWidth*2.75)

	field.ABLine = *FindABLine(field.Boundary)

	// Calculate field dimensions from boundary
	//minX, maxX, minY, maxY := getBoundingBox(field.WorkingArea)
	//fmt.Printf("minX: %v, maxX: %v, minY: %v, maxY: %v\n", minX, maxX, minY, maxY)

	//fmt.Println("fieldWidth: ", fieldWidth, "fieldLength: ", fieldLength)

	// print ab line
	//fmt.Printf("abLine: %v\n", field.ABLine)

	// Calculate AB line direction vector
	dx := field.ABLine.PointB.X - field.ABLine.PointA.X
	dy := field.ABLine.PointB.Y - field.ABLine.PointA.Y
	length := math.Sqrt(dx*dx + dy*dy)

	if length == 0 {
		return nil, nil, errors.New("invalid AB line: points A and B are the same")
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
	maxPerpDistance := 0.0
	for _, p := range field.Boundary {
		distance := math.Abs((p.X-field.ABLine.PointA.X)*perpX + (p.Y-field.ABLine.PointA.Y)*perpY)
		if distance > maxPerpDistance {
			maxPerpDistance = distance
		}
	}

	// find the point furthest from the ab line start point in the direction of dx and dy from Point A
	maxDistance := 0.0
	for _, p := range field.Boundary {
		distance := math.Abs((p.X-field.ABLine.PointA.X)*dx + (p.Y-field.ABLine.PointA.Y)*dy)
		if distance > maxDistance {
			maxDistance = distance
		}
	}

	// maximum length of a pass
	numPasses := int(math.Ceil(maxPerpDistance / implement.WorkingWidth))
	//fmt.Printf("numPasses: %v\n", numPasses)

	const intervalDistance = 20.0 // Distance between points in meters

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
		path[len(path)-1].End = true

	}

	//path = ClipPath(path, field.WorkingArea)

	//pathHeadlands, controlPoints, err := GenerateHeadlandPath(path, field, tractor, implement)
	//if err != nil {
	//	return path, nil, err
	//}

	return path, nil, nil
}

func ClipPath(path []Point, boundary []Point) []Point {

	var newPath []Point

	var lastPoint *Point
	var outside bool
	for i, p := range path {

		if IsPointInField(boundary, p) {

			if outside {
				// intersect last point and current point
				_, p2 := IntersectBoundary(boundary, path[i-1], p)
				if p2 != nil {
					newPath = append(newPath, *p2)
				}
			} else {
				newPath = append(newPath, p)
			}
			outside = false
			lastPoint = &p
		} else {
			//fmt.Printf("point %v is outside the field boundary\n", p)
			if lastPoint == nil {
				panic("lastPoint is nil")
			}

			_, p2 := IntersectBoundary(boundary, *lastPoint, p)

			if p2 != nil {

				p2.End = true
				newPath = append(newPath, *p2)

			}
			lastPoint = &p
			outside = true
		}
	}

	// remove duplicate points
	newPath = RemoveDuplicatePoints(newPath)

	return newPath
}

func RemoveDuplicatePoints(path []Point) []Point {
	var newPoints []Point

	newPoints = append(newPoints, path[0])
	for i := 1; i < len(path); i++ {
		x := path[i].X == path[i-1].X
		y := path[i].Y == path[i-1].Y
		if !(x && y) { // not duplicate
			newPoints = append(newPoints, path[i])
			//newPoints[len(newPoints)-1].End = path[i-1].End || path[i].End
		} else {
			//fmt.Printf("duplicate point: %v %v\n", path[i], path[i-1])
			if path[i].End {
				newPoints[len(newPoints)-1].End = true
			}
		}
	}

	return newPoints
}

func GenerateHeadlandPath(path []Point, field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, []Point, error) {

	// clip path
	//path := ClipPath(path, field.WorkingArea)

	clippedPath := path
	//test patj

	//clippedPathLeft := []Point{
	//	{X: -10, Y: 0},
	//	{X: 0, Y: 0, End: true},
	//	{X: 0, Y: 10},
	//	{X: -10, Y: 10, End: true},
	//	{X: -10, Y: 20},
	//	{X: 50, Y: 20, End: true},
	//	{X: 50, Y: 10},
	//}

	headlandPath := []Point{}
	controlPath := []Point{}
	// scan clip path point by point
	headlandPath = append(headlandPath, clippedPath[0])
	for i := 1; i < len(clippedPath); i++ {
		if clippedPath[i-1].End {
			headlandCurve, controlPoints := GenerateHeadlandBezierCurve(clippedPath[i-2], clippedPath[i-1], clippedPath[i], 0.5, implement)
			headlandPath = append(headlandPath, headlandCurve...)
			controlPath = append(controlPath, controlPoints...)

		}
		headlandPath = append(headlandPath, clippedPath[i])
	}

	return headlandPath, controlPath, nil
}

func IntersectBoundary(boundary []Point, insidePoint Point, outsidePoint Point) (bool, *Point) {

	// for each segment in the boundary, check if it intersects with the line segment from insidePoint to outsidePoint
	for i := 0; i < len(boundary)-1; i++ {
		segment1 := [2]Point{boundary[i], boundary[i+1]}
		segment2 := [2]Point{insidePoint, outsidePoint}
		if intersects, p := DoSegmentsIntersect(segment1, segment2); intersects {

			return true, p
		}
	}

	return false, nil
}

// check if two segments intersect, return true if they do and the intersection point
func DoSegmentsIntersect(segment1 [2]Point, segment2 [2]Point) (bool, *Point) {
	// Helper function to determine if point c is left of line from a to b
	isLeftOf := func(a, b, c Point) float64 {
		return (b.X-a.X)*(c.Y-a.Y) - (c.X-a.X)*(b.Y-a.Y)
	}

	// Get points from segments
	p1, p2 := segment1[0], segment1[1]
	p3, p4 := segment2[0], segment2[1]

	// Check if the segments intersect using orientation tests
	d1 := isLeftOf(p3, p4, p1)
	d2 := isLeftOf(p3, p4, p2)
	d3 := isLeftOf(p1, p2, p3)
	d4 := isLeftOf(p1, p2, p4)

	// check if the segments are parallel
	if d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0 {
		return false, nil
	}

	// If the orientations are different (one positive, one negative),
	// then the segments intersect
	if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
		((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
		intersect := intersect(segment1, segment2)
		return true, intersect
	}

	// Check if any endpoint lies exactly on the other segment
	if d1 == 0 && IsPointOnLine(p3, p4, p1) {
		intersect := intersect(segment1, segment2)
		return true, intersect
	}
	if d2 == 0 && IsPointOnLine(p3, p4, p2) {
		intersect := intersect(segment1, segment2)
		return true, intersect
	}
	if d3 == 0 && IsPointOnLine(p1, p2, p3) {
		intersect := intersect(segment1, segment2)
		return true, intersect
	}
	if d4 == 0 && IsPointOnLine(p1, p2, p4) {
		intersect := intersect(segment1, segment2)
		return true, intersect
	}

	return false, nil
}

// SpiralPlanner is a planner that generates a spiral path, it implements PathPlanner interface
type SpiralPlanner struct {
	Path []Point

	Centre Point

	MaxLaps int
}

func NewSpiralPlanner() *SpiralPlanner {
	return &SpiralPlanner{
		MaxLaps: 0,
	}
}

// pathplanner GeneratePath method as per interface
func (s *SpiralPlanner) GeneratePath(field *Field, tractor *tractor.Tractor, implement *implement.Implement) ([]Point, []Point, error) {

	// generate spiral path

	// find the centre of the field

	//
	if s.MaxLaps < 0 {
		panic("max laps must be greater or equal to 0")
	}

	maxlaps := s.MaxLaps

	if s.MaxLaps == 0 {
		maxlaps = math.MaxInt
	}

	firstlap := GenerateInnerBoundary(field.Boundary, implement.WorkingWidth/2)
	if firstlap == nil {
		return nil, nil, errors.New("invalid boundary or offset")
	}

	innerlap := firstlap

	fullpath := []Point{}
	fullpath = append(fullpath, firstlap...)

	// add the first point to close the path
	fullpath = append(fullpath, fullpath[0])
	//fullpath = Subdivide(fullpath, implement.WorkingWidth/2)
	// trim last point
	//fullpath = fullpath[:len(fullpath)-1]

	// trim the last two points of pull path to make it a loop
	trimmed := TrimLine(fullpath[len(fullpath)-2:], implement.WorkingWidth)
	fullpath = fullpath[:len(fullpath)-2]
	fullpath = append(fullpath, trimmed...)

	// for each lap, generate an inner boundary to form the lap, using the working width, use this recursively to form the spiral, offset by half the working width
	for i := 1; i < maxlaps; i++ {
		innerlap = GenerateInnerBoundary(innerlap, implement.WorkingWidth)
		if innerlap == nil {
			break
		}

		innerlapclosed := append(innerlap, innerlap[0])

		innerlapclosed = TrimPath(innerlapclosed, implement.WorkingWidth)

		fullpath = append(fullpath, innerlapclosed...)

		if MinimumDistanceToCentre(innerlap) < implement.WorkingWidth {
			break
		}
	}

	controlpath := RoundCorners(fullpath, implement.WorkingWidth)
	fullpath = Subdivide(controlpath, implement.Length)

	fullpath = ForwardOffsetPath(fullpath, implement.Length)

	return fullpath, controlpath, nil
}

// subdivde a path givin as points into more points at a fixed interval
func Subdivide(path []Point, interval float64) []Point {

	// for each point in the path, calculate the distance to the next point
	// if the distance is greater than the interval, add a new point at the interval distance
	// return the new path

	newPath := []Point{}

	for i := 0; i < len(path)-1; i++ {
		distance := math.Sqrt(math.Pow(path[i+1].X-path[i].X, 2) + math.Pow(path[i+1].Y-path[i].Y, 2))
		if distance > interval {

			// calculate the number of points to add
			numPoints := int(math.Ceil(distance / interval))
			for j := 0; j < numPoints; j++ {
				t := float64(j) / float64(numPoints)
				newPath = append(newPath, Point{
					X: path[i].X + (path[i+1].X-path[i].X)*t,
					Y: path[i].Y + (path[i+1].Y-path[i].Y)*t,
				})
			}
		} else {
			newPath = append(newPath, path[i])
		}
	}

	return newPath
}

func MinimumDistanceToCentre(boundary []Point) float64 {

	centre := FindCentreOfField(boundary)

	minDistance := math.Inf(1)

	for _, p := range boundary {

		distance := p.Distance(centre)
		if distance < minDistance {
			minDistance = distance
		}
	}

	return minDistance
}

// RoundCorners smooths the corners of a path by replacing sharp corners with arcs of the specified radius
func RoundCorners(path []Point, radius float64) []Point {
	if len(path) < 3 || radius <= 0 {
		return path
	}

	result := []Point{}

	// Add the first point
	result = append(result, path[0])

	// Process each corner (middle points)
	for i := 1; i < len(path)-1; i++ {
		prev := path[i-1]
		current := path[i]
		next := path[i+1]

		// Calculate vectors for the two segments
		v1 := Vector{X: current.X - prev.X, Y: current.Y - prev.Y}
		v2 := Vector{X: next.X - current.X, Y: next.Y - current.Y}

		// Normalize vectors
		v1Len := math.Sqrt(v1.X*v1.X + v1.Y*v1.Y)
		v2Len := math.Sqrt(v2.X*v2.X + v2.Y*v2.Y)

		// invert the vectors
		//v1 = v1.Reverse()
		//v2 = v2.Reverse()

		if v1Len == 0 || v2Len == 0 {
			result = append(result, current)
			continue
		}

		v1.X /= v1Len
		v1.Y /= v1Len
		v2.X /= v2Len
		v2.Y /= v2Len

		// Calculate the angle between the segments
		cosAngle := v1.X*v2.X + v1.Y*v2.Y
		angle := math.Acos(math.Max(-1, math.Min(1, cosAngle)))

		// If the angle is too small, just keep the corner point
		if math.Abs(angle) < 0.1 || math.Abs(angle-math.Pi) < 0.1 {
			result = append(result, current)
			continue
		}

		// Calculate the tangent distance
		tanDist := radius * math.Tan(angle/2)

		// Ensure tangent distance isn't too large for the segments
		//tanDist = math.Min(tanDist, v1Len/2)
		//tanDist = math.Min(tanDist, v2Len/2)

		// Calculate tangent points
		t1 := Point{
			X: current.X - v1.X*tanDist,
			Y: current.Y - v1.Y*tanDist,
		}

		t2 := Point{
			X: current.X + v2.X*tanDist,
			Y: current.Y + v2.Y*tanDist,
		}

		v1 = v1.Reverse()

		// Calculate the center of the arc
		bisector := Vector{
			X: v1.X + v2.X,
			Y: v1.Y + v2.Y,
		}
		bisectorLen := math.Sqrt(bisector.X*bisector.X + bisector.Y*bisector.Y)

		if bisectorLen < 0.001 {
			result = append(result, current)
			continue
		}

		bisector.X /= bisectorLen
		bisector.Y /= bisectorLen

		// Direction from corner to center
		direction := Direction(prev, current, next)

		// Distance from corner to center
		cornerToCenterDist := radius / math.Sin(angle/2)

		center := Point{
			X: current.X + bisector.X*cornerToCenterDist,
			Y: current.Y + bisector.Y*cornerToCenterDist,
		}

		// Calculate start and end angles
		startAngle := math.Atan2(t1.Y-center.Y, t1.X-center.X)
		endAngle := math.Atan2(t2.Y-center.Y, t2.X-center.X)

		//startangleDeg := startAngle * 180 / math.Pi
		//endangleDeg := endAngle * 180 / math.Pi

		//fmt.Println("startAngle", startangleDeg, "endAngle", endangleDeg)

		// Ensure we go the short way around
		if math.Abs(endAngle-startAngle) > math.Pi {
			//fmt.Println("angle", endAngle-startAngle)
			endAngle = endAngle - math.Pi*2
		}

		// Add the first tangent point
		result = append(result, t1)

		// Add arc points
		numArcPoints := int(math.Max(3, math.Abs(endAngle-startAngle)*radius))
		for j := 1; j < numArcPoints; j++ {
			t := float64(j) / float64(numArcPoints)
			angle := startAngle + t*(math.Abs(endAngle-startAngle)*float64(direction))
			arcPoint := Point{
				X: center.X + radius*math.Cos(angle),
				Y: center.Y + radius*math.Sin(angle),
			}
			result = append(result, arcPoint)
		}

		// Add the second tangent point
		result = append(result, t2)
	}

	// Add the last point
	result = append(result, path[len(path)-1])

	return result
}

// trim vector by length
func TrimLine(line []Point, trimlength float64) []Point {

	v1 := Vector{X: line[1].X - line[0].X, Y: line[1].Y - line[0].Y}

	if v1.Length() < trimlength {
		return line
	}

	v1 = v1.SetLength(v1.Length() - trimlength)

	v2 := PointToVector(line[0])
	v2 = v2.Add(v1)

	return []Point{
		line[0],
		v2.ToPoint(),
	}
}

func TrimPath(path []Point, trimlength float64) []Point {

	if len(path) < 2 {
		return path
	}

	trimmed := TrimLine(path[len(path)-2:], trimlength)

	return append(path[:len(path)-2], trimmed...)
}

// generate a forward offset path whereby the path is offset by a fixed distance in the direction of travel
func ForwardOffsetPath(path []Point, offset float64) []Point {

	offsetPath := []Point{}

	for i := 1; i < len(path)-1; i++ {

		// get a vector from the current point to the next point
		v1 := PointToVector(path[i-1])
		v2 := PointToVector(path[i])

		v2 = v2.Sub(v1)

		v2 = v2.SetLength(offset)

		np := PointToVector(path[i]).Add(v2)

		// if values are NaN, skip this point
		if math.IsNaN(np.X) || math.IsNaN(np.Y) {
			continue
		}

		// add the offset vector to the current point
		offsetPath = append(offsetPath, np.ToPoint())
	}

	return offsetPath
}
