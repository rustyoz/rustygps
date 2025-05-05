package planner

import (
	"math"
	"testing"
)

func TestVecOperations(t *testing.T) {
	// Test vector length
	v := Vector{X: 3, Y: 4}
	length := vecLength(v)
	if length != 5 {
		t.Errorf("vecLength({3, 4}) = %f; want 5", length)
	}

	// Test unit vector
	unit := vecUnit(v)
	if math.Abs(unit.X-0.6) > 0.0001 || math.Abs(unit.Y-0.8) > 0.0001 {
		t.Errorf("vecUnit({3, 4}) = {%f, %f}; want {0.6, 0.8}", unit.X, unit.Y)
	}

	// Test zero vector
	zeroUnit := vecUnit(Vector{X: 0, Y: 0})
	if zeroUnit.X != 0 || zeroUnit.Y != 0 {
		t.Errorf("vecUnit({0, 0}) = {%f, %f}; want {0, 0}", zeroUnit.X, zeroUnit.Y)
	}

	// Test vector multiplication
	scaled := vecMul(v, 2)
	if scaled.X != 6 || scaled.Y != 8 {
		t.Errorf("vecMul({3, 4}, 2) = {%f, %f}; want {6, 8}", scaled.X, scaled.Y)
	}

	// Test dot product
	v1 := Vector{X: 2, Y: 3}
	v2 := Vector{X: 4, Y: -1}
	dot := vecDot(v1, v2)
	if dot != 5 { // (2*4 + 3*-1)
		t.Errorf("vecDot({2, 3}, {4, -1}) = %f; want 5", dot)
	}

	// Test 90-degree rotations
	rotCW := vecRot90CW(v)
	if rotCW.X != 4 || rotCW.Y != -3 {
		t.Errorf("vecRot90CW({3, 4}) = {%f, %f}; want {4, -3}", rotCW.X, rotCW.Y)
	}

	rotCCW := vecRot90CCW(v)
	if rotCCW.X != -4 || rotCCW.Y != 3 {
		t.Errorf("vecRot90CCW({3, 4}) = {%f, %f}; want {-4, 3}", rotCCW.X, rotCCW.Y)
	}
}

func TestIntersect(t *testing.T) {
	// Test intersection of two perpendicular lines
	line1 := [2]Point{{X: 0, Y: 5}, {X: 10, Y: 5}}
	line2 := [2]Point{{X: 5, Y: 0}, {X: 5, Y: 10}}
	intersection := intersect(line1, line2)
	if intersection.X != 5 || intersection.Y != 5 {
		t.Errorf("intersect() = {%f, %f}; want {5, 0}", intersection.X, intersection.Y)
	}

	// Test parallel lines (should return midpoint of first line as fallback)
	line3 := [2]Point{{X: 0, Y: 0}, {X: 10, Y: 0}}
	line4 := [2]Point{{X: 0, Y: 5}, {X: 10, Y: 5}}
	parallelIntersection := intersect(line3, line4)
	if parallelIntersection != nil {
		t.Errorf("intersect() with parallel lines = {%f, %f}; want nil",
			parallelIntersection.X, parallelIntersection.Y)
	}
}

func TestPolyIsCw(t *testing.T) {
	// Test clockwise polygon
	clockwisePoly := []Point{
		{X: 0, Y: 0},
		{X: 10, Y: 0},
		{X: 10, Y: -10},
		{X: 0, Y: -10},
	}
	if !polyIsCw(clockwisePoly) {
		t.Error("polyIsCw() with clockwise polygon = false; want true")
	}

	// Test counterclockwise polygon
	counterclockwisePoly := []Point{
		{X: 0, Y: 0},
		{X: 10, Y: 0},
		{X: 10, Y: 10},
		{X: 0, Y: 10},
	}
	if polyIsCw(counterclockwisePoly) {
		t.Error("polyIsCw() with counterclockwise polygon = true; want false")
	}
}

func TestGenerateInnerBoundary(t *testing.T) {
	// Test square boundary with offset
	square := []Point{
		{X: 0, Y: 0},
		{X: 100, Y: 0},
		{X: 100, Y: 100},
		{X: 0, Y: 100},
	}
	offset := 10.0
	inner := GenerateInnerBoundary(square, offset)

	// Check we have the right number of points
	if len(inner) != len(square) {
		t.Errorf("GenerateInnerBoundary() returned %d points; want %d", len(inner), len(square))
	}

	// For a square, the inner points should be offset by approximately 'offset' in both X and Y
	// We don't check exact coordinates because of the intersection calculation,
	// but we can check if they're approximately where expected
	expectedInner := []Point{
		{X: offset, Y: offset},
		{X: 100 - offset, Y: offset},
		{X: 100 - offset, Y: 100 - offset},
		{X: offset, Y: 100 - offset},
	}

	for i, p := range inner {
		expected := expectedInner[i]
		if math.Abs(p.X-expected.X) > 1.0 || math.Abs(p.Y-expected.Y) > 1.0 {
			t.Errorf("Inner point %d = {%f, %f}; approximately want {%f, %f}",
				i, p.X, p.Y, expected.X, expected.Y)
		}
	}

	// Test with invalid inputs
	if inner = GenerateInnerBoundary(nil, offset); inner != nil {
		t.Error("GenerateInnerBoundary(nil, offset) = non-nil; want nil")
	}

	if inner = GenerateInnerBoundary(square, -1); inner != nil {
		t.Error("GenerateInnerBoundary(square, -1) = non-nil; want nil")
	}

	// Test with complex shape to ensure it doesn't panic
	complex := []Point{
		{X: 0, Y: 0},
		{X: 100, Y: 0},
		{X: 120, Y: 50},
		{X: 100, Y: 100},
		{X: 50, Y: 120},
		{X: 0, Y: 100},
	}
	inner = GenerateInnerBoundary(complex, offset)
	if len(inner) != len(complex) {
		t.Errorf("GenerateInnerBoundary() with complex shape returned %d points; want %d",
			len(inner), len(complex))
	}
}

func TestDirection(t *testing.T) {

	a := Point{X: 0, Y: 0}
	b := Point{X: 1, Y: 0}
	c := Point{X: 1, Y: 1}
	direction := Direction(a, b, c)
	if direction != 1 {
		t.Errorf("Left turn Direction(%v, %v, %v) = %d; want 1", a, b, c, direction)
	}

	a = Point{X: 0, Y: 1}
	b = Point{X: 1, Y: 1}
	c = Point{X: 1, Y: 0}
	direction = Direction(a, b, c)
	if direction != -1 {
		t.Errorf("Right turn Direction(%v, %v, %v) = %d; want -1", a, b, c, direction)
	}

	a = Point{X: 0, Y: 0}
	b = Point{X: 1, Y: 0}
	c = Point{X: 2, Y: 0}
	direction = Direction(a, b, c)
	if direction != 0 {
		t.Errorf("Straight Direction(%v, %v, %v) = %d; want 0", a, b, c, direction)
	}
}
