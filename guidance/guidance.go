package guidance

import (
	"fmt"
	"math"

	"github.com/rustyoz/rustygps/planner"
	"github.com/rustyoz/rustygps/types"
)

// this file will define the guidance system for the tractor
// it will take in the tractor's position and the desired path
// and return the steering angle and throttle

type Guidance struct {
	TargetPoint         planner.Point
	TargetSteeringAngle float64
	TargetSpeed         float64
	TargetThrottle      float64
	GuidanceType        string
	GuidanceConfig      GuidanceConfig
	TractorState        TractorState
	Path                *[]planner.Point
}

// GuidanceConfig holds configuration parameters for the guidance system
type GuidanceConfig struct {
	MaxSteeringAngle float64 // Maximum steering angle in radians
	LookAheadDist    float64 // Look-ahead distance in meters
	PlanningDistance float64 // Maximum distance ahead to look for target points
	Deadzone         float64 // Deadzone in radians
}

// TractorState represents the current state of the tractor
type TractorState struct {
	Position      types.Position
	WorldPos      types.WorldPosition
	SteeringAngle float64 // Steering angle in radians
	Throttle      float64 // Throttle in percent
}

func NewGuidance(path *[]planner.Point) *Guidance {
	return &Guidance{
		Path: path,
		GuidanceConfig: GuidanceConfig{
			MaxSteeringAngle: math.Pi / 4,     // 45 degrees
			LookAheadDist:    1.0,             // 1 meter
			PlanningDistance: 10.0,            // 10 meters
			Deadzone:         math.Pi / 180.0, // 1 degree
		},
		TargetPoint: planner.Point{
			X: 0,
			Y: 0,
		},
		TractorState: TractorState{
			Position: types.Position{
				Lat: 0,
				Lon: 0,
			},
			WorldPos: types.WorldPosition{
				X: 0,
				Y: 0,
			},
			SteeringAngle: 0,
			Throttle:      0,
		},
	}
}

func (g *Guidance) SetTractorState(state TractorState) {
	g.TractorState = state
}

func (g *Guidance) Update() {
	g.TargetPoint = g.GetTargetPoint()
	g.TargetSteeringAngle = g.CalculateSteeringAngle(g.TargetPoint)
}

// CalculateSteeringAngle calculates the desired steering angle to follow the path
func (g *Guidance) CalculateSteeringAngle(targetPoint planner.Point) float64 {
	if len(*g.Path) < 2 {
		return 0
	}

	return g.calculatePurePursuitAngle(g.TractorState, targetPoint, g.GuidanceConfig.MaxSteeringAngle, g.GuidanceConfig.Deadzone)
}

// findLookAheadPoint finds a point on the path that is approximately lookAheadDist away
func findLookAheadPoint(state TractorState, path []planner.Point, lookAheadDist float64) planner.Point {
	closestIdx := 0
	minDist := math.Inf(1)

	// Find the closest point on the path
	for i, point := range path {
		dist := distance(state.WorldPos.X, state.WorldPos.Y, point.X, point.Y)
		if dist < minDist {
			minDist = dist
			closestIdx = i
		}
	}

	// Look ahead from the closest point
	for i := closestIdx; i < len(path); i++ {
		dist := distance(state.WorldPos.X, state.WorldPos.Y, path[i].X, path[i].Y)
		if dist >= lookAheadDist {
			return path[i]
		}
	}

	// If we can't find a point far enough ahead, use the last point
	return path[len(path)-1]
}

// follow the path and return the next point to follow based on whether it has visited the point.
// once the tractor has been within 1m distance of the point, mark it as visited and return the next point
var lastVisitedPointIndex int = 0

func (g *Guidance) GetTargetPoint() planner.Point {
	// start at the last visited point index
	for i := lastVisitedPointIndex; i < len(*g.Path); i++ {
		dist := distance(g.TractorState.WorldPos.X, g.TractorState.WorldPos.Y, (*g.Path)[i].X, (*g.Path)[i].Y)

		// Mark points as visited if we're close enough
		if dist < g.GuidanceConfig.PlanningDistance {
			(*g.Path)[i].Visited = true
			lastVisitedPointIndex = i
		}

		// Return first unvisited point
		if !(*g.Path)[i].Visited {
			return (*g.Path)[i]
		}
	}
	return g.TargetPoint
}

// calculatePurePursuitAngle calculates the steering angle using pure pursuit algorithm
func (g *Guidance) calculatePurePursuitAngle(state TractorState, target planner.Point, maxSteeringAngle float64, deadzone float64) float64 {
	// Calculate relative position of target in tractor's coordinate frame
	dx := target.X - state.WorldPos.X
	dy := target.Y - state.WorldPos.Y

	//fmt.Println("dx:", dx, "dy:", dy)

	// Convert to tractor's local coordinate frame
	localX := dx*math.Cos(-state.WorldPos.Heading) - dy*math.Sin(-state.WorldPos.Heading)
	localY := dx*math.Sin(-state.WorldPos.Heading) + dy*math.Cos(-state.WorldPos.Heading)

	// normalize localX and localY to the lookahead distance
	length := math.Sqrt(localX*localX + localY*localY)
	localX = localX / length * g.GuidanceConfig.LookAheadDist
	localY = localY / length * g.GuidanceConfig.LookAheadDist

	// calculate heading in local frame
	localHeading := math.Atan2(localY, localX)

	//fmt.Println("localX:", localX, "localY:", localY, "localHeading:", localHeading)
	// Calculate curvature (inverse of turning radius)
	curvature := (2 * localY) / (localX*localX + localY*localY)
	//fmt.Println("curvature:", curvature)

	//	fmt.Println("maxSteeringAngle:", maxSteeringAngle)
	// Convert curvature to steering angle
	steeringAngle := math.Atan(curvature)

	// Apply deadzone
	if math.Abs(steeringAngle) < deadzone {
		return 0
	}

	// if the target point is behind the tractor, return the max steering angle
	if localX < 0 {
		if localY > 0 {
			steeringAngle = maxSteeringAngle
		} else {
			steeringAngle = -maxSteeringAngle
		}
	}

	// if the heading is larger than the max steering angle, return the max steering angle
	if localHeading > maxSteeringAngle {
		steeringAngle = maxSteeringAngle
	} else if localHeading < -maxSteeringAngle {
		steeringAngle = -maxSteeringAngle
	}

	// Apply steering limits
	if steeringAngle > maxSteeringAngle {
		steeringAngle = maxSteeringAngle
	} else if steeringAngle < -maxSteeringAngle {
		steeringAngle = -maxSteeringAngle
	}

	// apply deadzone
	if math.Abs(steeringAngle) < deadzone {
		return 0
	}

	//fmt.Println("Steering angle:", steeringAngle)
	return steeringAngle
}

// distance calculates the Euclidean distance between two points
func distance(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Sqrt(dx*dx + dy*dy)
}

// Add method to update guidance parameters
func (g *Guidance) UpdateParameters(lookAheadDist, deadzone, planningDistance float64) {
	g.GuidanceConfig.LookAheadDist = lookAheadDist
	g.GuidanceConfig.Deadzone = deadzone
	g.GuidanceConfig.PlanningDistance = planningDistance
	fmt.Println("Updated guidance parameters: lookAheadDist:", lookAheadDist, "deadzone:", deadzone, "planningDistance:", planningDistance)
}

func (g *Guidance) UpdateConfig(config GuidanceConfig) {
	g.GuidanceConfig.MaxSteeringAngle = config.MaxSteeringAngle
	g.GuidanceConfig.LookAheadDist = config.LookAheadDist
	g.GuidanceConfig.PlanningDistance = config.PlanningDistance
	g.GuidanceConfig.Deadzone = config.Deadzone
}

func (g *Guidance) Reset() {

	lastVisitedPointIndex = 0
	g.TargetPoint = (*g.Path)[0]
}
