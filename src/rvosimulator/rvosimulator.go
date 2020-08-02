package rvosimulator

var (
	Sim *RVOSimulator
)

func init() {
}

// RVOSimulator :
type RVOSimulator struct {
	TimeStep         float64
	Agents           map[int]*Agent
	Obstacles        [][]*Vector2
	ObstacleVertices []*Obstacle
	KdTree           *KdTree
	DefaultAgent     *Agent
	GlobalTime       float64
	NextAgentID      int
}

// NewRVOSimulator : RVOSimulator with options
func NewRVOSimulator(timeStep float64, neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) *RVOSimulator {
	kdTree := NewKdTree()
	defaultAgent := NewEmptyAgent()

	defaultAgent.MaxNeighbors = maxNeighbors
	defaultAgent.MaxSpeed = maxSpeed
	defaultAgent.NeighborDist = neighborDist
	defaultAgent.Radius = radius
	defaultAgent.TimeHorizon = timeHorizon
	defaultAgent.TimeHorizonObst = timeHorizonObst
	defaultAgent.Velocity = velocity

	sim := &RVOSimulator{
		TimeStep:         timeStep,
		Agents:           make(map[int]*Agent),
		Obstacles:        make([][]*Vector2, 0),
		ObstacleVertices: make([]*Obstacle, 0),
		KdTree:           kdTree,
		DefaultAgent:     defaultAgent,
		GlobalTime:       0.0,
		NextAgentID:      0,
	}
	Sim = sim

	return sim
}

// NewDefaultRVOSimulator : RVOSimulator with default values
func NewEmptyRVOSimulator() *RVOSimulator {
	kdTree := NewKdTree()
	defaultAgent := NewEmptyAgent()

	sim := &RVOSimulator{
		TimeStep:         0,
		Agents:           make(map[int]*Agent),
		Obstacles:        make([][]*Vector2, 0),
		ObstacleVertices: make([]*Obstacle, 0),
		KdTree:           kdTree,
		DefaultAgent:     defaultAgent,
		GlobalTime:       0.0,
	}
	Sim = sim
	return sim
}

type AddAgentParam struct {
	Position        *Vector2
	NeighborDist    float64
	MaxNeighbors    int
	TimeHorizon     float64
	TimeHorizonObst float64
	Radius          float64
	MaxSpeed        float64
	Velocity        *Vector2
}

// AddDefaultAgent : Add agent with default values
func (rvo *RVOSimulator) AddDefaultAgent(position *Vector2) (int, bool) {

	if rvo.DefaultAgent == nil {
		err := true
		return -1, err
	}

	agent := NewEmptyAgent()
	agent.Position = position
	agent.MaxNeighbors = rvo.DefaultAgent.MaxNeighbors
	agent.MaxSpeed = rvo.DefaultAgent.MaxSpeed
	agent.NeighborDist = rvo.DefaultAgent.NeighborDist
	agent.Radius = rvo.DefaultAgent.Radius
	agent.TimeHorizon = rvo.DefaultAgent.TimeHorizon
	agent.TimeHorizonObst = rvo.DefaultAgent.TimeHorizonObst
	agent.Velocity = rvo.DefaultAgent.Velocity
	agent.ID = rvo.NextAgentID

	(rvo.Agents)[rvo.NextAgentID] = agent
	rvo.NextAgentID++

	return agent.ID, false
}

// AddAgent : Add agent with options
func (rvo *RVOSimulator) AddAgent(position *Vector2, neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) (int, bool) {

	agent := NewEmptyAgent()
	agent.Position = position
	agent.MaxNeighbors = maxNeighbors
	agent.MaxSpeed = maxSpeed
	agent.NeighborDist = neighborDist
	agent.Radius = radius
	agent.TimeHorizon = timeHorizon
	agent.TimeHorizonObst = timeHorizonObst
	agent.Velocity = velocity
	agent.ID = rvo.NextAgentID

	rvo.Agents[rvo.NextAgentID] = agent
	rvo.NextAgentID++

	return agent.ID, false
}

// RemoveAgent : Remove agent by agentID
func (rvo *RVOSimulator) RemoveAgent(agentID int) bool {

	rvo.Agents[agentID].Status = 3 // mark as to be deleted
	// delete(rvo.Agents, agentID)
	// rvo.Agents = append(rvo.Agents[:agentID], rvo.Agents[agentID+1:]...)

	return false
}

// GetAgentNoByID : Get Agent No. by Agent ID
// func (rvo *RVOSimulator) GetAgentNoByID(id int) int {
// 	for i := 0; i < rvo.GetNumAgents(); i++ {
// 		if rvo.GetAgent(i).ID == id {
// 			return i
// 		}
// 	}
// 	return -1
// }

// AddObstacle : Add Obstacle with vertices
func (rvo *RVOSimulator) AddObstacle(vertices []*Vector2) (int, bool) {

	// add obstacle
	rvo.Obstacles = append(rvo.Obstacles, vertices)

	// add obstacle vertices
	if len(vertices) < 2 {
		err := true
		return -1, err
	}

	// 一つ一つ大きなObstacleはObstacleNoとして管理
	obstacleNo := len(rvo.ObstacleVertices)

	// Obstacleを一点ずつ置いて行って形を作る
	for i := 0; i < len(vertices); i++ {
		obstacle := NewEmptyObstacle()
		obstacle.Point = vertices[i]

		// NextとPrevObstacleをセット
		if i != 0 {
			obstacle.PrevObstacle = rvo.ObstacleVertices[len(rvo.ObstacleVertices)-1]
			obstacle.PrevObstacle.NextObstacle = obstacle
		}

		if i == len(vertices)-1 {
			obstacle.NextObstacle = rvo.ObstacleVertices[obstacleNo]
			obstacle.NextObstacle.PrevObstacle = obstacle
		}

		var ti int
		if i == len(vertices)-1 {
			ti = 0
		} else {
			ti = i + 1
		}

		obstacle.UnitDir = Normalize(Sub(vertices[ti], vertices[i]))

		var ki int
		if i == 0 {
			ki = len(vertices) - 1
		} else {
			ki = i - 1
		}

		// 凸かどうか　？
		if len(vertices) == 2 {
			obstacle.IsConvex = true
		} else {
			obstacle.IsConvex = (LeftOf(vertices[ki], vertices[i], vertices[ti]) >= 0.0)
		}

		obstacle.ID = len(rvo.ObstacleVertices)

		rvo.ObstacleVertices = append(rvo.ObstacleVertices, obstacle)

	}

	return obstacleNo, false
}

// DoStep : Forward Step
func (rvo *RVOSimulator) DoStep() {
	rvo.KdTree.BuildAgentTree()

	for _, v := range rvo.Agents {
		// agentのneighborsを計算
		v.ComputeNeighbors()
		// agentの速度を計算
		v.ComputeNewVelocity()
	}

	for _, v := range rvo.Agents {
		// agentを更新
		v.Update()
	}

	// globaltimeを更新
	rvo.GlobalTime += rvo.TimeStep
}

// IsReachedGoal :
func (rvo *RVOSimulator) IsReachedGoal() bool {
	/* Check if all agents have reached their goals. */
	for i := range rvo.Agents {
		if !rvo.IsAgentReachedGoal(i) {
			return false
		}
	}
	return true
}

// IsAgentReachedGoal :
func (rvo *RVOSimulator) IsAgentReachedGoal(agentID int) bool {
	/* Check if agent have reached their goals. */
	if Sqr(Sub(rvo.GetAgentGoal(agentID), rvo.GetAgentPosition(agentID))) > rvo.GetAgentRadius(agentID)*rvo.GetAgentRadius(agentID) {
		return false
	}
	return true
}

// GetAgentGoalVector :
func (rvo *RVOSimulator) GetAgentGoalVector(agentID int) *Vector2 {
	return Normalize(Sub(rvo.GetAgentGoal(agentID), rvo.GetAgentPosition(agentID)))
}

// GetAgents :
func (rvo *RVOSimulator) GetAgents() map[int]*Agent {
	return rvo.Agents
}

// GetAgent :
func (rvo *RVOSimulator) GetAgent(agentID int) *Agent {
	return rvo.Agents[agentID]
}

// GetAgentAgentNeighbor :
func (rvo *RVOSimulator) GetAgentAgentNeighbor(agentID int, neighborNo int) int {
	return rvo.Agents[agentID].AgentNeighbors[neighborNo].Agent.ID
}

// GetAgentMaxNeighbors :
func (rvo *RVOSimulator) GetAgentMaxNeighbors(agentID int) int {
	agent := rvo.Agents[agentID]
	return agent.MaxNeighbors
}

// GetAgentMaxSpeed :
func (rvo *RVOSimulator) GetAgentMaxSpeed(agentID int) float64 {
	agent := rvo.Agents[agentID]
	return agent.MaxSpeed
}

// GetAgentNeighborDist :
func (rvo *RVOSimulator) GetAgentNeighborDist(agentID int) float64 {
	agent := rvo.Agents[agentID]
	return agent.NeighborDist
}

// GetAgentNumAgentNeighbors :
func (rvo *RVOSimulator) GetAgentNumAgentNeighbors(agentID int) int {
	agent := rvo.Agents[agentID]
	return len(agent.AgentNeighbors)
}

// GetAgentNumObstacleNeighbors :
func (rvo *RVOSimulator) GetAgentNumObstacleNeighbors(agentID int) int {
	agent := rvo.Agents[agentID]
	return len(agent.ObstacleNeighbors)
}

// GetAgentNumORCALines :
func (rvo *RVOSimulator) GetAgentNumORCALines(agentID int) int {
	agent := rvo.Agents[agentID]
	return len(agent.OrcaLines)
}

// GetAgentObstacleNeighbor :
func (rvo *RVOSimulator) GetAgentObstacleNeighbor(agentID int, neighborNo int) int {
	agent := rvo.Agents[agentID]
	obstacleNeighbor := agent.ObstacleNeighbors[neighborNo]
	return obstacleNeighbor.Obstacle.ID
}

// GetAgentORCALine :
func (rvo *RVOSimulator) GetAgentORCALine(agentID int, lineNo int) *Line {
	agent := rvo.Agents[agentID]
	return agent.OrcaLines[lineNo]
}

// GetAgentPosition :
func (rvo *RVOSimulator) GetAgentPosition(agentID int) *Vector2 {
	agent := rvo.Agents[agentID]
	return agent.Position
}

// GetAgentGoal :
func (rvo *RVOSimulator) GetAgentGoal(agentID int) *Vector2 {
	agent := rvo.Agents[agentID]
	return agent.Goal
}

// GetAgentPrefVelocity :
func (rvo *RVOSimulator) GetAgentPrefVelocity(agentID int) *Vector2 {
	agent := rvo.Agents[agentID]
	return agent.PrefVelocity
}

// GetAgentRadius :
func (rvo *RVOSimulator) GetAgentRadius(agentID int) float64 {
	agent := rvo.Agents[agentID]
	return agent.Radius
}

// GetAgentTimeHorizon :
func (rvo *RVOSimulator) GetAgentTimeHorizon(agentID int) float64 {
	agent := rvo.Agents[agentID]
	return agent.TimeHorizon
}

// GetAgentTimeHorizonObst :
func (rvo *RVOSimulator) GetAgentTimeHorizonObst(agentID int) float64 {
	agent := rvo.Agents[agentID]
	return agent.TimeHorizonObst
}

// GetAgentVelocity :
func (rvo *RVOSimulator) GetAgentVelocity(agentID int) *Vector2 {
	agent := rvo.Agents[agentID]
	return agent.Velocity
}

// GetAgentStatus :
func (rvo *RVOSimulator) GetAgentStatus(agentID int) int {
	agent := rvo.Agents[agentID]
	return agent.Status
}

// GetGlobalTime :
func (rvo *RVOSimulator) GetGlobalTime() float64 {
	return rvo.GlobalTime
}

// GetNumAgents :
func (rvo *RVOSimulator) GetNumAgents() int {
	return len(rvo.Agents)
}

// GetNumObstacleVertices :
func (rvo *RVOSimulator) GetNumObstacleVertices() int {
	return len(rvo.ObstacleVertices)
}

// GetObstacleVertex :
func (rvo *RVOSimulator) GetObstacleVertex(vertexNo int) *Vector2 {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.Point
}

// GetObstacles :
func (rvo *RVOSimulator) GetObstacles() [][]*Vector2 {
	return rvo.Obstacles
}

// GetNumObstacles :
func (rvo *RVOSimulator) GetNumObstacles() int {
	return len(rvo.Obstacles)
}

// GetObstacle :
func (rvo *RVOSimulator) GetObstacle(obstacleNo int) []*Vector2 {
	return rvo.Obstacles[obstacleNo]
}

// GetNextObstacleVertexNo :
func (rvo *RVOSimulator) GetNextObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.NextObstacle.ID
}

// GetPrevObstacleVertexNo :
func (rvo *RVOSimulator) GetPrevObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.PrevObstacle.ID
}

// GetTimeStep :
func (rvo *RVOSimulator) GetTimeStep() float64 {
	return rvo.TimeStep
}

// ProcessObstacles :
func (rvo *RVOSimulator) ProcessObstacles() {
	rvo.KdTree.BuildObstacleTree()
}

// QueryVisibility :
func (rvo *RVOSimulator) QueryVisibility(point1 *Vector2, point2 *Vector2, radius float64) bool {
	return rvo.KdTree.QueryVisibility(point1, point2, radius)
}

// SetAgentDefaults :
func (rvo *RVOSimulator) SetAgentDefaults(neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) {
	if rvo.DefaultAgent == nil {
		rvo.DefaultAgent = NewEmptyAgent()
	}

	rvo.DefaultAgent.MaxNeighbors = maxNeighbors
	rvo.DefaultAgent.MaxSpeed = maxSpeed
	rvo.DefaultAgent.NeighborDist = neighborDist
	rvo.DefaultAgent.Radius = radius
	rvo.DefaultAgent.TimeHorizon = timeHorizon
	rvo.DefaultAgent.TimeHorizonObst = timeHorizonObst
	rvo.DefaultAgent.Velocity = velocity

}

// SetAgentMaxNeighbors :
func (rvo *RVOSimulator) SetAgentMaxNeighbors(agentID int, maxNeighbors int) {
	rvo.Agents[agentID].MaxNeighbors = maxNeighbors
}

// SetAgentMaxSpeed :
func (rvo *RVOSimulator) SetAgentMaxSpeed(agentID int, maxSpeed float64) {
	rvo.Agents[agentID].MaxSpeed = maxSpeed
}

// SetAgentNeighborDist :
func (rvo *RVOSimulator) SetAgentNeighborDist(agentID int, neighborDist float64) {
	rvo.Agents[agentID].NeighborDist = neighborDist
}

// SetAgentPosition :
func (rvo *RVOSimulator) SetAgentPosition(agentID int, position *Vector2) {
	rvo.Agents[agentID].Position = position
}

// SetAgentGoal :
func (rvo *RVOSimulator) SetAgentGoal(agentID int, goal *Vector2) {
	rvo.Agents[agentID].Goal = goal
}

// SetAgentPrefVelocity :
func (rvo *RVOSimulator) SetAgentPrefVelocity(agentID int, prefVelocity *Vector2) {
	rvo.Agents[agentID].PrefVelocity = prefVelocity
}

// SetAgentRadius :
func (rvo *RVOSimulator) SetAgentRadius(agentID int, radius float64) {
	rvo.Agents[agentID].Radius = radius
}

// SetAgentTimeHorizon :
func (rvo *RVOSimulator) SetAgentTimeHorizon(agentID int, timeHorizon float64) {
	rvo.Agents[agentID].TimeHorizon = timeHorizon
}

// SetAgentTimeHorizonObst :
func (rvo *RVOSimulator) SetAgentTimeHorizonObst(agentID int, timeHorizonObst float64) {
	rvo.Agents[agentID].TimeHorizonObst = timeHorizonObst
}

// SetAgentVelocity :
func (rvo *RVOSimulator) SetAgentVelocity(agentID int, velocity *Vector2) {
	rvo.Agents[agentID].Velocity = velocity
}

// SetAgentStatus :
func (rvo *RVOSimulator) SetAgentStatus(agentID int, status int) {
	rvo.Agents[agentID].Status = status
}

// SetTimeStep :
func (rvo *RVOSimulator) SetTimeStep(timeStep float64) {
	rvo.TimeStep = timeStep
}
