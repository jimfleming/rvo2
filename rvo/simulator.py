import rvo.math as rvo_math

from .kdtree import KdTree
from .agent import Agent
from .obstacle import Obstacle


class Simulator:
    """
    Defines the simulation.
    """

    def __init__(self):
        """
        Constructs and initializes a simulation.
        """
        self.agents_ = []
        self.defaultAgent_ = None
        self.kdTree_ = KdTree(self)
        self.obstacles_ = []
        self.globalTime_ = 0.0
        self.timeStep_ = 0.1
        self.doneEvents_ = []

    def addAgent(self, position):
        """
        Adds a new agent with default properties to the simulation.

        Args:
            position (Vector2): The two-dimensional starting position of this agent.

        Returns:
            int: The number of the agent, or -1 when the agent defaults have not been set.
        """
        if self.defaultAgent_ is None:
            raise ArgumentError('Default agent not set. Call setAgentDefaults first.')

        agent = Agent(self)
        agent.id_ = len(self.agents_)
        agent.maxNeighbors_ = self.defaultAgent_.maxNeighbors_
        agent.maxSpeed_ = self.defaultAgent_.maxSpeed_
        agent.neighborDist_ = self.defaultAgent_.neighborDist_
        agent.position_ = position
        agent.radius_ = self.defaultAgent_.radius_
        agent.timeHorizon_ = self.defaultAgent_.timeHorizon_
        agent.timeHorizonObst_ = self.defaultAgent_.timeHorizonObst_
        agent.velocity_ = self.defaultAgent_.velocity_
        self.agents_.append(agent)

        return agent.id_

    def addObstacle(self, vertices):
        """
        Adds a new obstacle to the simulation.

        Args:
            vertices (list): List of the vertices of the polygonal obstacle in counterclockwise order.

        Returns:
            int: The number of the first vertex of the obstacle, or -1 when the number of vertices is less than two.

        Remarks:
            To add a "negative" obstacle, e.g. a bounding polygon around the environment, the vertices should be listed in clockwise order.
        """
        if len(vertices) < 2:
            raise ArgumentError('Must have at least 2 vertices.')

        obstacleNo = len(self.obstacles_)

        for i in range(len(vertices)):
            obstacle = Obstacle()
            obstacle.point_ = vertices[i]

            if i != 0:
                obstacle.previous_ = self.obstacles_[len(self.obstacles_) - 1]
                obstacle.previous_.next_ = obstacle

            if i == len(vertices) - 1:
                obstacle.next_ = self.obstacles_[obstacleNo]
                obstacle.next_.previous_ = obstacle

            obstacle.direction_ = rvo_math.normalize(vertices[0 if i == len(vertices) - 1 else i + 1] - vertices[i])

            if len(vertices) == 2:
                obstacle.convex_ = True
            else:
                obstacle.convex_ = (rvo_math.leftOf(vertices[len(vertices) - 1 if i == 0 else i - 1], vertices[i], vertices[0 if i == len(vertices) - 1 else i + 1]) >= 0.0)

            obstacle.id_ = len(self.obstacles_)
            self.obstacles_.append(obstacle)

        return obstacleNo

    def doStep(self):
        """
        Performs a simulation step and updates the two-dimensional position and two-dimensional velocity of each agent.

        Returns:
            float: The global time after the simulation step.
        """

        self.kdTree_.buildAgentTree()

        # TODO: Try async/await here.
        # Performs a simulation step.
        for agentNo in range(self.getNumAgents()):
            self.agents_[agentNo].computeNeighbors()
            self.agents_[agentNo].computeNewVelocity()

        # TODO: Try async/await here.
        # Updates the two-dimensional position and two-dimensional velocity of each agent.
        for agentNo in range(self.getNumAgents()):
            self.agents_[agentNo].update()

        self.globalTime_ += self.timeStep_

        return self.globalTime_

    def getAgentPosition(self, agentNo):
        """
        Returns the two-dimensional position of a specified agent.

        Args:
            agentNo (int): The number of the agent whose two-dimensional position is to be retrieved.

        Returns:
            Vector2: The present two-dimensional position of the (center of the) agent.
        """
        return self.agents_[agentNo].position_

    def getAgentPrefVelocity(self, agentNo):
        """
        Returns the two-dimensional preferred velocity of a specified agent.

        Args:
            agentNo (int): The number of the agent whose two-dimensional preferred velocity is to be retrieved.

        Returns:
            Vector2: The present two-dimensional preferred velocity of the agent.
        """
        return self.agents_[agentNo].prefVelocity_

    def getAgentRadius(self, agentNo):
        """
        Returns the radius of a specified agent.

        Args:
            agentNo (int): The number of the agent whose radius is to be retrieved.

        Returns:
            float: The present radius of the agent.
        """
        return self.agents_[agentNo].radius_

    # TODO: property
    def getGlobalTime(self):
        """
        Returns the global time of the simulation.
        """
        return self.globalTime_

    # TODO: property
    def getNumAgents(self):
        """
        Returns the count of agents in the simulation.
        """
        return len(self.agents_)

    def processObstacles(self):
        """
        Processes the obstacles that have been added so that they are accounted for in the simulation.

        Remarks:
            Obstacles added to the simulation after this function has been called are not accounted for in the simulation.
        """
        self.kdTree_.buildObstacleTree()

    def setAgentDefaults(self, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity):
        """
        Sets the default properties for any new agent that is added.

        Args:
            neighborDist (float): The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
            maxNeighbors (int): The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
            timeHorizon (float): The default minimal amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
            timeHorizonObst (float): The default minimal amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to obstacles. The larger this number, the sooner an agent will respond to the presence of obstacles, but the less freedom the agent has in choosing its velocities. Must be positive.
            radius (float): The default radius of a new agent. Must be non-negative.
            maxSpeed (float): The default maximum speed of a new agent. Must be non-negative.
            velocity (Vector2): The default initial two-dimensional linear velocity of a new agent.
        """
        if self.defaultAgent_ is None:
            self.defaultAgent_ = Agent(self)

        self.defaultAgent_.maxNeighbors_ = maxNeighbors
        self.defaultAgent_.maxSpeed_ = maxSpeed
        self.defaultAgent_.neighborDist_ = neighborDist
        self.defaultAgent_.radius_ = radius
        self.defaultAgent_.timeHorizon_ = timeHorizon
        self.defaultAgent_.timeHorizonObst_ = timeHorizonObst
        self.defaultAgent_.velocity_ = velocity

    def setAgentPrefVelocity(self, agentNo, prefVelocity):
        """
        Sets the two-dimensional preferred velocity of a specified agent.

        Args:
            agentNo (int): The number of the agent whose two-dimensional preferred velocity is to be modified.
            prefVelocity (Vector2): The replacement of the two-dimensional preferred velocity.
        """
        self.agents_[agentNo].prefVelocity_ = prefVelocity

    def setTimeStep(self, timeStep):
        """
        Sets the time step of the simulation.

        Args:
            timeStep (float): The time step of the simulation. Must be positive.
        """
        self.timeStep_ = timeStep
