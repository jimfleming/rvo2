import rvo.math as rvo_math

from .obstacle import Obstacle

MAX_LEAF_SIZE = 10


class AgentTreeNode:
    def __init__(self):
        self.begin_ = None
        self.end_ = None
        self.left_ = None
        self.right_ = None
        self.maxX_ = None
        self.maxY_ = None


class ObstacleTreeNode:
    def __init__(self):
        self.obstacle_ = None
        self.left_ = None
        self.right_ = None


class FloatPair:
    """
    Defines a pair of scalar values.
    """

    def __init__(self, a, b):
        self.a_ = a
        self.b_ = b

    def __lt__(self, other):
        """
        Returns true if the first pair of scalar values is less than the second pair of scalar values.
        """
        return self.a_ < other.a_ or not (other.a_ < self.a_) and self.b_ < other.b_

    def __le__(self, other):
        """
        Returns true if the first pair of scalar values is less than or equal to the second pair of scalar values.
        """
        return (self.a_ == other.a_ and self.b_ == other.b_) or self < other

    def __gt__(self, other):
        """
        Returns true if the first pair of scalar values is greater than the second pair of scalar values.
        """
        return not (self <= other)

    def __ge__(self, other):
        """
        Returns true if the first pair of scalar values is greater than or equal to the second pair of scalar values.
        """
        return not (self < other)


class KdTree:
    """
    Defines k-D trees for agents and static obstacles in the simulation.
    """

    def __init__(self, simulator):
        self.simulator_ = simulator
        self.agents_ = None
        self.agentTree_ = None
        self.obstacleTree_ = None

    def build_agent_tree(self):
        """
        Builds an agent k-D tree.
        """
        if self.agents_ is None or len(self.agents_) != len(self.simulator_.agents_):
            self.agents_ = list(self.simulator_.agents_)
            self.agentTree_ = [AgentTreeNode() for i in range(2 * len(self.agents_))]

        if len(self.agents_) != 0:
            self.build_agent_tree_recursive(0, len(self.agents_), 0)

    def build_obstacle_tree(self):
        """
        Builds an obstacle k-D tree.
        """
        obstacles = list(self.simulator_.obstacles_)
        self.obstacleTree_ = self.build_obstacle_treeRecursive(obstacles)

    def compute_agent_neighbors(self, agent, rangeSq):
        """
        Computes the agent neighbors of the specified agent.

        Args:
            agent (Agent): The agent for which agent neighbors are to be computed.
            rangeSq (float): The squared range around the agent.
        """
        self.query_agent_tree_recursive(agent, rangeSq, 0)

    def compute_obstacle_neighbors(self, agent, rangeSq):
        """
        Computes the obstacle neighbors of the specified agent.

        Args:
            agent (Agent): The agent for which obstacle neighbors are to be computed.
            rangeSq (float): The squared range around the agent.
        """
        self.query_obstacle_tree_recursive(agent, rangeSq, self.obstacleTree_)

    def build_agent_tree_recursive(self, begin, end, node):
        """
        Recursive method for building an agent k-D tree.

        Args:
            begin (int): The beginning agent k-D tree node node index.
            end (int): The ending agent k-D tree node index.
            node (int): The current agent k-D tree node index.
        """
        self.agentTree_[node].begin_ = begin
        self.agentTree_[node].end_ = end
        self.agentTree_[node].minX_ = self.agentTree_[node].maxX_ = self.agents_[begin].position_.x_
        self.agentTree_[node].minY_ = self.agentTree_[node].maxY_ = self.agents_[begin].position_.y_

        for i in range(begin + 1, end):
            self.agentTree_[node].maxX_ = max(self.agentTree_[node].maxX_, self.agents_[i].position_.x_)
            self.agentTree_[node].minX_ = min(self.agentTree_[node].minX_, self.agents_[i].position_.x_)
            self.agentTree_[node].maxY_ = max(self.agentTree_[node].maxY_, self.agents_[i].position_.y_)
            self.agentTree_[node].minY_ = min(self.agentTree_[node].minY_, self.agents_[i].position_.y_)

        if end - begin > MAX_LEAF_SIZE:
            # No leaf node.
            isVertical = self.agentTree_[node].maxX_ - self.agentTree_[node].minX_ > self.agentTree_[node].maxY_ - self.agentTree_[node].minY_
            splitValue = 0.5 * (self.agentTree_[node].maxX_ + self.agentTree_[node].minX_ if isVertical else self.agentTree_[node].maxY_ + self.agentTree_[node].minY_)

            left = begin
            right = end

            while left < right:
                while left < right and (self.agents_[left].position_.x_ if isVertical else self.agents_[left].position_.y_) < splitValue:
                    left += 1

                while right > left and (self.agents_[right - 1].position_.x_ if isVertical else self.agents_[right - 1].position_.y_) >= splitValue:
                    right -= 1

                if left < right:
                    tempAgent = self.agents_[left]
                    self.agents_[left] = self.agents_[right - 1]
                    self.agents_[right - 1] = tempAgent
                    left += 1
                    right -= 1

            leftSize = left - begin

            if leftSize == 0:
                leftSize += 1
                left += 1
                right += 1

            self.agentTree_[node].left_ = node + 1
            self.agentTree_[node].right_ = node + 2 * leftSize

            self.build_agent_tree_recursive(begin, left, self.agentTree_[node].left_)
            self.build_agent_tree_recursive(left, end, self.agentTree_[node].right_)

    def build_obstacle_treeRecursive(self, obstacles):
        """
        Recursive method for building an obstacle k-D tree.

        Args:
            obstacles (list): A list of obstacles.

        Returns:
            ObstacleTreeNode: An obstacle k-D tree node.
        """
        if len(obstacles) == 0:
            return None

        node = ObstacleTreeNode()

        optimalSplit = 0
        minLeft = len(obstacles)
        minRight = len(obstacles)

        for i in range(len(obstacles)):
            leftSize = 0
            rightSize = 0

            obstacleI1 = obstacles[i]
            obstacleI2 = obstacleI1.next_

            # Compute optimal split node.
            for j in range(len(obstacles)):
                if i == j:
                    continue

                obstacleJ1 = obstacles[j]
                obstacleJ2 = obstacleJ1.next_

                j1LeftOfI = rvo_math.left_of(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_)
                j2LeftOfI = rvo_math.left_of(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_)

                if j1LeftOfI >= -rvo_math.EPSILON and j2LeftOfI >= -rvo_math.EPSILON:
                    leftSize += 1
                elif j1LeftOfI <= rvo_math.EPSILON and j2LeftOfI <= rvo_math.EPSILON:
                    rightSize += 1
                else:
                    leftSize += 1
                    rightSize += 1

                if FloatPair(max(leftSize, rightSize), min(leftSize, rightSize)) >= FloatPair(max(minLeft, minRight), min(minLeft, minRight)):
                    break

            if FloatPair(max(leftSize, rightSize), min(leftSize, rightSize)) < FloatPair(max(minLeft, minRight), min(minLeft, minRight)):
                minLeft = leftSize
                minRight = rightSize
                optimalSplit = i

        # Build split node.
        leftObstacles = [None for _ in range(minLeft)]
        rightObstacles = [None for _ in range(minRight)]

        leftCounter = 0
        rightCounter = 0
        i = optimalSplit

        obstacleI1 = obstacles[i]
        obstacleI2 = obstacleI1.next_

        for j in range(len(obstacles)):
            if i == j:
                continue

            obstacleJ1 = obstacles[j]
            obstacleJ2 = obstacleJ1.next_

            j1LeftOfI = rvo_math.left_of(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_)
            j2LeftOfI = rvo_math.left_of(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_)

            if j1LeftOfI >= -rvo_math.EPSILON and j2LeftOfI >= -rvo_math.EPSILON:
                leftObstacles[leftCounter] = obstacles[j]
                leftCounter += 1
            elif j1LeftOfI <= rvo_math.EPSILON and j2LeftOfI <= rvo_math.EPSILON:
                rightObstacles[rightCounter] = obstacles[j]
                rightCounter += 1
            else:
                # Split obstacle j.
                t = rvo_math.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / rvo_math.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_)

                splitPoint = obstacleJ1.point_ + t * (obstacleJ2.point_ - obstacleJ1.point_)

                newObstacle = Obstacle()
                newObstacle.point_ = splitPoint
                newObstacle.previous_ = obstacleJ1
                newObstacle.next_ = obstacleJ2
                newObstacle.convex_ = True
                newObstacle.direction_ = obstacleJ1.direction_

                newObstacle.id_ = len(self.simulator_.obstacles_)

                self.simulator_.obstacles_.append(newObstacle)

                obstacleJ1.next_ = newObstacle
                obstacleJ2.previous_ = newObstacle

                if j1LeftOfI > 0.0:
                    leftObstacles[leftCounter] = obstacleJ1
                    leftCounter += 1
                    rightObstacles[rightCounter] = newObstacle
                    rightCounter += 1
                else:
                    rightObstacles[rightCounter] = obstacleJ1
                    rightCounter += 1
                    leftObstacles[leftCounter] = newObstacle
                    leftCounter += 1

        node.obstacle_ = obstacleI1
        node.left_ = self.build_obstacle_treeRecursive(leftObstacles)
        node.right_ = self.build_obstacle_treeRecursive(rightObstacles)

        return node

    def query_agent_tree_recursive(self, agent, rangeSq, node):
        """
        Recursive method for computing the agent neighbors of the specified agent.

        Args:
            agent (Agent): The agent for which agent neighbors are to be computed.
            rangeSq (float): The squared range around the agent.
            node (int): The current agent k-D tree node index.
        """
        if self.agentTree_[node].end_ - self.agentTree_[node].begin_ <= MAX_LEAF_SIZE:
            for i in range(self.agentTree_[node].begin_, self.agentTree_[node].end_):
                rangeSq = agent.insert_agent_neighbor(self.agents_[i], rangeSq)
        else:
            distSqLeft = rvo_math.square(max(0.0, self.agentTree_[self.agentTree_[node].left_].minX_ - agent.position_.x_)) + rvo_math.square(max(0.0, agent.position_.x_ - self.agentTree_[self.agentTree_[node].left_].maxX_)) + rvo_math.square(max(0.0, self.agentTree_[self.agentTree_[node].left_].minY_ - agent.position_.y_)) + rvo_math.square(max(0.0, agent.position_.y_ - self.agentTree_[self.agentTree_[node].left_].maxY_))
            distSqRight = rvo_math.square(max(0.0, self.agentTree_[self.agentTree_[node].right_].minX_ - agent.position_.x_)) + rvo_math.square(max(0.0, agent.position_.x_ - self.agentTree_[self.agentTree_[node].right_].maxX_)) + rvo_math.square(max(0.0, self.agentTree_[self.agentTree_[node].right_].minY_ - agent.position_.y_)) + rvo_math.square(max(0.0, agent.position_.y_ - self.agentTree_[self.agentTree_[node].right_].maxY_))

            if distSqLeft < distSqRight:
                if distSqLeft < rangeSq:
                    rangeSq = self.query_agent_tree_recursive(agent, rangeSq, self.agentTree_[node].left_)

                    if distSqRight < rangeSq:
                        rangeSq = self.query_agent_tree_recursive(agent, rangeSq, self.agentTree_[node].right_)
            else:
                if distSqRight < rangeSq:
                    rangeSq = self.query_agent_tree_recursive(agent, rangeSq, self.agentTree_[node].right_)

                    if distSqLeft < rangeSq:
                        rangeSq = self.query_agent_tree_recursive(agent, rangeSq, self.agentTree_[node].left_)
        return rangeSq

    def query_obstacle_tree_recursive(self, agent, rangeSq, node):
        """
        Recursive method for computing the obstacle neighbors of the specified agent.

        Args:
            agent (Agent): The agent for which obstacle neighbors are to be computed.
            rangeSq (float): The squared range around the agent.
            node (ObstacleTreeNode): The current obstacle k-D node.
        """
        if node is not None:
            obstacle1 = node.obstacle_
            obstacle2 = obstacle1.next_

            agentLeftOfLine = rvo_math.left_of(obstacle1.point_, obstacle2.point_, agent.position_)

            self.query_obstacle_tree_recursive(agent, rangeSq, node.left_ if agentLeftOfLine >= 0.0 else node.right_)

            distSqLine = rvo_math.square(agentLeftOfLine) / rvo_math.abs_sq(obstacle2.point_ - obstacle1.point_)

            if distSqLine < rangeSq:
                if agentLeftOfLine < 0.0:
                    # Try obstacle at this node only if agent is on right side of obstacle (and can see obstacle).
                    agent.insert_obstacle_neighbor(node.obstacle_, rangeSq)

                # Try other side of line.
                self.query_obstacle_tree_recursive(agent, rangeSq, node.right_ if agentLeftOfLine >= 0.0 else node.left_)
