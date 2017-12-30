import math
import rvo.math as rvo_math

from .line import Line
from .vector import Vector2


class Agent:
    """
    Defines an agent in the simulation.
    """

    def __init__(self, simulator):
        self.simulator_ = simulator
        self.agentNeighbors_ = [] # (float, Agent)
        self.obstacleNeighbors_ = [] # (float, Obstacle)
        self.orcaLines_ = [] # Line
        self.position_ = Vector2()
        self.prefVelocity_ = Vector2()
        self.velocity_ = Vector2()
        self.id_ = 0
        self.maxNeighbors_ = 0
        self.maxSpeed_ = 0.0
        self.neighborDist_ = 0.0
        self.radius_ = 0.0
        self.timeHorizon_ = 0.0
        self.timeHorizonObst_ = 0.0
        self.newVelocity_ = Vector2()

    def computeNeighbors(self):
        """
        Computes the neighbors of this agent.
        """
        rangeSq = rvo_math.sqr(self.timeHorizonObst_ * self.maxSpeed_ + self.radius_)
        self.obstacleNeighbors_ = []
        self.simulator_.kdTree_.computeObstacleNeighbors(self, rangeSq)

        self.agentNeighbors_ = []
        if self.maxNeighbors_ > 0:
            rangeSq = rvo_math.sqr(self.neighborDist_)
            self.simulator_.kdTree_.computeAgentNeighbors(self, rangeSq)

    def computeNewVelocity(self):
        """
        Computes the new velocity of this agent.
        """
        self.orcaLines_ = []

        invTimeHorizonObst = 1.0 / self.timeHorizonObst_

        # Create obstacle ORCA lines.
        for i in range(len(self.obstacleNeighbors_)):
            obstacle1 = self.obstacleNeighbors_[i][1]
            obstacle2 = obstacle1.next_

            relativePosition1 = obstacle1.point_ - self.position_
            relativePosition2 = obstacle2.point_ - self.position_

            # Check if velocity obstacle of obstacle is already taken care of by previously constructed obstacle ORCA lines.
            alreadyCovered = False

            for j in range(len(self.orcaLines_)):
                det1 = rvo_math.det(invTimeHorizonObst * relativePosition1 - self.orcaLines_[j].point, self.orcaLines_[j].direction)
                det2 = rvo_math.det(invTimeHorizonObst * relativePosition2 - self.orcaLines_[j].point, self.orcaLines_[j].direction)
                if (det1 - invTimeHorizonObst * self.radius_ >= -rvo_math.EPSILON) and (det2 - invTimeHorizonObst * self.radius_ >= -rvo_math.EPSILON):
                    alreadyCovered = True
                    break

            if alreadyCovered:
                continue

            # Not yet covered. Check for collisions.
            distSq1 = rvo_math.absSq(relativePosition1)
            distSq2 = rvo_math.absSq(relativePosition2)

            radiusSq = rvo_math.sqr(self.radius_)

            obstacleVector = obstacle2.point_ - obstacle1.point_
            s = (-relativePosition1 @ obstacleVector) / rvo_math.absSq(obstacleVector)
            distSqLine = rvo_math.absSq(-relativePosition1 - s * obstacleVector)

            line = Line()

            if s < 0.0 and distSq1 <= radiusSq:
                # Collision with left vertex. Ignore if non-convex.
                if obstacle1.convex_:
                    line.point = Vector2(0.0, 0.0)
                    line.direction = rvo_math.normalize(Vector2(-relativePosition1.y, relativePosition1.x))
                    self.orcaLines_.append(line)
                continue
            elif s > 1.0 and distSq2 <= radiusSq:
                # Collision with right vertex. Ignore if non-convex or if it will be taken care of by neighboring obstacle.
                if obstacle2.convex_ and rvo_math.det(relativePosition2, obstacle2.direction_) >= 0.0:
                    line.point = Vector2(0.0, 0.0)
                    line.direction = rvo_math.normalize(Vector2(-relativePosition2.y, relativePosition2.x))
                    self.orcaLines_.append(line)
                continue
            elif s >= 0.0 and s < 1.0 and distSqLine <= radiusSq:
                # Collision with obstacle segment.
                line.point = Vector2(0.0, 0.0)
                line.direction = -obstacle1.direction_
                self.orcaLines_.append(line)
                continue

            # No collision. Compute legs. When obliquely viewed, both legs can come from a single vertex. Legs extend cut-off line when non-convex vertex.
            leftLegDirection = None
            rightLegDirection = None

            if s < 0.0 and distSqLine <= radiusSq:
                # Obstacle viewed obliquely so that left vertex defines velocity obstacle.
                if not obstacle1.convex_:
                    # Ignore obstacle.
                    continue

                obstacle2 = obstacle1

                leg1 = math.sqrt(distSq1 - radiusSq)
                leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * self.radius_, relativePosition1.x * self.radius_ + relativePosition1.y * leg1) / distSq1
                rightLegDirection = Vector2(relativePosition1.x * leg1 + relativePosition1.y * self.radius_, -relativePosition1.x * self.radius_ + relativePosition1.y * leg1) / distSq1
            elif s > 1.0 and distSqLine <= radiusSq:
                # Obstacle viewed obliquely so that right vertex defines velocity obstacle.
                if not obstacle2.convex_:
                    # Ignore obstacle.
                    continue

                obstacle1 = obstacle2

                leg2 = math.sqrt(distSq2 - radiusSq)
                leftLegDirection = Vector2(relativePosition2.x * leg2 - relativePosition2.y * self.radius_, relativePosition2.x * self.radius_ + relativePosition2.y * leg2) / distSq2
                rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * self.radius_, -relativePosition2.x * self.radius_ + relativePosition2.y * leg2) / distSq2
            else:
                # Usual situation.
                if obstacle1.convex_:
                    leg1 = math.sqrt(distSq1 - radiusSq)
                    leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * self.radius_, relativePosition1.x * self.radius_ + relativePosition1.y * leg1) / distSq1
                else:
                    # Left vertex non-convex left leg extends cut-off line.
                    leftLegDirection = -obstacle1.direction_

                if obstacle2.convex_:
                    leg2 = math.sqrt(distSq2 - radiusSq)
                    rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * self.radius_, -relativePosition2.x * self.radius_ + relativePosition2.y * leg2) / distSq2
                else:
                    # Right vertex non-convex right leg extends cut-off line.
                    rightLegDirection = obstacle1.direction_

            # Legs can never point into neighboring edge when convex vertex, take cutoff-line of neighboring edge instead. If velocity projected on "foreign" leg, no constraint is added.

            leftNeighbor = obstacle1.previous_

            isLeftLegForeign = False
            isRightLegForeign = False

            if obstacle1.convex_ and rvo_math.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0:
                # Left leg points into obstacle.
                leftLegDirection = -leftNeighbor.direction_
                isLeftLegForeign = True

            if obstacle2.convex_ and rvo_math.det(rightLegDirection, obstacle2.direction_) <= 0.0:
                # Right leg points into obstacle.
                rightLegDirection = obstacle2.direction_
                isRightLegForeign = True

            # Compute cut-off centers.
            leftCutOff = invTimeHorizonObst * (obstacle1.point_ - self.position_)
            rightCutOff = invTimeHorizonObst * (obstacle2.point_ - self.position_)
            cutOffVector = rightCutOff - leftCutOff

            # Project current velocity on velocity obstacle.

            # Check if current velocity is projected on cutoff circles.
            t = 0.5 if obstacle1 == obstacle2 else ((self.velocity_ - leftCutOff) @ cutOffVector) / rvo_math.absSq(cutOffVector)
            tLeft = (self.velocity_ - leftCutOff) @ leftLegDirection
            tRight = (self.velocity_ - rightCutOff) @ rightLegDirection

            if (t < 0.0 and tLeft < 0.0) or (obstacle1 == obstacle2 and tLeft < 0.0 and tRight < 0.0):
                # Project on left cut-off circle.
                unitW = rvo_math.normalize(self.velocity_ - leftCutOff)
                line.direction = Vector2(unitW.y, -unitW.x)
                line.point = leftCutOff + self.radius_ * invTimeHorizonObst * unitW
                self.orcaLines_.append(line)
                continue

            elif t > 1.0 and tRight < 0.0:
                # Project on right cut-off circle.
                unitW = rvo_math.normalize(self.velocity_ - rightCutOff)
                line.direction = Vector2(unitW.y, -unitW.x)
                line.point = rightCutOff + self.radius_ * invTimeHorizonObst * unitW
                self.orcaLines_.append(line)
                continue

            # Project on left leg, right leg, or cut-off line, whichever is closest to velocity.
            distSqCutoff = math.inf if t < 0.0 or t > 1.0 or obstacle1 == obstacle2 else rvo_math.absSq(self.velocity_ - (leftCutOff + t * cutOffVector))
            distSqLeft = math.inf if tLeft < 0.0 else rvo_math.absSq(self.velocity_ - (leftCutOff + tLeft * leftLegDirection))
            distSqRight = math.inf if tRight < 0.0 else rvo_math.absSq(self.velocity_ - (rightCutOff + tRight * rightLegDirection))

            if distSqCutoff <= distSqLeft and distSqCutoff <= distSqRight:
                # Project on cut-off line.
                line.direction = -obstacle1.direction_
                line.point = leftCutOff + self.radius_ * invTimeHorizonObst * Vector2(-line.direction.y, line.direction.x)
                self.orcaLines_.append(line)
                continue

            if distSqLeft <= distSqRight:
                # Project on left leg.
                if isLeftLegForeign:
                    continue

                line.direction = leftLegDirection
                line.point = leftCutOff + self.radius_ * invTimeHorizonObst * Vector2(-line.direction.y, line.direction.x)
                self.orcaLines_.append(line)

                continue

            # Project on right leg.
            if isRightLegForeign:
                continue

            line.direction = -rightLegDirection
            line.point = rightCutOff + self.radius_ * invTimeHorizonObst * Vector2(-line.direction.y, line.direction.x)
            self.orcaLines_.append(line)

        numObstLines = len(self.orcaLines_)

        invTimeHorizon = 1.0 / self.timeHorizon_

        # Create agent ORCA lines.
        for i in range(len(self.agentNeighbors_)):
            other = self.agentNeighbors_[i][1]

            relativePosition = other.position_ - self.position_
            relativeVelocity = self.velocity_ - other.velocity_

            distSq = rvo_math.absSq(relativePosition)
            combinedRadius = self.radius_ + other.radius_
            combinedRadiusSq = rvo_math.sqr(combinedRadius)

            line = Line()
            u = Vector2()

            if distSq > combinedRadiusSq:
                # No collision.
                w = relativeVelocity - invTimeHorizon * relativePosition

                # Vector from cutoff center to relative velocity.
                wLengthSq = rvo_math.absSq(w)
                dotProduct1 = w @ relativePosition

                if dotProduct1 < 0.0 and rvo_math.sqr(dotProduct1) > combinedRadiusSq * wLengthSq:
                    # Project on cut-off circle.
                    wLength = math.sqrt(wLengthSq)
                    unitW = w / wLength

                    line.direction = Vector2(unitW.y, -unitW.x)
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW
                else:
                    # Project on legs.
                    leg = math.sqrt(distSq - combinedRadiusSq)

                    if rvo_math.det(relativePosition, w) > 0.0:
                        # Project on left leg.
                        line.direction = Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq
                    else:
                        # Project on right leg.
                        line.direction = -Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq

                    dotProduct2 = relativeVelocity @ line.direction
                    u = dotProduct2 * line.direction - relativeVelocity
            else:
                # Collision. Project on cut-off circle of time timeStep.
                invTimeStep = 1.0 / self.simulator_.timeStep_

                # Vector from cutoff center to relative velocity.
                w = relativeVelocity - invTimeStep * relativePosition

                wLength = abs(w)
                unitW = w / wLength

                line.direction = Vector2(unitW.y, -unitW.x)
                u = (combinedRadius * invTimeStep - wLength) * unitW

            line.point = self.velocity_ + 0.5 * u
            self.orcaLines_.append(line)

        lineFail, self.newVelocity_ = self.linearProgram2(self.orcaLines_, self.maxSpeed_, self.prefVelocity_, False, self.newVelocity_)

        if lineFail < len(self.orcaLines_):
            self.newVelocity_ = self.linearProgram3(self.orcaLines_, numObstLines, lineFail, self.maxSpeed_, self.newVelocity_)

    def insertAgentNeighbor(self, agent, rangeSq):
        """
        Inserts an agent neighbor into the set of neighbors of this agent.
         
        Args:
            agent (Agent): A pointer to the agent to be inserted.
            rangeSq (float): The squared range around this agent.
        """
        if self != agent:
            distSq = rvo_math.absSq(self.position_ - agent.position_)

            if distSq < rangeSq:
                if len(self.agentNeighbors_) < self.maxNeighbors_:
                    self.agentNeighbors_.append((distSq, agent))

                i = len(self.agentNeighbors_) - 1
                while i != 0 and distSq < self.agentNeighbors_[i - 1][0]:
                    self.agentNeighbors_[i] = self.agentNeighbors_[i - 1]
                    i -= 1

                self.agentNeighbors_[i] = (distSq, agent)

                if len(self.agentNeighbors_) == self.maxNeighbors_:
                    rangeSq = self.agentNeighbors_[len(self.agentNeighbors_) - 1][0]
        return rangeSq

    def insertObstacleNeighbor(self, obstacle, rangeSq):
        """
        Inserts a static obstacle neighbor into the set of neighbors of this agent.

        Args:
            obstacle (Obstacle): The number of the static obstacle to be inserted.
            rangeSq (float): The squared range around this agent.
        """
        nextObstacle = obstacle.next_
        distSq = rvo_math.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, self.position_)

        if distSq < rangeSq:
            self.obstacleNeighbors_.append((distSq, obstacle))

            i = len(self.obstacleNeighbors_) - 1

            while i != 0 and distSq < self.obstacleNeighbors_[i - 1][0]:
                self.obstacleNeighbors_[i] = self.obstacleNeighbors_[i - 1]
                i -= 1

            self.obstacleNeighbors_[i] = (distSq, obstacle)

    def update(self):
        """
        Updates the two-dimensional position and two-dimensional velocity of this agent.
        """
        self.velocity_ = self.newVelocity_
        self.position_ += self.velocity_ * self.simulator_.timeStep_

    def linearProgram1(self, lines, lineNo, radius, optVelocity, directionOpt):
        """
        Solves a one-dimensional linear program on a specified line subject to linear constraints defined by lines and a circular constraint.

        Args:
            lines (list): Lines defining the linear constraints.
            lineNo (int): The specified line constraint.
            radius (float): The radius of the circular constraint.
            optVelocity (Vector2): The optimization velocity.
            directionOpt (bool): True if the direction should be optimized.

        Returns:
            bool: True if successful.
            Vector2: A reference to the result of the linear program.
        """
        dotProduct = lines[lineNo].point @ lines[lineNo].direction
        discriminant = rvo_math.sqr(dotProduct) + rvo_math.sqr(radius) - rvo_math.absSq(lines[lineNo].point)

        if discriminant < 0.0:
            # Max speed circle fully invalidates line lineNo.
            return False, None

        sqrtDiscriminant = math.sqrt(discriminant)
        tLeft = -dotProduct - sqrtDiscriminant
        tRight = -dotProduct + sqrtDiscriminant

        for i in range(lineNo):
            denominator = rvo_math.det(lines[lineNo].direction, lines[i].direction)
            numerator = rvo_math.det(lines[i].direction, lines[lineNo].point - lines[i].point)

            if abs(denominator) <= rvo_math.EPSILON:
                # Lines lineNo and i are (almost) parallel.
                if numerator < 0.0:
                    return False, None
                continue

            t = numerator / denominator

            if denominator >= 0.0:
                # Line i bounds line lineNo on the right.
                tRight = min(tRight, t)
            else:
                # Line i bounds line lineNo on the left.
                tLeft = max(tLeft, t)

            if tLeft > tRight:
                return False, None

        if directionOpt:
            # Optimize direction.
            if optVelocity @ lines[lineNo].direction > 0.0:
                # Take right extreme.
                result = lines[lineNo].point + tRight * lines[lineNo].direction
            else:
                # Take left extreme.
                result = lines[lineNo].point + tLeft * lines[lineNo].direction
        else:
            # Optimize closest point.
            t = lines[lineNo].direction @ (optVelocity - lines[lineNo].point)

            if t < tLeft:
                result = lines[lineNo].point + tLeft * lines[lineNo].direction
            elif t > tRight:
                result = lines[lineNo].point + tRight * lines[lineNo].direction
            else:
                result = lines[lineNo].point + t * lines[lineNo].direction

        return True, result

    def linearProgram2(self, lines, radius, optVelocity, directionOpt, result):
        """
        Solves a two-dimensional linear program subject to linear constraints defined by lines and a circular constraint.

        Args:
            lines (list): Lines defining the linear constraints.
            radius (float): The radius of the circular constraint.
            optVelocity (Vector2): The optimization velocity.
            directionOpt (bool): True if the direction should be optimized.
            result (Vector2): A reference to the result of the linear program.

        Returns:
            int: The number of the line it fails on, and the number of lines if successful.
            Vector2: A reference to the result of the linear program.
        """
        if directionOpt:
            # Optimize direction. Note that the optimization velocity is of unit length in this case.
            result = optVelocity * radius
        elif rvo_math.absSq(optVelocity) > rvo_math.sqr(radius):
            # Optimize closest point and outside circle.
            result = rvo_math.normalize(optVelocity) * radius
        else:
            # Optimize closest point and inside circle.
            result = optVelocity

        for i in range(len(lines)):
            if rvo_math.det(lines[i].direction, lines[i].point - result) > 0.0:
                # Result does not satisfy constraint i. Compute new optimal result.
                tempResult = result
                success, result = self.linearProgram1(lines, i, radius, optVelocity, directionOpt)
                if not success:
                    result = tempResult
                    return i, result

        return len(lines), result

    def linearProgram3(self, lines, numObstLines, beginLine, radius, result):
        """
        Solves a two-dimensional linear program subject to linear constraints defined by lines and a circular constraint.

        Args:
            lines (list): Lines defining the linear constraints.
            numObstLines (int): Count of obstacle lines.
            beginLine (int): The line on which the 2-d linear program failed.
            radius (float): The radius of the circular constraint.
            result (Vector2): A reference to the result of the linear program.

        Returns:
            Vector2: A reference to the result of the linear program.
        """
        distance = 0.0

        for i in range(beginLine, len(lines)):
            if rvo_math.det(lines[i].direction, lines[i].point - result) > distance:
                # Result does not satisfy constraint of line i.
                projLines = []

                for ii in range(numObstLines):
                    projLines.append(lines[ii])

                for j in range(numObstLines, i):
                    line = Line()
                    determinant = rvo_math.det(lines[i].direction, lines[j].direction)

                    if abs(determinant) <= rvo_math.EPSILON:
                        # Line i and line j are parallel.
                        if lines[i].direction @ lines[j].direction > 0.0:
                            # Line i and line j point in the same direction.
                            continue
                        else:
                            # Line i and line j point in opposite direction.
                            line.point = 0.5 * (lines[i].point + lines[j].point)
                    else:
                        line.point = lines[i].point + (rvo_math.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction

                    line.direction = rvo_math.normalize(lines[j].direction - lines[i].direction)
                    projLines.append(line)

                tempResult = result
                lineFail, result = self.linearProgram2(projLines, radius, Vector2(-lines[i].direction.y, lines[i].direction.x), True, result)
                if lineFail < len(projLines):
                    """
                    This should in principle not happen. The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept.
                    """
                    result = tempResult

                distance = rvo_math.det(lines[i].direction, lines[i].point - result)
        return result
