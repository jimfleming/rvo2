"""
Example file showing a demo with 250 agents initially positioned evenly distributed on a circle attempting to move to the antipodal position on the circle.
"""
import math
import gym.envs.classic_control.rendering as rendering

import rvo.math as rvo_math

from rvo.vector import Vector2
from rvo.simulator import Simulator

RVO_OUTPUT_TIME_AND_POSITIONS = True


class Circle:

    def __init__(self):
        # Store the goals of the agents.
        self.goals_ = []
        self.simulator_ = Simulator()

    def setupScenario(self):
        # Specify the global time step of the simulation.
        self.simulator_.setTimeStep(0.25)

        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.setAgentDefaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, Vector2(0.0, 0.0))

        # Add agents, specifying their start position, and store their goals on the opposite side of the environment.
        for i in range(250):
            self.simulator_.addAgent(200.0 *
                Vector2(math.cos(i * 2.0 * math.pi / 250.0), math.sin(i * 2.0 * math.pi / 250.0)))
            self.goals_.append(-self.simulator_.getAgentPosition(i))

    def updateVisualization(self, viewer):
        if not RVO_OUTPUT_TIME_AND_POSITIONS:
            return

        # Output the current global time.
        global_time = self.simulator_.getGlobalTime()

        # Render the current position of all the agents.
        colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
        for i in range(self.simulator_.getNumAgents()):
            position = self.simulator_.getAgentPosition(i)
            color = colors[i % len(colors)]
            circle = viewer.draw_circle(radius=self.simulator_.defaultAgent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))

        viewer.render()

    def setPreferredVelocities(self):
        """
        Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        """
        for i in range(self.simulator_.getNumAgents()):
            goal_vector = self.goals_[i] - self.simulator_.getAgentPosition(i)

            if rvo_math.absSq(goal_vector) > 1.0:
                goal_vector = rvo_math.normalize(goal_vector)

            self.simulator_.setAgentPrefVelocity(i, goal_vector)

    def reachedGoal(self):
        """
        Check if all agents have reached their goals.
        """
        for i in range(self.simulator_.getNumAgents()):
            if rvo_math.absSq(self.simulator_.getAgentPosition(i) - self.goals_[i]) > self.simulator_.getAgentRadius(i) * self.simulator_.getAgentRadius(i):
                return False

        return True


def main():
    viewer = rendering.Viewer(600, 600)
    viewer.set_bounds(-300, 300, -300, 300)

    circle = Circle()

    # Set up the scenario.
    circle.setupScenario()

    # Perform (and manipulate) the simulation.
    while not circle.reachedGoal():
        if RVO_OUTPUT_TIME_AND_POSITIONS:
            circle.updateVisualization(viewer)
        circle.setPreferredVelocities()
        circle.simulator_.doStep()


if __name__ == '__main__':
    main()
