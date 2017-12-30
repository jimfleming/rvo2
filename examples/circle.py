"""
Example file showing a demo with 250 agents initially positioned evenly distributed on a circle attempting to move to the antipodal position on the circle.
"""
import math
import gym.envs.classic_control.rendering as rendering

import rvo.math as rvo_math

from rvo.vector import Vector2
from rvo.simulator import Simulator

RVO_RENDER = True


class Circle:

    def __init__(self):
        # Store the goals of the agents.
        self.goals_ = []
        self.simulator_ = Simulator()

    def setup_scenario(self):
        # Specify the global time step of the simulation.
        self.simulator_.set_time_step(0.25)

        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, Vector2(0.0, 0.0))

        # Add agents, specifying their start position, and store their goals on the opposite side of the environment.
        for i in range(250):
            self.simulator_.add_agent(200.0 *
                Vector2(math.cos(i * 2.0 * math.pi / 250.0), math.sin(i * 2.0 * math.pi / 250.0)))
            self.goals_.append(-self.simulator_.agents_[i].position_)

    def update_visualization(self, viewer):
        if not RVO_RENDER:
            return

        # Render the current position of all the agents.
        for i in range(self.simulator_.num_agents):
            position = self.simulator_.agents_[i].position_
            color = [0, 0, 0]
            color[i % 3] = 1
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))

        viewer.render()

    def set_preferred_velocities(self):
        """
        Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        """
        for i in range(self.simulator_.num_agents):
            goal_vector = self.goals_[i] - self.simulator_.agents_[i].position_

            if rvo_math.abs_sq(goal_vector) > 1.0:
                goal_vector = rvo_math.normalize(goal_vector)

            self.simulator_.set_agent_pref_velocity(i, goal_vector)

    def reached_goal(self):
        """
        Check if all agents have reached their goals.
        """
        for i in range(self.simulator_.num_agents):
            if rvo_math.abs_sq(self.simulator_.agents_[i].position_ - self.goals_[i]) > self.simulator_.agents_[i].radius_ * self.simulator_.agents_[i].radius_:
                return False

        return True


def main():
    viewer = None

    circle = Circle()

    # Set up the scenario.
    circle.setup_scenario()

    # Perform (and manipulate) the simulation.
    while not circle.reached_goal():
        if RVO_RENDER:
            if viewer is None:
                viewer = rendering.Viewer(750, 750)
                viewer.set_bounds(-225, 225, -225, 225)

            circle.update_visualization(viewer)
        circle.set_preferred_velocities()
        circle.simulator_.step()


if __name__ == '__main__':
    main()
