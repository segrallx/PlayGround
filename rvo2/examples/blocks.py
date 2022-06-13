"""
Example file showing a demo with 100 agents split in four groups initially positioned in four corners of the environment. Each agent attempts to move to other side of the environment through a narrow passage generated by four obstacles. There is no roadmap to guide the agents around the obstacles.
"""
import math
import random
import gym.envs.classic_control.rendering as rendering

import rvo.math as rvo_math

from rvo.vector import Vector2
from rvo.simulator import Simulator

RVO_RENDER = True


class Blocks:

    def __init__(self):
        self.goals_ = [] # Vector2
        self.obstacles_ = [] # Vector2
        self.simulator_ = Simulator()

    def setup_scenario(self):
        # Specify the global time step of the simulation.
        self.simulator_.set_time_step(0.25)

        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.set_agent_defaults(15.0, 10, 5.0, 5.0, 2.0, 2.0, Vector2(0.0, 0.0))

        # Add agents, specifying their start position, and store their goals on the opposite side of the environment.
        for i in range(5):
            for j in range(5):
                self.simulator_.add_agent(Vector2(55.0 + i * 10.0, 55.0 + j * 10.0))
                self.goals_.append(Vector2(-75.0, -75.0))

                self.simulator_.add_agent(Vector2(-55.0 - i * 10.0, 55.0 + j * 10.0))
                self.goals_.append(Vector2(75.0, -75.0))

                self.simulator_.add_agent(Vector2(55.0 + i * 10.0, -55.0 - j * 10.0))
                self.goals_.append(Vector2(-75.0, 75.0))

                self.simulator_.add_agent(Vector2(-55.0 - i * 10.0, -55.0 - j * 10.0))
                self.goals_.append(Vector2(75.0, 75.0))

        #####################################################################################
        # # Add (polygonal) obstacles, specifying their vertices in counterclockwise order. #
        # obstacle1 = []                                                                    #
        # obstacle1.append(Vector2(-10.0, 40.0))                                            #
        # obstacle1.append(Vector2(-40.0, 40.0))                                            #
        # obstacle1.append(Vector2(-40.0, 10.0))                                            #
        # obstacle1.append(Vector2(-10.0, 10.0))                                            #
        # self.simulator_.add_obstacle(obstacle1)                                           #
        # self.obstacles_.append(obstacle1)                                                 #
        #                                                                                   #
        # obstacle2 = []                                                                    #
        # obstacle2.append(Vector2(10.0, 40.0))                                             #
        # obstacle2.append(Vector2(10.0, 10.0))                                             #
        # obstacle2.append(Vector2(40.0, 10.0))                                             #
        # obstacle2.append(Vector2(40.0, 40.0))                                             #
        # self.simulator_.add_obstacle(obstacle2)                                           #
        # self.obstacles_.append(obstacle2)                                                 #
        #                                                                                   #
        # obstacle3 = []                                                                    #
        # obstacle3.append(Vector2(10.0, -40.0))                                            #
        # obstacle3.append(Vector2(40.0, -40.0))                                            #
        # obstacle3.append(Vector2(40.0, -10.0))                                            #
        # obstacle3.append(Vector2(10.0, -10.0))                                            #
        # self.simulator_.add_obstacle(obstacle3)                                           #
        # self.obstacles_.append(obstacle3)                                                 #
        #                                                                                   #
        # obstacle4 = []                                                                    #
        # obstacle4.append(Vector2(-10.0, -40.0))                                           #
        # obstacle4.append(Vector2(-10.0, -10.0))                                           #
        # obstacle4.append(Vector2(-40.0, -10.0))                                           #
        # obstacle4.append(Vector2(-40.0, -40.0))                                           #
        # self.simulator_.add_obstacle(obstacle4)                                           #
        # self.obstacles_.append(obstacle4)                                                 #
        #####################################################################################

        obstacle1 = []                                                                    #
        obstacle1.append(Vector2(2.0, -2.0))                                            #
        obstacle1.append(Vector2(2.0, 2.0))                                            #
        obstacle1.append(Vector2(-2.0, 2.0))                                            #
        obstacle1.append(Vector2(-2.0, -2.0))                                            #


        self.simulator_.add_obstacle(obstacle1)                                           #
        self.obstacles_.append(obstacle1)                                                 #

        # Process the obstacles so that they are accounted for in the simulation.
        self.simulator_.process_obstacles()

    def update_visualization(self, viewer):
        if not RVO_RENDER:
            return

        for i in range(self.simulator_.num_agents):
            position = self.simulator_.agents_[i].position_
            color = [0, 0, 0]
            color[i % 3] = 1
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))

        for obstacle in self.obstacles_:
            v = [(vec.x, vec.y) for vec in obstacle]
            viewer.draw_polygon(v=v, color=(0, 0, 0))

        viewer.render()

    def set_preferred_velocities(self):
        # Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        for i in range(self.simulator_.num_agents):
            goal_vector = self.goals_[i] - self.simulator_.agents_[i].position_

            if rvo_math.abs_sq(goal_vector) > 1.0:
                goal_vector = rvo_math.normalize(goal_vector)

            self.simulator_.set_agent_pref_velocity(i, goal_vector)

            # Perturb a little to avoid deadlocks due to perfect symmetry.
            angle = random.random() * 2.0 * math.pi
            dist = random.random() * 0.0001

            self.simulator_.set_agent_pref_velocity(i, self.simulator_.agents_[i].pref_velocity_ +
                dist * Vector2(math.cos(angle), math.sin(angle)))

    def reached_goal(self):
        # Check if all agents have reached their goals.
        for i in range(self.simulator_.num_agents):
            if rvo_math.abs_sq(self.simulator_.agents_[i].position_ - self.goals_[i]) > 400.0:
                return False
        return True


def main():
    viewer = None

    blocks = Blocks()

    # Set up the scenario.
    blocks.setup_scenario()

    # Perform (and manipulate) the simulation.
    while not blocks.reached_goal():
        if RVO_RENDER:
            if viewer is None:
                viewer = rendering.Viewer(750, 750)
                viewer.set_bounds(-100, 100, -100, 100)

            blocks.update_visualization(viewer)
        blocks.set_preferred_velocities()
        blocks.simulator_.step()


if __name__ == '__main__':
    main()