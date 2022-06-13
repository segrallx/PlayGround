"""
Example file showing a demo with 250 agents initially positioned evenly distributed on a circle attempting to move to the antipodal position on the circle.
"""
import math
import gym.envs.classic_control.rendering as rendering
import random
import rvo.math as rvo_math

from rvo.vector import Vector2
from rvo.simulator import Simulator

RVO_RENDER = True


class CircleBlock:
    def __init__(self, pos, radius):
        self.pos_ = pos
        self.radius_ = radius


    def to_polygon(self):
        ret = []
        edgeCnt = 7
        for i in range(edgeCnt):
            ret.append(self.pos_+ self.radius_ *1.2*
                       Vector2( math.cos(i * 2.0 * math.pi / edgeCnt), math.sin(i * 2.0 * math.pi / edgeCnt)))
        #for v  in ret :
        #    print(v.x_, v.y_)
        return ret



class Circle:

    def __init__(self):
        # Store the goals of the agents.
        self.goals_ = []
        self.simulator_ = Simulator()
        self.obstacles_ = [] # Vector2
        self.shows_ = [] # Vector2
        self.count_ = 0

    # 随机点，有随机的停止点.
    def setup_scenario(self):
        # Specify the global time step of the simulation.
        self.simulator_.set_time_step(0.5)
        radius = 10
        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.set_agent_defaults(2*radius, 6, 10, 10.0, radius, radius, Vector2(18.0, 8.0))
        width = 225


        size = int(width/(radius*3))
        xSize = width/3
        '''
        self.simulator_.add_agent(Vector2(-2*xSize,1*(radius*2))) #
        self.goals_.append(Vector2(2*xSize, (radius*2)))        #
        '''

        # 弧形阻挡

        '''
        goal1 = self.changeAngle(Vector2(-100),  Vector2(0,0), 0) #
        idx = self.simulator_.add_agent(goal1) #                      #
        self.goals_.append(goal1)        #                            #
        self.simulator_.agents_[idx].static_= True                    #


        for j in range(1, 5):                            #
            angele = 25
            goal1 = self.changeAngle(Vector2(-100,0),  Vector2(0,0), angele*j+random.randint(-3,3))
            goal2 = self.changeAngle(Vector2(-100,0),  Vector2(0,0), 360-(angele*j +random.randint(-3,3)))
            idx = self.simulator_.add_agent(goal1) #
            self.goals_.append(goal1)        #
            self.simulator_.agents_[idx].static_= True

            idx = self.simulator_.add_agent(goal2) #
            self.goals_.append(goal2)        #
            self.simulator_.agents_[idx].static_= True
        '''

        # 网格阻挡
        for j in range(-size, size):                            #
            #continue
            x1 = -0 * xSize                                      #
            if random.randint(0,100)<70:
                idx = self.simulator_.add_agent(Vector2(x1,j*(radius*2))) #
                self.goals_.append(Vector2(x1, j*(radius*2)))        #
                self.simulator_.agents_[idx].static_= True

            continue
            #随机在起左右边生成阻挡.                                                #
            xx = random.randint(0,7)                                                # #
            yy = random.randint(0,7)
            if xx==0:                                                               # #
                idx = self.simulator_.add_agent(Vector2(x1-2*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1-2*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif xx == 1:                                                           # #
                idx = self.simulator_.add_agent(Vector2(x1+2*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1+2*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif xx == 2:                                                           # #
                idx = self.simulator_.add_agent(Vector2(x1-2*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1-2*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif xx ==3:
                idx = self.simulator_.add_agent(Vector2(x1+2*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1+2*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            else:                                                                   # #
                fxfwef=0                                                                 # #


            if yy==0:
                idx = self.simulator_.add_agent(Vector2(x1-4*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1-4*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif  yy ==1:
                idx = self.simulator_.add_agent(Vector2(x1+4*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1+4*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif yy==2:
                idx = self.simulator_.add_agent(Vector2(x1-4*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1-4*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif yy ==3:
                idx = self.simulator_.add_agent(Vector2(x1+4*radius,j*(radius*2)))  # #
                self.goals_.append(Vector2(x1+4*radius, j*(radius*2)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            else:
                yfwef=0


        #生成移动点.
        x = random.randint(-size, size)
        c =0
        for j in range(-size, size):                             #
            c+=1
            if c>10000:
                continue
            if j%10<5:
                continue
            x1 = -2 * xSize                                      #
            x2 = 2 * xSize                                       #
            self.simulator_.add_agent(Vector2(x1,(j+x) *(radius*2)))  #
            self.goals_.append(Vector2(x2, -(j+x)*(radius*2)))       #
            #break


        ###############################################################
        # for j in range(-size, size):                            #   #
        #     if j%2==1:                                              #
        #         continue                                            #
        #     x1 = 2 * xSize                                      #   #
        #     x2 = -2 * xSize                                       # #
        #     self.simulator_.add_agent(Vector2(x1,j*(radius*2))) #   #
        #     self.goals_.append(Vector2(x2, -j*(radius*2)))        # #
        ###############################################################



        #####################################################
        # idx = 0                                           #
        # # 这里全是阻挡.                                   #
        # self.simulator_.add_agent(Vector2(0,0))           #
        # self.goals_.append(Vector2(0,0))                  #
        # self.simulator_.agents_[idx].static_ = True       #
        #                                                   #
        # self.simulator_.add_agent(Vector2(0,radius*2))  # #
        # self.goals_.append(Vector2(0,radius*2))         # #
        # idx +=1                                         # #
        # self.simulator_.agents_[idx].static_ = True #     #
        #                                                 # #
        # self.simulator_.add_agent(Vector2(0,-radius*2)) # #
        # self.goals_.append(Vector2(0,-radius*2))        # #
        # idx +=1                                         # #
        # self.simulator_.agents_[idx].static_ = True #     #
        #                                                   #
        # self.simulator_.add_agent(Vector2(0,radius*4))  # #
        # self.goals_.append(Vector2(0,radius*4))        #  #
        # idx +=1                                         # #
        # self.simulator_.agents_[idx].static_ = True #     #
        #                                                 # #
        # self.simulator_.add_agent(Vector2(0,-radius*4)) # #
        # self.goals_.append(Vector2(0,-radius*4))        # #
        # idx +=1                                         # #
        # self.simulator_.agents_[idx].static_ = True #     #
        #                                                   #
        # # 只是用来展示的目标点，没有碰撞                  #
        # self.shows_.append(Vector2(100,50))               #
        # self.shows_.append(Vector2(-50,-50))              #
        #                                                   #
        # self.shows_.append(Vector2(-225,-225))            #
        # self.shows_.append(Vector2(225,225))              #
        #                                                   #
        # self.simulator_.add_agent(Vector2(-50,0))         #
        # self.goals_.append(Vector2(100,50))               #
        #####################################################


    def setup_scenario1(self):
        # Specify the global time step of the simulation.
        self.simulator_.set_time_step(0.5)

        radius = 13

        # Specify the default parameters for agents that are subsequently added.
        self.simulator_.set_agent_defaults(2*radius, 6, 10, 10.0, radius, radius, Vector2(18.0, 8.0))

        # Add agents, specifying their start position, and store their goals on the opposite side of the environment.
        ##############################################################################################
        # for i in range(250):                                                                       #
        #     self.simulator_.add_agent(200.0 *                                                      #
        #         Vector2(math.cos(i * 2.0 * math.pi / 250.0), math.sin(i * 2.0 * math.pi / 250.0))) #
        #     self.goals_.append(-self.simulator_.agents_[i].position_)                              #
        ##############################################################################################

        ss = True
        obs = False
        idx = 0

        if obs:
            ####################################################################
            # block1 = CircleBlock(Vector2(0,0),radius).to_polygon()           #
            # self.simulator_.add_obstacle(block1)                             #
            # self.obstacles_.append(block1)                                   #
            #                                                                  #
            # block1 = CircleBlock(Vector2(0,radius*2),radius).to_polygon()  # #
            # self.simulator_.add_obstacle(block1)                           # #
            # self.obstacles_.append(block1)                                 # #
            #                                                                # #
            # block1 = CircleBlock(Vector2(0,-radius*2),radius).to_polygon() # #
            # self.simulator_.add_obstacle(block1)                           # #
            # self.obstacles_.append(block1)                                 # #
            #                                                                  #
            # block1 = CircleBlock(Vector2(0,radius*4),radius).to_polygon()    #
            # self.simulator_.add_obstacle(block1)                             #
            # self.obstacles_.append(block1)                                   #
            #                                                                  #
            # block1 = CircleBlock(Vector2(0,-radius*4),radius).to_polygon()   #
            # self.simulator_.add_obstacle(block1)                             #
            # self.obstacles_.append(block1)                                   #
            ####################################################################


            block1 = [Vector2(-radius/2, -radius*4) , Vector2(-radius/2, radius*4) ,Vector2(radius/2, radius*4), Vector2(radius/2, -radius*4)  ]
            block1.reverse()
            self.simulator_.add_obstacle(block1)                             #
            self.obstacles_.append(block1)                                   #

            self.simulator_.process_obstacles()

        else:

            # 这里全是阻挡.
            self.simulator_.add_agent(Vector2(0,0))
            self.goals_.append(Vector2(0,0))
            if ss:
                self.simulator_.agents_[idx].static_ = True

            self.simulator_.add_agent(Vector2(0,radius*2))  #
            self.goals_.append(Vector2(0,radius*2))         #
            idx +=1                                         #
            if ss:                                          #
                self.simulator_.agents_[idx].static_ = True #
                                                            #
            self.simulator_.add_agent(Vector2(0,-radius*2)) #
            self.goals_.append(Vector2(0,-radius*2))        #
            idx +=1                                         #
            if ss:                                          #
                self.simulator_.agents_[idx].static_ = True #

            self.simulator_.add_agent(Vector2(0,radius*4))  #
            self.goals_.append(Vector2(0,radius*4))        #
            idx +=1                                         #
            if ss:                                          #
                self.simulator_.agents_[idx].static_ = True #
                                                            #
            self.simulator_.add_agent(Vector2(0,-radius*4)) #
            self.goals_.append(Vector2(0,-radius*4))        #
            idx +=1                                         #
            if ss:                                          #
                self.simulator_.agents_[idx].static_ = True #



        # 目标点标识.
        #####################################################
        # self.simulator_.add_agent(Vector2(100,50))        #
        # self.goals_.append(Vector2(100,50))               #
        # idx += 1                                          #
        # if ss:                                          # #
        #     self.simulator_.agents_[idx].static_ = True # #
        #                                                   #
        # self.simulator_.add_agent(Vector2(-50,-50))       #
        # self.goals_.append(Vector2(-50,-50))              #
        # idx += 1                                          #
        # if ss:                                          # #
        #     self.simulator_.agents_[idx].static_ = True # #
        #####################################################

        # 只是用来展示的目标点，没有碰撞
        self.shows_.append(Vector2(100,50))
        self.shows_.append(Vector2(-50,-50))


        self.simulator_.add_agent(Vector2(-50,0))
        self.goals_.append(Vector2(100,50))

        ############################################
        # self.simulator_.add_agent(Vector2(50,0)) #
        # self.goals_.append(Vector2(-50,-50))     #
        ############################################

        ##########################################################
        # block1 = CircleBlock(Vector2(0,0),radius).to_polygon() #
        # #print(block1)                                         #
        # self.simulator_.add_obstacle(block1)                   #
        # self.obstacles_.append(block1)                         #
        ##########################################################


    # 逆时针旋转角度。
    def changeAngle(self, position, goal, tha1):
        v1 = goal - position
        value = abs(v1)
        print(value)
        cos1 = v1.x / value
        sin1 = v1.y / value

        tha = tha1/180* math.pi

        cos2 = math.cos(tha)
        sin2 = math.sin(tha)

        cos3 = cos1*cos2 - sin1*sin2
        sin3 = sin1*cos2 + cos1*sin2

        newX = value*cos3
        newY = value*sin3
        return Vector2(newX,newY)

    def update_visualization(self, viewer):
        if not RVO_RENDER:
            return

        # Render the current position of all the agents.
        for i in range(self.simulator_.num_agents):
            goal = self.goals_[i]
            agent1 = self.simulator_.agents_[i]
            position = self.simulator_.agents_[i].position_
            pre_position = self.simulator_.agents_[i].pre_position_
            velocity = self.simulator_.agents_[i].velocity_
            static = self.simulator_.agents_[i].static_
            color = [0, 0, 0]
            if not agent1.static_:
                color[i % 3] = 1
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))
            nearGoal = abs(goal- position)
            radius = self.simulator_.default_agent_.radius_
            movepos = abs(position - pre_position)
            print(i, "velocity", velocity, "abs velocity", abs(velocity),
                  "movedis", movepos, "position", position)


            randomGoal = False
            if agent1.static_:
                randomGoal = False
            elif (nearGoal > 2*radius) and (abs(velocity) <=0.1):
                randomGoal = True
            elif  (nearGoal > 2*radius) and (agent1.check_preposition_ and movepos<1): #
                randomGoal = True                                                       #
                #exit()
            else:
                randomGoal = False

            #randomGoal= False
            #if nearGoal > 2*radius and (abs(velocity) <=0.1 or (agent1.check_preposition_ and movepos<0.7) )  and (not static):
            if randomGoal:
                self.simulator_.agents_[i].randomGoalTick_ =240
                # 选一个偏移方向. 45-135  225- 315
                #

                randomTha1 = 0
                span = 45
                rspan = 90
                if agent1.keepDirTick_> 0:
                    agent1.keepDirTick_-=1
                    if agent1.keepDir_==0:
                        randomTha1 = random.randint(360-span-rspan,360-span)
                    else:
                        randomTha1 = random.randint(span,span+rspan)
                else:
                    # 这里需要强制转向
                    agent1.keepDirTick_=5
                    #if goal.y > position.y:
                    if random.randint(0,1) ==0:
                        print("goal up")
                        randomTha1 = random.randint(360-span-rspan,360-span)
                        agent1.keepDir_ = 0
                    else:
                        randomTha1 = random.randint(span,span+rspan)
                        agent1.keepDir_ = 1
                        print("goal down")


                newGoal = self.changeAngle(position, goal, randomTha1)
                print(i, "random reset randomTha", randomTha1, "pos", position, "goal", goal, "newGoal", newGoal)
                self.simulator_.agents_[i].randomGoal_ = newGoal
                ##############################################################################
                # self.simulator_.agents_[i].randomGoal_ = Vector2(random.randint(-100,100), #
                #                                                  random.randint(-100,100)) #
                ##############################################################################
                # 根据目标点位置，随机偏移一定的角度。

            if agent1.randomGoalTick_>0:
                circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
                circle.add_attr(rendering.Transform(translation=(agent1.randomGoal_.x, agent1.randomGoal_.y)))


        print("\n")

        for obstacle in self.obstacles_:
            v = [(vec.x, vec.y) for vec in obstacle]
            viewer.draw_polygon(v=v, color=(0, 0, 0))

        # 展示目标点.
        for show in self.shows_:
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(show.x, show.y)))

        i = 0
        for goal in self.goals_:
            color = [0, 0, 0]
            color[i % 3] = 1
            if self.simulator_.agents_[i].static_:
                i+=1
                continue
            circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(goal.x, goal.y)))
            i+=1

        viewer.render()

    def set_preferred_velocities(self):
        """
        Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        """

        self.count_+=1
        for i in range(self.simulator_.num_agents):
            goal = self.goals_[i]
            agent = self.simulator_.agents_[i]

            if self.count_ %400 ==0:
                agent.check_preposition_ = True
            elif self.count_ %425 ==200:
                agent.pre_position_ = agent.position_
            else:
                agent.check_preposition_ = False

            if agent.randomGoalTick_ >0:
                agent.randomGoalTick_-=1
                goal = agent.randomGoal_
                print("random ", i, "goal: ", goal, agent.randomGoalTick_)

            goal_vector = goal - self.simulator_.agents_[i].position_

            if rvo_math.abs_sq(goal_vector) > 1.0:
                goal_vector = rvo_math.normalize(goal_vector)

            self.simulator_.set_agent_pref_velocity(i, goal_vector)

    def reached_goal(self):
        """
        Check if all agents have reached their goals.
        """
        for i in range(self.simulator_.num_agents):
            if rvo_math.abs_sq(self.simulator_.agents_[i].position_ - self.goals_[i]) > self.simulator_.agents_[i].radius_ * self.simulator_.agents_[i].radius_:
                print(i, "not reach")
                return False

        return True


def main():
    viewer = None

    circle = Circle()
    goal1 = circle.changeAngle(Vector2(0,0), Vector2(10,10), 270)
    print(goal1)
    #exit()

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
