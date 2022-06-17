"""
Example file showing a demo with 250 agents initially positioned evenly distributed on a circle attempting to move to the antipodal position on the circle.
"""
import math
import gym.envs.classic_control.rendering as rendering
import random
import rvo.math as rvo_math
import time

from rvo.vector import Vector2
from rvo.simulator import Simulator

import astar

RVO_RENDER = True
opt_speed = 3

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

class Block():
    def __init__(self, position):
        self.position_ = position
        self.static_ = False


class BlockFinder(astar.AStar):
    def __init__(self, maze):
        self.maze_ = maze

    # 两个格子之间的距离
    def heuristic_cost_estimate(self, current, goal):
        cX, cY = current[0], current[1]
        gX, gY = goal[0], goal[1]
        return abs(Vector2(gX-cX, gY-cY))

    def distance_between(self, n1, n2):
        cX, cY = n1[0], n1[1]
        gX, gY = n2[0], n2[1]
        return abs(Vector2(gX-cX, gY-cY))

    def neighbors(self, node):
        x, y = node[0], node[1]
        ret = []
        #for i, dd in  enumerate([(-1,1), (-1,0), (-1,-1), (0,1), (0,-1),(1,1), (1,0), (1,-1)]):
        nex = [ (-1,0),  (0,1), (0,-1), (1,0)]
        random.shuffle(nex)
        for i, dd in enumerate(nex):
            nearbyKey = (x+dd[0], y+dd[1])
            if nearbyKey in self.maze_:
                nearbyBody = self.maze_[nearbyKey]
                if not nearbyBody.static_:
                    ret.append(nearbyKey)
        return ret


class Circle:
    def __init__(self):
        # Store the goals of the agents.
        self.blockFinder_ = BlockFinder({})
        self.goals_ = []
        self.simulator_ = Simulator()
        self.obstacles_ = [] # Vector2
        self.shows_ = [] # Vector2
        self.count_ = 0
        self.infradius_ = 8
        self.infradius_max_ = 12
        self.gridSize_ = 20
        self.width_ = int(225/self.gridSize_)
        self.height_ = int(225/self.gridSize_)

    def keyToPos(self, key):
        x = key[0]
        y = key[1]
        return Vector2(x*self.gridSize_ , y*self.gridSize_)

    def posToKey(self, pos):
        return (int(pos.x/self.gridSize_), int(pos.y/self.gridSize_))

    def make_maze(self):
        print("make_maze")
        maze = {}
        for i in range(-self.width_,self.width_):
            for j in range(-self.height_,self.height_):
                pos = (i,j)
                maze[pos] = Block(pos)

        for i in range(self.simulator_.num_agents):
            agent1 = self.simulator_.agents_[i]
            print(agent1.static_)
            if agent1.static_:
                posKey = self.posToKey(agent1.position_)
                if posKey in maze:
                    #print(posKey, "is block")
                    posBody = maze[posKey]
                    posBody.static_ = True
                #continue
                x, y = posKey[0], posKey[1]
                if agent1.radius_*2 > self.gridSize_:
                    for i, dd in  enumerate([(-1,1), (-1,0), (-1,-1), (0,1), (0,-1),(1,1), (1,0), (1,-1)]):
                        nearbyKey = (x+dd[0], y+dd[1])
                        print(nearbyKey , "set static")
                        if nearbyKey in maze:
                            posBody = maze[nearbyKey]
                            posBody.static_ = True
                        else:
                            bl = Block(nearbyKey)
                            bl.static_ = True
                            maze[nearbyKey] = bl



        #exit()
        self.blockFinder_ = BlockFinder(maze)


    # 随机点，有随机的停止点.
    def setup_scenario(self):
        # Specify the global time step of the simulation.
        # Specify the default parameters for agents that are subsequently added.
        radius = self.infradius_
        radius_max = self.infradius_max_
        self.simulator_.set_agent_defaults(3, 6, 10, 10,  10, 3.5, Vector2(18.0, 8.0), radius_max)
        width = 225

        size = int(width/(radius*4))
        #xSize = width/3
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
        # bridge


        print("bridge ...")
        xscale =2
        yscale =1
        yshift = -10
        xshift = 0
        #obstacle1 = [ Vector2(40+xshift,-10*yscale+yshift), Vector2(-20*xscale+xshift,-10*yscale+yshift),
        #              Vector2(-40*xscale+xshift,-20*yscale+yshift), Vector2(40*xscale+xshift,-20*yscale+yshift) ]

        obstacle1 = [Vector2(40,-20), Vector2(-40,-20),Vector2(-80,-180),Vector2(80,-180) ]
        self.simulator_.add_obstacle(obstacle1)
        self.obstacles_.append(obstacle1)


        #cuo
        xscale =2
        yscale =1
        yshift = 10
        xshift = 0
        #obstacle1 = [ Vector2(20*xscale+xshift,10*yscale+yshift), Vector2(40*xscale+xshift,20*yscale+yshift),
         #             Vector2(-40*xscale+xshift,20*yscale+yshift), Vector2(-20*xscale+xshift,10*yscale+yshift) ]

        obstacle1 = [Vector2(40,20), Vector2(80,180),Vector2(-80,180),Vector2(-40,20) ]
        self.simulator_.add_obstacle(obstacle1)
        self.obstacles_.append(obstacle1)

        #self.simulator_.add_obstacle(obstacle1)                                           #
        #self.obstacles_.append(obstacle1)                                                 #


        self.simulator_.process_obstacles()


        # 网格阻挡
        '''
        for j in range(-size, size):                            #
            continue
            x1 = -0                                       #
            if random.randint(0,100)<70:
                #if j%2==0:
                 #   continue
                idx = self.simulator_.add_agent( self.keyToPos((x1, j)))
                self.goals_.append(self.keyToPos((x1, j)))
                self.simulator_.agents_[idx].static_= True

            continue
            #随机在起左右边生成阻挡.                                                #
            xx = random.randint(0,7)                                                # #
            yy = random.randint(0,7)

            ################
            # if xx ==2:   #
            #     yy = 3   #
            # elif xx ==3: #
            #     yy=2     #
            # else:        #
            #     fwefwf=0 #
            ################
            #xx = -1
            #yy = 3

            if xx==0:                                                               # #
                idx = self.simulator_.add_agent( self.keyToPos((x1-1, j)) )  # #
                self.goals_.append(self.keyToPos((x1-1, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif xx == 1:                                                           # #
                idx = self.simulator_.add_agent( self.keyToPos((x1+1, j)) )  # #
                self.goals_.append(self.keyToPos((x1-1, j)))        #     # #
                self.simulator_.agents_[idx].static_= True
            elif xx == 2:                                                           # #
                idx = self.simulator_.add_agent( self.keyToPos((x1-2, j)) )  # #
                self.goals_.append(self.keyToPos((x1-2, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif xx ==3:
                idx = self.simulator_.add_agent( self.keyToPos((x1+2, j)) )  # #
                self.goals_.append(self.keyToPos((x1-2, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            else:                                                                   # #
                fxfwef=0                                                                 # #

            #continue
            if yy==0:                                                               # #
                idx = self.simulator_.add_agent( self.keyToPos((x1-3, j)) )  # #
                self.goals_.append(self.keyToPos((x1-3, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif yy == 1:                                                           # #
                idx = self.simulator_.add_agent( self.keyToPos((x1+3, j)) )  # #
                self.goals_.append(self.keyToPos((x1-3, j)))        #     # #
                self.simulator_.agents_[idx].static_= True
            elif yy == 2:                                                           # #
                idx = self.simulator_.add_agent( self.keyToPos((x1-4, j)) )  # #
                self.goals_.append(self.keyToPos((x1-4, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            elif yy ==3:
                idx = self.simulator_.add_agent( self.keyToPos((x1+4, j)) )  # #
                self.goals_.append(self.keyToPos((x1-4, j)))        #     # #
                self.simulator_.agents_[idx].static_= True                          # #
            else:                                                                   # #
                fxfwef=0                                                                 # #
        '''

        #生成移动点.
        x =0
        c =0
        x = random.randint(-size, size)
        x = 1
        xSize=3


        for j in range( -int(size), int(size)):                             #
            if c>24:
                continue
            #if random.randint(0,1)==0:
            #    continue
            if j%2==0:
                continue

            x1 = -3 * xSize                                      #
            x2 = 3 * xSize                                       #

            #if c == 2:
            #    c+=1
                #continue

            #x = 5
            '''
            if random.randint(0,2) == 3:
                self.simulator_.add_agent(self.keyToPos((x1,x+j)))  #
                self.goals_.append(self.keyToPos( (x2, -(x+j)) ))       #
            else:
                self.simulator_.add_agent(self.keyToPos((x1,x+j)),10)  #
                self.goals_.append(self.keyToPos( (x2, -(x+j)) ))       #
            '''

            #if random.randint(0,1) == 0:
            if c%2==0:
                self.simulator_.add_agent(self.keyToPos((x1,x+j)),9.8)  #     #
                self.goals_.append(self.keyToPos( (x2, -(x+j)) ))       # #
            else:                                                         #
                self.simulator_.add_agent(self.keyToPos((-x1,x+j)), 9.8)   #
                self.goals_.append(self.keyToPos( (-x2, -(x+j)) ))        #
            c+=1
            #break

        self.shows_.append(Vector2(-20,20))              #
        self.make_maze()

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

        goalPlan = random.randint(1,200)
        if goalPlan<150:
            goalPlan = 2
        else:
            goalPlan = 2

        #goalPlan = 3

        # Render the current position of all the agents.
        size = self.simulator_.num_agents
        print("num_agents", size)
        for i in range(size):
            #print(i)
            goal = self.goals_[i]
            agent1 = self.simulator_.agents_[i]
            position = self.simulator_.agents_[i].position_
            pre_position = self.simulator_.agents_[i].pre_position_
            x_position = self.simulator_.agents_[i].x_position_
            velocity = self.simulator_.agents_[i].velocity_
            static = self.simulator_.agents_[i].static_
            color = [0, 0, 0]
            if not agent1.static_:
                color[i % 3] = 1
            circle = viewer.draw_circle(radius=agent1.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(position.x, position.y)))
            nearGoal = abs(goal- position)
            radius = agent1.radius_
            movepos = abs(position - pre_position)
            xmovepos = abs(position - x_position)
            print(i, "velocity", velocity, "abs velocity", abs(velocity),
                  "movedis", movepos, "xmovepos", xmovepos, "position", position, "goal", goal, "randgoal", agent1.randomGoal_, "static", static, "radius", agent1.radius_)


            randomGoal = False
            if agent1.static_:
                randomGoal = False
            elif (nearGoal > 2*radius) and (abs(velocity) <=0.3):
                randomGoal = True
            elif  (nearGoal > 2*radius) and (agent1.check_preposition_ and movepos<2): #
                randomGoal = True
                print(i, "distance random")
                #exit()
            else:
                if random.randint(0,100) < 1 and agent1.randomGoalTick_<=0:
                    randomGoal = False
                else:
                    randomGoal = False

            #randomGoal= False
            #if nearGoal > 2*radius and (abs(velocity) <=0.1 or (agent1.check_preposition_ and movepos<0.7) )  and (not static):
            if randomGoal:
                agent1.randomGoalTick_ =40
                # 选一个偏移方向. 45-135  225- 315
                #

                # A*找点.
                if goalPlan==1:
                    ###########################################
                    # curX = int(position.x / self.gridSize_) #
                    # curY = int(position.y / self.gridSize_) #
                    # start = (curX,curY)                     #
                    ###########################################
                    start = self.posToKey(position)
                    curX = start[0]
                    curY = start[1]


                    lls = [ (0,1), (0,-1), (-1,0),(1,0), (-1,1), (-1,-1), (1,1), (1,-1)]
                    #random.shuffle(lls)

                    goalPlan=2

                    distance = 1000000
                    findKey = start
                    findKeyX = False


                    if start in self.blockFinder_.maze_:            #
                        nearbyBody = self.blockFinder_.maze_[start] #
                        # 当前点必须不是阻挡.                       #
                        if not nearbyBody.static_:                  #
                            findKeyX = True
                        else:
                            print("astar is static",start)

                    if not findKeyX:
                        for i, dd in  enumerate(lls):                           #
                            nearbyKey = (curX+dd[0], curY+dd[1])                #
                            if nearbyKey in self.blockFinder_.maze_:            #
                                nearbyBody = self.blockFinder_.maze_[nearbyKey] #
                                if not nearbyBody.static_:                      #
                                    pos = Vector2( nearbyKey[0],nearbyKey[1])   #
                                    dis = abs(position - pos)                   #
                                    if dis < distance:                          #
                                        findKey = nearbyKey                     #
                                        findKeyX = True                         #
                                        distance = dis


                    if findKeyX:
                        key = self.posToKey(goal)
                        goalX= key[0]
                        goalY= key[1]
                        ret = self.blockFinder_.astar(findKey, (goalX,goalY))
                        print("astar find l1",  start , (goalX,goalY))
                        if ret !=  None:
                            ll = [station for station in ret]
                            goldTuple = ll[0]
                            agent1.randomGoal_ = self.keyToPos(goldTuple)#  Vector2(goldTuple[0]*self.gridSize_, goldTuple[1]*self.gridSize_ )
                            agent1.randomGoalList_ =ll[1:]
                            print("astar find",  goldTuple,agent1.randomGoal_ , ll)
                            goalPlan=1
                            #continue
                                    #exit()
                        else:
                            print("astar start is static", start)
                    else:
                        print("astar start not in maze", start)

                    #if goalPlan==2:
                        #goalPlan=1
                        #agent1.randomGoalTick_ =0



                # 随机找点.
                if goalPlan ==2:
                    #agent1.randomGoalTick_ *=0.5
                    print("try random")
                    randomTha1 = 0
                    span = 30
                    rspan = 120
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
                    agent1.randomGoal_ = newGoal
                    ##############################################################################
                    # agent1.randomGoal_ = Vector2(random.randint(-100,100), #
                    #                                                  random.randint(-100,100)) #
                    ##############################################################################
                    # 根据目标点位置，随机偏移一定的角度。

                if goalPlan ==3 :
                    agent1.randomGoal_ = goal

            if agent1.randomGoalTick_>0:
                circle = viewer.draw_circle(radius=radius/2, color=color)
                circle.add_attr(rendering.Transform(translation=(agent1.randomGoal_.x, agent1.randomGoal_.y)))


        print("\n")

        for obstacle in self.obstacles_:
            v = [(vec.x, vec.y) for vec in obstacle]
            print("draw obstacle\n", v)
            viewer.draw_polygon(v=v, color=(0, 0, 0))



        i = 0
        for goal in self.goals_:
            color = [0, 0, 0]
            color[i % 3] = 1
            agent = self.simulator_.agents_[i]
            if self.simulator_.agents_[i].static_:
                i+=1
                continue
            circle = viewer.draw_circle(radius=agent.radius_, color=color)
            circle.add_attr(rendering.Transform(translation=(goal.x, goal.y)))
            i+=1

        # 展示目标点.
        #for show in self.shows_:
        #    circle = viewer.draw_circle(radius=self.simulator_.default_agent_.radius_, color=[1,1,0])
        #    circle.add_attr(rendering.Transform(translation=(show.x, show.y)))

        viewer.render()

    def set_preferred_velocities(self):
        """
        Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        """

        self.count_+=1
        for i in range(self.simulator_.num_agents):
            goal = self.goals_[i]
            agent = self.simulator_.agents_[i]
            xxy = 5


            if self.count_ %xxy ==0:
                agent.check_preposition_ = True
            elif self.count_ %xxy ==xxy/2:
                agent.pre_position_ = agent.position_
            else:
                agent.check_preposition_ = False

            if agent.randomGoalTick_ >0:
                agent.randomGoalTick_-=1
                if rvo_math.abs_sq(agent.position_ - agent.randomGoal_ ) < agent.radius_ * agent.radius_:
                    if len(agent.randomGoalList_) >0:
                        xx = agent.randomGoalList_[0]
                        print("random pop", i, "goal: ", xx)
                        agent.randomGoal_ =  self.keyToPos(xx) #  Vector2(xx[0]*self.gridSize_,xx[1]*self.gridSize_)
                        agent.randomGoalList_=agent.randomGoalList_[1:]
                        goal = agent.randomGoal_
                    else:
                        agent.randomGoalList_=[]
                        agent.randomGoal_= Vector2()
                        agent.randomGoalTick_= 0
                else:
                    goal = agent.randomGoal_
                print("random set_preferred_velocities", i, "goal: ", goal, agent.randomGoalTick_)

                ''' # 随机便宜
                if abs(self.simulator_.agents_[i].position_ -  goal)>0.3:
                    if random.randint(0,1) == 0:
                        goal = self.changeAngle(self.simulator_.agents_[i].position_,  goal, random.randint(1,30))
                    else:
                        goal = self.changeAngle(self.simulator_.agents_[i].position_,  goal, random.randint(330,360))
                '''


            goal_vector = goal - self.simulator_.agents_[i].position_
            if rvo_math.abs_sq(goal_vector) > 1.0:
                goal_vector = rvo_math.normalize(goal_vector)

            self.simulator_.set_agent_pref_velocity(i, goal_vector)

    def reached_goal(self):
        """
        Check if all agents have reached their goals.
        """
        for i in range(self.simulator_.num_agents):
            '''
            agent = self.simulator_.agents_[i]
            if rvo_math.abs_sq(agent.position_ - agent.randomGoal_ ) < agent.radius_ * agent.radius_:
                if len(agent.randomGoalList_) >0:
                    xx = agent.randomGoalList_[0]
                    print("random pop", i, "goal: ", xx)
                    agent.randomGoal_ = Vector2(xx[0]*self.gridSize_,xx[1]*self.gridSize_)
                    agent.randomGoalList_=agent.randomGoalList_[1:]
                else:
                    agent.randomGoalList_=[]
                    agent.randomGoal_= Vector2()
                    agent.randomGoalTick_= 0
            '''
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

    #fps = 15
    # Set up the scenario.
    circle.setup_scenario()
    step = 0.1
    circle.simulator_.set_time_step(4)

    # Perform (and manipulate) the simulation.
    while not circle.reached_goal():
        print("-----",time.ctime())
        if RVO_RENDER:
            if viewer is None:
                viewer = rendering.Viewer(750, 750)
                viewer.set_bounds(-225, 225, -225, 225)

        circle.update_visualization(viewer)
        circle.set_preferred_velocities()
        circle.simulator_.step()
        time.sleep(0.3)


if __name__ == '__main__':
    main()