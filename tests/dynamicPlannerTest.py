from tests.TestTemplate import TestTemplate
from helpers.objectLibrary import Agent, Obstacle
from helpers.helperFunctions import render_goal
W=H=800
from RRTNode import RRTNodeCalc
import pymunk
from dynamicLocalPlanner import LocalPlannerCalc

import pygame
from pymunk import pygame_util


OBSTACLE_VELOCITY = 100,0,0
# OBSTACLE_VELOCITY = 5,0,0


class DynamicPlannerTest(TestTemplate):

    def __init__(self,start,goal):
        self.start = start
        self.goal = goal
        self.cnt = 0
        super().__init__(W,H,80)

    def debugclbck(self,space):
        if self.cnt > 100:
            return
        draw_options = pymunk.pygame_util.DrawOptions(self.display)
        self.display.fill((255, 255, 255))
        space.debug_draw(draw_options)
        pygame.display.update()
        #save the image
        pygame.image.save(self.display, f"./debug/screenshot{self.cnt}.jpg")
        self.cnt += 1

    def setup(self):
        #objects
        agent = Agent(self.start.x,self.start.y,50,50)
        agent.add(self.space)

        #the one that should be plausible to reach via line
        block = Obstacle(100,400,200,100)
        block.set_body_type(pymunk.Body.KINEMATIC)
        block.body.velocity = OBSTACLE_VELOCITY[0],OBSTACLE_VELOCITY[1]
        block.body.angular_velocity = OBSTACLE_VELOCITY[2]
        block.add(self.space)

        #the one that should be blocked
        block2 = Obstacle(400,700,100,200)
        block2.add(self.space)

        #planner
        lp = LocalPlannerCalc(self.space, agent.shape,block.body, obstacle_velocity=OBSTACLE_VELOCITY,dt=1/100)
        lp.verbose = True
        # lp.set_debug_callback(self.debugclbck)
        chpoints = lp.check_path(self.start, self.goal)
        print(chpoints)


    def post_render(self):
        render_goal(self.display, self.goal)







if __name__ == "__main__":
    START = RRTNodeCalc(50, 750, 0,0)
    GOAL = RRTNodeCalc(50, 50, 0,5)
    GOAL2 = RRTNodeCalc(700, 700, 0,5)
    t = DynamicPlannerTest(START, GOAL)
    t.run()