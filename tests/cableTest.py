from tests.TestTemplate import TestTemplate
from src.helpers.objectLibrary import Obstacle, RandomBlock
from src.helpers.cables import MultibodyCable,HardJointCable
from src.controls.keyControls import KeyControls
from src.helpers.goalspecifier import GoalSpecifier
import time
import pygame
import pymunk
from pymunk import pygame_util
import random


#simple playground to test movement of cable between static obstacles
#and catch possible tearing collisions
#so far not with RRT
#but with goal region specified

#will need redefinition of start and goal
BLOCK_RADIUS = 150
BLOCK_ROWS = 3
BLOCK_COLUMNS = 4
SEED_SEQ_INIT = 20
CABLE_LENGTH = 400
CABLE_SEGMENTS = 60
# CABLE_SEGMENTS = 1
OBSTALCES = True
MOVING_FORCE = 1000

class CableTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .1
        self.prev_vel = 0

    def setup(self):
        self.draw_constraints = False
        random.seed(SEED_SEQ_INIT)
        blocks = []
        #add obstacles
        for i in range(BLOCK_COLUMNS):
            for j in range(BLOCK_ROWS):
                block = RandomBlock(100 + 200 * i, 200 + 200 * j, BLOCK_RADIUS,random.randint(0,1000))
                if OBSTALCES:
                    block.add(self.space)
                blocks.append(block)

        end_platform = Obstacle(400, 800, 800, 100)
        end_platform.add(self.space)

        #goal
        self.goal = GoalSpecifier(440, 500, 300,200, self.space, CABLE_SEGMENTS)

        #cable
        self.cable = MultibodyCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, MultibodyCable.standardParams, thickness=5)
        self.cable.add(self.space)
        self.kk = KeyControls(self.space,self.cable.segments,MOVING_FORCE,self.display)


    def pre_render(self):
        # self.cable.segments_shapes[self.kk.current].color = (0, 0, 255, 255)
        self.kk.solve_keys(self.keys,self.keydowns,self.click)
        # self.cable.segments_shapes[self.kk.current].color = (255, 0, 0, 255)
        # dv = (self.cable.segments[self.kk.current].velocity[0] - self.prev_vel[0],
        #       self.cable.segments[self.kk.current].velocity[1] - self.prev_vel[1])
        # self.prev_vel = self.cable.segments[self.kk.current].velocity
        # print((dv[0]**2+dv[1]**2)**0.5*self.FPS)
        cur = self.kk.objects[self.kk.current]

        if cur.force.length > 1000:
            seg__vel_sum = sum([s.velocity.length for s in self.kk.objects],0)
            if seg__vel_sum <100 and seg__vel_sum < self.prev_vel:
                print("blocked", seg__vel_sum)
            self.prev_vel = seg__vel_sum

    def post_render(self):
        pass


if __name__ == "__main__":
    scene = CableTest()
    scene.run()
