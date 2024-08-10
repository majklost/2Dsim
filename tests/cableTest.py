from tests.TestTemplate import TestTemplate
from src.helpers.objectLibrary import Obstacle, RandomBlock
from src.helpers.cables import MultibodyCable,HardJointCable
from src.controls.keyControls import KeyControls
import time
import pygame
import pymunk
from pymunk import pygame_util
import random


#simple playground to test movement of cable between static obstacles
#and catch possible tearing collisions
#so far not with RRT

#will need redefinition of start and goal
BLOCK_RADIUS = 100
BLOCK_ROWS = 3
BLOCK_COLUMNS = 4
SEED_SEQ_INIT = 20
CABLE_LENGTH = 400
CABLE_SEGMENTS = 60
OBSTALCES = True

class CableTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .1
        self.prev_vel = (0,0)

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

        #cable
        springParams = MultibodyCable.SpringParams(5000, 10)
        self.cable = MultibodyCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, springParams, thickness=5)
        #
        # self.cable = HardJointCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, springParams)
        self.cable.add(self.space)
        self.kk = KeyControls(self.space,self.cable.segments)
        self.prev_vel = self.cable.segments[self.kk.current].velocity



    def pre_render(self):
        self.cable.segments_shapes[self.kk.current].color = (0, 0, 255, 255)
        self.kk.solve_keys(self.keys,self.keydowns)
        self.cable.segments_shapes[self.kk.current].color = (255, 0, 0, 255)
        dv = (self.cable.segments[self.kk.current].velocity[0] - self.prev_vel[0],
              self.cable.segments[self.kk.current].velocity[1] - self.prev_vel[1])
        self.prev_vel = self.cable.segments[self.kk.current].velocity
        # print((dv[0]**2+dv[1]**2)**0.5*self.FPS)

    def post_render(self):
        pass


if __name__ == "__main__":
    scene = CableTest()
    scene.run()
