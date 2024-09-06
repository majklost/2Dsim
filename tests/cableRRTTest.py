import random

import pymunk

from tests.TestTemplate import TestTemplate
from src.helpers.objectLibrary import Obstacle, RandomBlock
from src.helpers.cables import MultibodyCable
from src.helpers.goalspecifier import GoalSpecifier
from src.cable_planner import CablePlanner
from src.bezier_sampler import Sampler
from src.RRTNode import RRTNodeCable
from src.cableRRT import RRT


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
INDEXES = [0]

class CableRRTTest:
    def __init__(self):
        self.space = pymunk.Space()
        self.space.damping = .1
        self.prev_vel = 0

    def setup(self):
        random.seed(SEED_SEQ_INIT)
        blocks = []
        #add obstacles
        for i in range(BLOCK_COLUMNS):
            for j in range(BLOCK_ROWS):
                block = RandomBlock(100 + 200 * i, 200 + 200 * j, BLOCK_RADIUS,random.randint(0,1000))
                if OBSTALCES:
                    block.add(self.space)
                blocks.append(block)

        end_platform_bot = Obstacle(400, 800, 800, 20)
        end_platform_bot.add(self.space)

        end_platform_top = Obstacle(400, 0, 800, 20)
        end_platform_top.add(self.space)

        end_platform_left = Obstacle(0, 400, 20, 800)
        end_platform_left.add(self.space)

        end_platform_right = Obstacle(800, 400, 20, 800)
        end_platform_right.add(self.space)

        # goal
        self.goal = GoalSpecifier(440, 500, 300, 200, self.space, CABLE_SEGMENTS)

        # cable
        self.cable = MultibodyCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, MultibodyCable.standardParams, thickness=5)
        self.cable.add(self.space)
        for i,s in enumerate(self.cable.segments):
            s.movedID = i

        for i in INDEXES:
            self.cable.segments[i].controlledID = i



        #planner
        self.planner = CablePlanner(MOVING_FORCE, verbose=False, rendered=False, auto_stop=True)

        #sampler constraints
        sc = Sampler.SamplingConstraints(0, 800, 0, 800)

        #RRT
        self.rrt = RRT(self.planner,self.goal,sc,CABLE_LENGTH,CABLE_SEGMENTS,INDEXES,seed=10)
        START = RRTNodeCable(simSpace=self.space)
        START.fill_points()

        import time
        start_t = time.time()
        self.rrt.find_path(START,iters=60)
        print("Time elapsed: ",time.time()-start_t)
        print("Avg steps: ",self.planner.avg_steps)


if __name__ == "__main__":

    CableRRTTest().setup()