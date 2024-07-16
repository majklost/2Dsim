
from tests.TestTemplate import TestTemplate
from helpers.objectLibrary import Agent, Obstacle, Cross
from helpers.helperFunctions import render_goal
from tree_rendering import TreeRenderer
from dynamicRRT import RRT
from dynamicLocalPlanner import LocalPlannerCalc
import math
import time
import pygame

W =H =800

class DynamicRRTTest(TestTemplate):

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        super().__init__(W, H, 80)

    def setup(self):
        # objects
        agent = Agent(self.start.x, self.start.y,20,20)
        agent.add(self.space)

        block1 = Obstacle(300/2,H/2,300,100)
        block1.add(self.space)

        block2 = Obstacle(W-300/2, H/2, 300, 100)
        block2.add(self.space)

        cross = Cross(W/2, H/2, 100, 2)
        cross.add(self.space)

        #planning
        lp = LocalPlannerCalc(self.space, agent.shape, block1.shape, (0,0,2))
        rrt = RRT(self.display.get_width(), self.display.get_height(), 2*math.pi, 50, lp)
        start = LocalPlannerCalc.node_from_shape(agent.shape)
        st = time.time()
        path = rrt.find_path(start, self.goal)
        print("Time taken: ", time.time() - st)
        verts = sorted(list(rrt.get_verts()), key=lambda x: x.added_cnt)
        self.tree_renderer = TreeRenderer(verts)
        print(len(verts))
        self.tree_renderer = TreeRenderer(verts)

    def pre_render(self):
        self.tree_renderer.render(self.display, pygame.time.get_ticks())
        self.tree_renderer.render_path(self.display)


    def post_render(self):
        render_goal(self.display, self.goal)


if __name__ == "__main__":
    from RRTNode import RRTNodeCalc
    START = RRTNodeCalc(50, 750, 0,0)
    GOAL = RRTNodeCalc(50, 50, 0,100)
    t = DynamicRRTTest(START, GOAL)
    t.run()
