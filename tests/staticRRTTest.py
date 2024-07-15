# Try RRT on some easy environment
import pygame
# import pymunk
# import pymunk.pygame_util
from staticLocalPlanner import LocalPlanner
from RRT import RRT
from RRTNode import RRTNode
from tree_rendering import TreeRenderer
import math
import time
from path_mover import PathMover
from tests.TestTemplate import TestTemplate
from helpers.objectLibrary import Agent, Obstacle
from helpers.helperFunctions import render_goal


class StaticRRTTest(TestTemplate):
    def __init__(self, start:RRTNode, goal:RRTNode):
        self.start = start
        self.goal = goal
        self.path_mover = None
        self.tree_renderer = None
        super().__init__(800, 800, 80)

    def setup(self):
        # objects
        agent = Agent(self.start.x, self.start.y)
        agent.add(self.space)

        block = Obstacle(450,450)
        block.add(self.space)

        block2 = Obstacle(350, 600)
        block2.add(self.space)

        #collision handlers
        handler = self.space.add_collision_handler(1, 2) #using collision types from imported objects
        handler.begin = test_begin

        #planning
        lp = LocalPlanner(self.space, agent.shape)
        rrt = RRT(self.display.get_width(), self.display.get_height(), math.pi*2, lp)
        start = LocalPlanner.node_from_shape(agent.shape)
        st = time.time()
        path = rrt.find_path(start, self.goal)
        print("Time taken: ", time.time()-st)
        verts = sorted(list(rrt.get_verts()), key=lambda x: x.added_cnt)
        self.path_mover = PathMover(path, agent.body, self.FPS)
        print(len(verts))
        self.tree_renderer = TreeRenderer(verts)

    def pre_render(self):
        self.tree_renderer.render(self.display, pygame.time.get_ticks())
        self.tree_renderer.render_path(self.display)

    def post_render(self):
        render_goal(self.display, self.goal)
        self.path_mover.move()


def test_begin(arbiter, space, data):
    print("Collision")
    return False

if __name__ == '__main__':
    GOAL = RRTNode(400, 50, math.pi * 2)
    t = StaticRRTTest(RRTNode(50, 750, 0), GOAL)
    t.run()
