
from tests.TestTemplate import TestTemplate
from helpers.objectLibrary import Agent, Obstacle, Cross
from helpers.helperFunctions import render_goal
from tree_rendering import TreeRenderer
from dynamicRRT import RRT
from dynamicLocalPlanner import LocalPlannerCalc
import time
import pygame
import pymunk
from pymunk import  pygame_util
from path_mover import PathMover

W =H =800
CROSS_ANGULAR_VELOCITY = 0.2
class DynamicRRTTest(TestTemplate):

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.path_mover = None
        self.tree_renderer = None
        super().__init__(W, H, 80)
        self.cnt = 0

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
        # objects
        agent = Agent(self.start.x, self.start.y,20,20)
        agent.add(self.space)

        block1 = Obstacle(300/2,H/2,300,100)
        block1.add(self.space)

        block2 = Obstacle(W-300/2, H/2, 300, 100)
        block2.add(self.space)

        cross = Cross(W/2, H/2, 100, CROSS_ANGULAR_VELOCITY)
        cross.add(self.space)

        #planning
        lp = LocalPlannerCalc(self.space, agent.shape, cross.body, (0,0,CROSS_ANGULAR_VELOCITY))
        # lp.set_debug_callback(self.debugclbck)
        rrt = RRT(self.display.get_width(), self.display.get_height(), 0, 10, lp,near_radius=10,seed=32)
        start = LocalPlannerCalc.node_from_shape(agent.shape)
        st = time.time()
        path = rrt.find_path(start, self.goal,4000)
        # print(path)
        print("Time taken: ", time.time() - st)
        verts = sorted(list(rrt.get_verts()), key=lambda x: x.added_cnt)
        self.path_mover = PathMover(path, agent.body, self.FPS)
        print(len(verts))
        self.tree_renderer = TreeRenderer(verts)

    def pre_render(self):
        self.tree_renderer.render(self.display, pygame.time.get_ticks())
        self.tree_renderer.render_path(self.display)


    def post_render(self):
        render_goal(self.display, self.goal)
        self.path_mover.move(pygame.time.get_ticks()/1000)

if __name__ == "__main__":
    from RRTNode import RRTNodeCalc
    START = RRTNodeCalc(50, 750, 0,0)
    GOAL = RRTNodeCalc(50, 50, 0,100)
    DUMBGOAL = RRTNodeCalc(50,600,0,100)
    t = DynamicRRTTest(START, GOAL)
    t.run()
