from tests.TestTemplate import TestTemplate
from helpers.objectLibrary import Agent, Obstacle
from helpers.helperFunctions import render_goal
W=H=800
from RRTNode import RRTNodeCalc
import pymunk
from dynamicLocalPlanner import LocalPlannerCalc

class DynamicPlannerTest(TestTemplate):

    def __init__(self,start,goal):
        self.start = start
        self.goal = goal
        super().__init__(W,H,80)

    def setup(self):
        #objects
        agent = Agent(self.start.x,self.start.y,50,50)
        agent.add(self.space)

        #the one that should be plausible to reach via line
        block = Obstacle(100,400,200,100)
        block.set_body_type(pymunk.Body.KINEMATIC)
        block.body.velocity = 50,0
        block.add(self.space)

        #the one that should be blocked
        block2 = Obstacle(400,700,100,200)
        block2.add(self.space)

        #planner
        lp = LocalPlannerCalc(self.space, agent.shape,block.shape, (50,0,0))
        lp.check_path(self.start, self.goal)

    def post_render(self):
        render_goal(self.display, self.goal)







if __name__ == "__main__":
    START = RRTNodeCalc(50, 750, 0,0)
    GOAL = RRTNodeCalc(50, 50, 0,5)
    GOAL2 = RRTNodeCalc(700, 700, 0,5)
    t = DynamicPlannerTest(START, GOAL)
    t.run()