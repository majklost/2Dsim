# simple scene for checking if local planner can connect two points by line
# and if it can find that it reached obstacle
# also tries to render the paths and nodes
from z_old_tests.TestTemplate import TestTemplate
from _old_src.helpers.objectLibrary import Agent, Obstacle
from _old_src.helpers.helperFunctions import render_goal
from _old_src.RRTNode import RRTNode

from _old_src.staticLocalPlanner import LocalPlanner


class StaticLocalPlannerTest(TestTemplate):
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        super().__init__(800, 800, 80)

    def setup(self):
        # objects
        agent = Agent(50, 90, 50, 50)
        agent.shape.data = "agent"
        agent.add(self.space)

        obstacle = Obstacle(200, 130, 400, 20)
        obstacle.shape.data = "obstacle"
        obstacle.add(self.space)

        # planner
        lp = LocalPlanner(self.space, agent.shape)
        lp.check_path(self.start, self.goal)

    def post_render(self):
        render_goal(self.display, self.goal)


if __name__ == "__main__":
    GOAL1 = RRTNode(50, 700, 0)  # the blocked one
    GOAL2 = RRTNode(700, 50, 0)  # free one
    START = RRTNode(50, 90, 0)
    test = StaticLocalPlannerTest(START, GOAL2)
    test.run()
