from tests.TestTemplate import TestTemplate
from src.helpers.cables import MultibodyCable
from src.helpers.goalspecifier import GoalSpecifier
from src.cable_planner import CablePlanner

CABLE_LENGTH = 200
CABLE_SEGMENTS = 30
MOVING_FORCE = 10000
GOAL = 420,500


class CablePlannerTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .1
    def setup(self):
        self.draw_constraints = False
        self.goal = GoalSpecifier(420, 500, 300, 200, self.space, CABLE_SEGMENTS)
        self.cable = MultibodyCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, MultibodyCable.standardParams, thickness=5)
        self.cable.add(self.space)

        for i,s in enumerate(self.cable.segments):
            s.movedID = i
        self.cable.segments[15].controlledID = 0


        #plannerSpec
        self.planner = CablePlanner(self.space,MOVING_FORCE)





if __name__ == "__main__":
    CablePlannerTest().run()