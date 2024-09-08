from _old_src.RRTNode import RRTNodeCable
from z_old_tests.TestTemplate import TestTemplate
from _old_src.helpers.cables import MultibodyCable
from _old_src.helpers.goalspecifier import GoalSpecifier
from _old_src.cable_planner import CablePlanner
from _old_src.bezier_sampler import Sampler
from _old_src.helpers.PygameRenderer import PygameRenderer

CABLE_LENGTH = 200
CABLE_SEGMENTS = 30
MOVING_FORCE = 1000
GOAL = 420,500


class CablePlannerTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .1
    def setup(self):

        controllable_idx = [i for i in range(0,29,3)]

        self.draw_constraints = False
        # self.goal = GoalSpecifier(420, 500, 300, 200, self.space, CABLE_SEGMENTS)
        self.cable = MultibodyCable(20, 50, CABLE_LENGTH, CABLE_SEGMENTS, MultibodyCable.standardParams, thickness=5)
        for i,s in enumerate(self.cable.segments):
            s.movedID = i
        self.cable.add(self.space)

        c_index = 0
        for i in controllable_idx:
            self.cable.segments[i].controlledID =  c_index
            for s in self.cable.segments[i].shapes:
                s.color = (0,255,0,255)
            c_index += 1





        #plannerSpec
        self.planner = CablePlanner(MOVING_FORCE,verbose=True,rendered=True,auto_stop=False)


        #sampler
        # self.sampler = Sampler(CABLE_LENGTH,CABLE_SEGMENTS,[i for i in range(CABLE_SEGMENTS)], seed=6)
        self.sampler = Sampler(CABLE_LENGTH,CABLE_SEGMENTS,controllable_idx, seed=5)
        sc = Sampler.SamplingConstraints(200,600,200,600,0,2*3.14)
        points = self.sampler.sample(sc)
        print(points)
        all_sampled = self.sampler.last_sampled

        START = RRTNodeCable(simSpace=self.space)
        GOAL =RRTNodeCable(points)

        def draw_shape(d,space):
            for g in all_sampled:
                if g[0] in points[:0]:
                    self.planner.renderer.draw_circle(g,5,(0,255,0))
                else:
                    self.planner.renderer.draw_circle(g,2,(255,0,0))
            for g in points:
                self.planner.renderer.draw_circle(g,5,(0,0,255))


        self.planner.renderer.update_cur_clb = draw_shape

        self.planner.check_path(START,GOAL)








if __name__ == "__main__":
    CablePlannerTest().setup() #run setup only