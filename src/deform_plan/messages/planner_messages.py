from typing import List
from deform_plan.messages.sim_node import SimNode

class PlannerResponse:
    def __init__(self):
        self.checkpoints = [] # type: List[SimNode]
        self.reached_goal = False

    def link_checkpoints(self):
        for i in range(len(self.checkpoints)):
            if i ==0:
                self.checkpoints[i].previous_node = self.checkpoints[i].replayer.parent
            else:
                self.checkpoints[i].previous_node = self.checkpoints[i-1]