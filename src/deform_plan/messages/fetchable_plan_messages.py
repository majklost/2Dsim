
from .base_planner_messages import BasePlannerRequest, BasePlannerResponse
from ..assets.PM.nodes.sim_node import NodeReached, NodeGoal

class FetchablePlannerRequest(BasePlannerRequest):
    def __init__(self, start:NodeReached, goal:NodeGoal):
        super().__init__(start, goal)








class FetchablePlannerResponse(BasePlannerResponse):
    def __init__(self):
        super().__init__()
        self.checkpoints = []
