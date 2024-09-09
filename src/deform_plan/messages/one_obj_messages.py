from .base_planner_messages import BasePlannerRequest, BasePlannerResponse

class OneObjPlannerRequest(BasePlannerRequest):
    def __init__(self, start, goal):
        super().__init__(start, goal)


class OneObjPlannerResponse(BasePlannerResponse):
    def __init__(self):
        super().__init__()
        self.checkpoints = []
