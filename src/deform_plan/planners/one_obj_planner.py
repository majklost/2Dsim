"""Planner for planning one rigid object"""
from deform_plan.planners.base_planner import BasePlanner
from ..messages.one_obj_messages import OneObjPlannerRequest, OneObjPlannerResponse
from ..simulators.PM.pm_simulator import Simulator


class OneObjPlanner(BasePlanner):
    def __init__(self,simulator: Simulator, max_iter_cnt: int = 1000):
        super().__init__(simulator)
        self.simulator = simulator # redeine so that the type hinting works
        self.max_iter_cnt = max_iter_cnt

    def check_path(self, request: OneObjPlannerRequest) -> OneObjPlannerResponse:
        start = request.start
        goal = request.goal
