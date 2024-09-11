from .base_rrt import BaseRRT
from ..samplers.ndim_sampler import NDIMSampler
from ..planners.velocity_planner import VelocityPlanner
from ..storage_mws.simple_mw import SimpleMW
from deform_plan.messages.nodes.sim_node import NodeReached,NodeGoal


class OneObjectRRT(BaseRRT):
    def __init__(self, sampler: NDIMSampler, planner: VelocityPlanner,mw: SimpleMW):
        self.sampler = sampler
        self.planner = planner
        self.mw = mw

    def find_path(self,goal: NodeGoal):
        start = self.planner.form_start_node()





