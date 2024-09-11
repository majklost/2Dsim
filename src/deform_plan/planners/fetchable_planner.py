"""Planner for planning one rigid object"""
import numpy as np
#TODO: rewrite to better use - now just PoC

from deform_plan.planners.base_planner import BasePlanner
from ..messages.fetchable_plan_messages import FetchablePlannerRequest, FetchablePlannerResponse
from ..simulators.PM.pm_simulator import Simulator
from deform_plan.messages.nodes.sim_node import NodeReached, NodeGoal



class FetchAblePlanner(BasePlanner):
    def __init__(self,simulator:Simulator, movable_index:int,
                 max_iter_cnt: int = 1000,
                 only_simuls: bool = False
                 ):
        super().__init__(simulator)

        # self.simulator = simulator
        self.only_simuls = only_simuls
        self.max_iter_cnt = max_iter_cnt
        self.movable_idx = movable_index
        self.after_load_clb = None
        self.iter_cnt = 0





    def check_path(self, request: FetchablePlannerRequest) -> FetchablePlannerResponse:

        start = request.start #type: NodeReached
        goal = request.goal #type: NodeGoal
        response = FetchablePlannerResponse()
        if start.sim_export is None:
            self._fetch_simspace(start)
        cur_iter_cnt = 0

        self.simulator: Simulator #cast the type
        self.simulator.import_from(start.sim_export)
        if self.after_load_clb is not None:
            self.after_load_clb(self.simulator)
        # print(goal.info_vec[-1])

        while self._guide_to_goal(goal):
            self.simulator.step()

            if self._check_end():
                break
            cur_iter_cnt += 1
            if cur_iter_cnt > self.max_iter_cnt:
                break
            if cur_iter_cnt % 20 == 0:
                print("Cur Iter: ", cur_iter_cnt)
            self.iter_cnt = cur_iter_cnt+start.iter_cnt

            response.checkpoints.append(self.create_checkpoint(self.iter_cnt, goal, start))
        print("Final cur Iter: ", cur_iter_cnt)
        response.checkpoints.append(self.create_checkpoint(self.iter_cnt, goal, start))
        return response

    def create_checkpoint(self,iter_cnt:int, goal: NodeGoal,parent:NodeReached) -> NodeReached:
        if self.only_simuls:
            return NodeReached(iter_cnt,sim_export=self.simulator.export())
        else:
            replayer = NodeReached.Replayer(real_goal=goal, parent=parent)
            return NodeReached(iter_cnt,replayer=replayer)

    def _check_end(self):
        x = self.simulator.movable_objects[self.movable_idx].collision_data
        if x is not None:
            return True
        return False
        # return self.simulator.movable_objects[self.movable_idx].collision_data is not None

    def form_start_node(self) -> NodeReached:
        return NodeReached(0, sim_export=self.simulator.export())

    def _fetch_simspace(self, node: NodeReached):
        if node.replayer is None:
            raise ValueError("No replayer in node without simSpace")

        parent_simSpace = node.replayer.parent.sim_export
        if parent_simSpace is None:
            raise ValueError("No simSpace in parent")
        self.simulator: Simulator
        self.simulator.import_from(parent_simSpace)
        for i in range(node.iter_cnt):
            #give forces like before
            self._guide_to_goal(node.replayer.real_goal)
            #simulate
            self.simulator.step()
        node.sim_export = self.simulator.export()
        node.replayer = None
        return node

    def _guide_to_goal(self, goal: NodeGoal):
        """
        MOST IMPORTANT FUNCTION
        TODO: extract as separate class
        :param goal:
        :return:
        """
        pos = goal.pos
        rot = goal.rot
        force = goal.force
        rot_diff = rot - self.simulator.movable_objects[self.movable_idx].orientation
        direction = pos - self.simulator.movable_objects[self.movable_idx].position
        dir_len = np.linalg.norm(direction)
        if dir_len < 2:
            return False
        coef = 1
        if dir_len < 10:
            coef = dir_len/10

        direction /= dir_len


        self.simulator.movable_objects[self.movable_idx].apply_force(np.pad(direction*force*coef,(0,2)))
        self.simulator.movable_objects[self.movable_idx].angular_velocity =  rot_diff * 0.1
        return True





