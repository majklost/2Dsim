"""Planner for planning one rigid object"""

from copy import deepcopy
import time
from typing import Callable,Any
#TODO: rewrite to better use - now just PoC

from .base_planner import BasePlanner
from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode, Replayer
from ..messages.planner_messages import PlannerResponse



class FetchAblePlanner(BasePlanner):
    """
    Planner that allows some simulation be left only to be fetched later
    """
    def __init__(self,simulator:Simulator,
                 guider: Callable[[Simulator,SimNode,Any,dict,int],bool],
                 end_condition: Callable[[Simulator,SimNode,Any,int],bool],
                 exporter: Callable[[Simulator,SimNode|None,Any,int],dict],
                 max_iter_cnt: int = 1000,
                 only_simuls: bool = False,
                 sampling_period =2000,

                 ):


        self.simulator = simulator
        self.guider = guider
        self.end_condition = end_condition
        self.exporter = exporter
        self.only_simuls = only_simuls
        self.max_iter_cnt = max_iter_cnt
        self.after_load_clb = None
        self.sampling_period = sampling_period
        self.PREPARATION = 0
        self.GUIDER = 0
        self.SIMULATOR = 0
        self.ENDER = 0
        self.EXPORTER = 0
        self.REST = 0





    def check_path(self, start:SimNode, goal:Any) -> PlannerResponse:
        response = PlannerResponse()
        # t1 = time.time()
        if start.sim_export is None:
            self._fetch_simspace(start)
        # t2 = time.time()

        self.simulator: Simulator #cast the type

        self.simulator.import_from(start.sim_export)

        if self.after_load_clb is not None:
            self.after_load_clb(self.simulator)

        guider_data = deepcopy(start.guider_data)

        collided = False

        cur_cnt = 0
        # self.PREPARATION += t2-t1
        for i in range(self.max_iter_cnt):
            #pick direction
            if not self.guider(self.simulator, start, goal, guider_data,cur_cnt):
                break
            # t3 = time.time()
            self.simulator.step()
            # t4 = time.time()
            #if collided do not continue and do not create checkpoint
            # t5 = time.time()
            if self.end_condition(self.simulator, start, goal, guider_data,cur_cnt):
                collided = True
                break
            # t6 = time.time()
            cur_cnt = i+1
            #now I am saving checkpoint that is collision free
            if cur_cnt % self.sampling_period == 0 and cur_cnt != 1:
                exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
                response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt+cur_cnt))
            # exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
            # response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt+cur_cnt))
            # t7 = time.time()
            # self.GUIDER += t4-t3
            # self.SIMULATOR += t5-t4
            # self.ENDER += t6-t5
            # self.EXPORTER += t7-t6
        # t8 = time.time()
        if not collided:
            exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
            response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt+cur_cnt))
        # t9 = time.time()
        # self.REST += t9-t3
        # response.checkpoints = response.checkpoints[::self.sampling_period]
        return response

    def create_checkpoint(self,exported_data:dict,
                          guider_data:dict,
                          cur_iter_cnt:int,
                          goal: Any,parent:SimNode,
                          all_iter_cnt:int) -> SimNode:
        """
        Create a SimNode from the given data
        :param exported_data: data that exported gave
        :param guider_data: data that guider gave
        :param cur_iter_cnt: iteration from last checkpoint that contains simulation
        :param goal: goal of the segment
        :param parent:  parent of the node
        :param all_iter_cnt: iteration from the start
        :return: SimNode
        """

        replayer = Replayer(segment_iter_cnt=cur_iter_cnt,real_goal=goal,parent=parent)
        if self.only_simuls:
            return SimNode(all_iter_cnt=all_iter_cnt,exporter_data=exported_data, guider_data=deepcopy(guider_data), replayer=replayer,sim_export=self.simulator.export())
        else:
            return SimNode(all_iter_cnt=all_iter_cnt,exporter_data=exported_data, guider_data=deepcopy(guider_data), replayer=replayer)


    def form_start_node(self) -> SimNode:
        replayer = Replayer(segment_iter_cnt=0,real_goal=None,parent=None)
        export = self.exporter(self.simulator, None, None, 0)
        return SimNode(export,dict(), sim_export=self.simulator.export(), all_iter_cnt=0, replayer=replayer)

    def _fetch_simspace(self, node: SimNode):
        if node.replayer is None:
            raise ValueError("No replayer in node without simSpace")
        parent = node.replayer.parent
        parent_guider_data = deepcopy(parent.guider_data)
        parent_sim_space = parent.sim_export
        if parent_sim_space is None:
            raise ValueError("No simSpace in parent")
        self.simulator: Simulator
        self.simulator.import_from(parent_sim_space)


        for i in range(node.replayer.segment_iter_cnt):
            #pick direction
            if not self.guider(self.simulator, parent, node.replayer.real_goal, parent_guider_data,i):
                break
            self.simulator.step()
        node.sim_export = self.simulator.export()
        return node





