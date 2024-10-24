"""Planner for planning one rigid object"""
import time
from copy import deepcopy
from typing import Callable,Any, TypedDict

#TODO: rewrite to better use - now just PoC

from .base_planner import BasePlanner
from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode, Replayer
from ..messages.planner_messages import PlannerResponse


# guider: Callable[[Simulator,SimNode,Any,dict,int],bool],
#                  fail_condition: Callable[[Simulator,SimNode,Any,int],bool],
#                  reached_condition: Callable[[Simulator,SimNode,Any,int],bool],
#                  exporter: Callable[[Simulator,SimNode|None,Any,int],dict],
class PlanningFncs(TypedDict):
    """
    A collection of planning-related functions.

    Attributes:
    - guider: A callable function with the signature:
        guider(simulator: Simulator, start: SimNode, goal: Any, guider_data: dict, cur_cnt: int) -> bool
    - fail_condition: A callable function with the signature:
        fail_condition(simulator: Simulator, start: SimNode, goal: Any, guider_data: dict, cur_cnt: int) -> bool
    - reached_condition: A callable function with the signature:
        reached_condition(simulator: Simulator, start: SimNode, goal: Any, guider_data: dict, cur_cnt: int) -> bool
    - exporter: A callable function with the signature:
        exporter(simulator: Simulator, start: SimNode | None, goal: Any, cur_cnt: int) -> dict
    """
    guider: Callable[[Simulator,SimNode,Any,dict,int],bool]
    fail_condition: Callable[[Simulator,SimNode,Any,dict,int],bool]
    reached_condition: Callable[[Simulator, SimNode, Any,dict, int], bool]
    exporter: Callable[[Simulator, SimNode | None, Any, int], dict]



class FetchAblePlanner(BasePlanner):
    """
    Planner that allows some simulation be left only to be fetched later
    """
    def __init__(self,simulator:Simulator,

                 planning_functions:PlanningFncs,
                 max_step_cnt: int = 1000,
                 only_simuls: bool = False,
                 sampling_period =2000,
                 guider_period = 1,
                 track_analytics = False

                 ):


        self.simulator = simulator
        self.guider = planning_functions["guider"]
        self.fail_condition = planning_functions["fail_condition"]
        self.reached_condition = planning_functions["reached_condition"]
        self.exporter = planning_functions["exporter"]
        self.only_simuls = only_simuls
        self.max_iter_cnt = max_step_cnt
        self.after_load_clb = None
        self.sampling_period = sampling_period
        self.guider_period = guider_period
        self.analytics = {
            "TIMES":{
            "GUIDER": 0,
            "FAIL": 0,
            "REACHED": 0,
            "FETCH": 0,
            "IMPORT": 0,
            "EXPORT": 0,
            "SIMULATOR": 0,
            "GUIDER_COPY": 0,
            },
            "SUM_STEP_CNT" : 0,
            "FILLED_CNT": 0,
            "COLLIDED_CNT": 0,
            "REACHED_CNT": 0

        }
        self.track_analytics = track_analytics


    def _check_path_analytics(self,start,goal):
        response = PlannerResponse()
        if start.sim_export is None:
            t1 = time.time()
            self._fetch_simspace(start)
            t2 = time.time()
            self.analytics["TIMES"]["FETCH"] += t2-t1
        else:
            t1 = time.time()
            self.simulator.import_from(start.sim_export)
            t2 = time.time()
            self.analytics["TIMES"]["IMPORT"] += t2-t1

        self.simulator: Simulator  # cast the type
        if self.after_load_clb is not None:
            self.after_load_clb(self.simulator)
        t1 = time.time()
        guider_data = deepcopy(start.guider_data)
        t2 = time.time()
        self.analytics["TIMES"]["GUIDER_COPY"] += t2-t1
        collided = False
        cur_cnt = 0
        reached = False
        for i in range(self.max_iter_cnt):
            t1 = time.time()
            if self.reached_condition(self.simulator, start, goal, guider_data, cur_cnt):
                if self.fail_condition(self.simulator, start, goal, guider_data, cur_cnt):
                    collided = True
                self.analytics["REACHED_CNT"] += 1
                reached = True
                break
            t2 = time.time()
            if i % self.guider_period == 0:
                self.guider(self.simulator, start, goal, guider_data, cur_cnt)
            t3 = time.time()
            self.simulator.step()
            t4 = time.time()
            if self.fail_condition(self.simulator, start, goal, guider_data, cur_cnt):
                collided = True
                break
            t5 = time.time()
            cur_cnt = i + 1
            if cur_cnt % self.sampling_period == 0 and cur_cnt != 1:
                exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
                response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start,
                                                                   start.all_iter_cnt + cur_cnt))

            t6 =time.time()
            self.analytics["TIMES"]["REACHED"] += t2-t1
            self.analytics["TIMES"]["GUIDER"] += t3 - t2
            self.analytics["TIMES"]["SIMULATOR"] += t4 - t3
            self.analytics["TIMES"]["FAIL"] += t5 - t4
            self.analytics["TIMES"]["EXPORT"] += t6 - t5
        t7 = time.time()
        self.analytics["SUM_STEP_CNT"] += cur_cnt

        if not collided:
            exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
            response.checkpoints.append(
                self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt + cur_cnt))
            response.reached_goal = reached
            response.checkpoints[-1].reached = reached
        t8 =time.time()
        self.analytics["TIMES"]["EXPORT"] += t8-t7
        if cur_cnt == self.max_iter_cnt:
            self.analytics["FILLED_CNT"] += 1
        if collided:
            self.analytics["COLLIDED_CNT"] += 1
        response.link_checkpoints()
        return response


    def _check_path(self,start,goal):
            response = PlannerResponse()
            if start.sim_export is None:
                self._fetch_simspace(start)
            else:
                self.simulator.import_from(start.sim_export)

            self.simulator: Simulator #cast the type
            if self.after_load_clb is not None:
                self.after_load_clb(self.simulator)

            guider_data = deepcopy(start.guider_data)
            collided = False
            reached = False
            cur_cnt = 0
            for i in range(self.max_iter_cnt):
                if self.reached_condition(self.simulator, start, goal, guider_data,cur_cnt):
                    if self.fail_condition(self.simulator, start, goal, guider_data, cur_cnt):
                        collided = True
                    self.analytics["REACHED_CNT"] += 1
                    reached = True
                    break
                if i% self.guider_period ==0:
                    self.guider(self.simulator, start, goal, guider_data,cur_cnt)
                self.simulator.step()
                if self.fail_condition(self.simulator, start, goal, guider_data,cur_cnt):
                    collided = True
                    break
                cur_cnt = i+1
                if cur_cnt % self.sampling_period == 0 and cur_cnt != 1:
                    exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
                    response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt+cur_cnt))
            if not collided:
                exported_data = self.exporter(self.simulator, start, goal, cur_cnt)
                response.checkpoints.append(self.create_checkpoint(exported_data, guider_data, cur_cnt, goal, start, start.all_iter_cnt+cur_cnt))
                response.reached_goal = reached
                response.checkpoints[-1].reached = reached
            response.link_checkpoints()
            return response

    def check_path(self, start:SimNode, goal:Any) -> PlannerResponse:
        if self.track_analytics:
            return self._check_path_analytics(start,goal)
        else:
            return self._check_path(start,goal)
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
        :param goal: _goal_points of the segment
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
            # raise ValueError("No simSpace in parent")
            self._fetch_simspace(parent)

        self.simulator: Simulator
        if parent_sim_space is not None:
            self.simulator.import_from(parent_sim_space)


        for i in range(node.replayer.segment_iter_cnt):
            #pick direction
            if self.reached_condition(self.simulator, parent, node.replayer.real_goal, parent_guider_data,i):
                break

            if i% self.guider_period ==0:
                self.guider(self.simulator, parent, node.replayer.real_goal, parent_guider_data,i)

            self.simulator.step()
        node.replayed_cnt +=1
        if node.replayed_cnt >=1:
            node.sim_export = self.simulator.export()
        return node





