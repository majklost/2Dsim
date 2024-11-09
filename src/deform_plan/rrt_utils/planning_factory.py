import numpy as np
import copy

from ..helpers.seed_manager import manager
from ..messages.sim_node import SimNode
from ..simulators.PM.pm_simulator import Simulator

def get_main_idxs(num_points, main_pts_num):
    seg = num_points//main_pts_num
    return np.array([0, seg, num_points-1-seg,num_points-1])

class Point:
    def __init__(self,node: SimNode|None,arbitrary_points=None,**kwargs):
        self.all_points = node.exporter_data["points"] if node is not None else arbitrary_points
        self.node = node
        self.config = kwargs["config"]
        self.main_points = self._calc_main_points()
        self.controlled_points = self._calc_controlled_points()

    def _calc_controlled_points(self):
        return self.all_points[self.config.CONTROL_IDXS]

    def _calc_main_points(self):
        idxs = get_main_idxs(len(self.all_points), self.config.MAIN_PTS_NUM)
        return self.all_points[idxs]



def make_reached_condition_standard(movable_idx,dist_fnc, control_idxs,threshold,main_pts_num,all_pts_num):
    threshold = threshold
    control_idxs_len = len(control_idxs)
    # indxs = get_main_idxs(all_pts_num, main_pts_num)

    def reached_condition(sim: Simulator, start, goal, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        forces = guider_data.get("forces", None)
        if forces is None:
            return False
        for i in range(len(control_idxs)):
            guided_obj.bodies[control_idxs[i]].apply_force_middle(forces[i])
        d1 = 1/control_idxs_len * dist_fnc(guided_obj.position[control_idxs],
                            goal.controlled_points)
        return d1 < threshold

    return reached_condition

def make_guider_standard(movable_idx,control_idxs,max_force):
    control_idxs = copy.copy(control_idxs)
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        all_pts = guided_obj.position
        control_pts = all_pts[control_idxs]
        dists = goal.controlled_points - control_pts
        forces = dists * max_force * len(all_pts) / len(control_pts)
        forces = np.clip(forces,-200*max_force* len(all_pts) / len(control_pts),200*max_force* len(all_pts) / len(control_pts))
        guider_data["forces"] = forces
        for i,idx in enumerate(control_idxs):
            guided_obj.bodies[idx].apply_force_middle(forces[i])
        return True
    return guider_fnc

def make_fail_condition_standard(movable_idx, control_idxs):
    def fail_condition(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if "give_up" in guider_data:
            return True
        return len(guided_obj.outer_collision_idxs) > 0
    return fail_condition

def make_exporter_standard(movable_idx):
    def exporter(sim:Simulator,start:SimNode,goal,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        return {"points": guided_obj.position}
    return exporter

def make_guider_cutoff(movable_idx,control_idxs,max_force,max_cost,cost_fnc):
    """enabling cutoff simulation where some cost is reached"""
    control_idxs = copy.copy(control_idxs) # copy so it is faster to reach
    fake_guider = make_guider_standard(movable_idx,control_idxs,max_force)
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        all_pts = guided_obj.position
        if cost_fnc(all_pts) > max_cost:
            print("cutoff")
            guider_data["give_up"] = True
        return fake_guider(sim,start,goal,guider_data,cur_iter_cnt)

        # guided_obj = sim.movable_objects[movable_idx]
        # all_pts = guided_obj.position
        # control_pts = all_pts[control_idxs]
        # cost = cost_fnc(all_pts)
        # if cost > max_cost:
        #     print("cutoff")
        #     guider_data["give_up"] = True
        #
        #
        # dists = goal.controlled_points - control_pts
        # forces = dists * max_force * len(all_pts) / len(control_pts)
        # guider_data["forces"] = forces
        # for i,idx in enumerate(control_idxs):
        #     guided_obj.bodies[idx].apply_force_middle(forces[i])
    return guider_fnc



def make_guider_TRRT(movable_idx,control_idxs,max_force,cost_fnc,K,T,alpha,n_fail_max,dist_fnc):
    rng = np.random.default_rng(manager().get_seed("TRRT_GUIDER"))
    n_fail = 0
    overall_rejections = 0
    def _transition_test(c_start, c_end, dist):
        nonlocal T, n_fail, overall_rejections
        cost_diff = c_end - c_start
        if cost_diff <= 0:
            # print("Prob ", 1)
            return True
        prob = np.exp(-cost_diff / (K * T * dist))
        # print("Cost_diff",cost_diff)
        # print("Prob",prob)
        if rng.random() < prob:
            T /= alpha
            n_fail = 0
            return True
        if n_fail >= n_fail_max:
            T *= alpha
            n_fail = 0
        else:
            n_fail += 1
        overall_rejections += 1
        return False

    fake_guider = make_guider_standard(movable_idx,control_idxs,max_force)

    def guider(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if "prev_node" not in guider_data:
            guider_data["prev_node"] = start.exporter_data["points"]
            guider_data["prev_cost"] = cost_fnc(guider_data["prev_node"])
            guider_data["wait_times"] = 0

        if guider_data["wait_times"] < 4:
            guider_data["wait_times"] += 1
            return fake_guider(sim,start,goal,guider_data,cur_iter_cnt)
        guider_data["wait_times"] = 0

        prev_node = guider_data["prev_node"]
        prev_cost = guider_data["prev_cost"]
        all_pts = guided_obj.position
        cost = cost_fnc(all_pts)
        if _transition_test(prev_cost,cost,dist_fnc(prev_node,all_pts)):
            guider_data["prev_node"] = all_pts
            guider_data["prev_cost"] = cost
        else:
            guider_data["give_up"] = True

        return fake_guider(sim,start,goal,guider_data,cur_iter_cnt)

    return guider


