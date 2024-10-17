import numpy as np
import copy


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
    indxs = get_main_idxs(all_pts_num, main_pts_num)

    def reached_condition(sim: Simulator, start, goal, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        forces = guider_data.get("forces", None)
        if forces is None:
            return False
        for i in range(control_idxs_len):
            guided_obj.bodies[i].apply_force_middle(forces[i])
        dist_sum = dist_fnc(guided_obj.position[indxs],
                            goal.main_points)
        return dist_sum < threshold

    return reached_condition

def make_guider_standard(movable_idx,control_idxs,max_force):
    control_idxs = copy.copy(control_idxs)
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        all_pts = guided_obj.position
        control_pts = all_pts[control_idxs]
        dists = goal.controlled_points - control_pts
        flip_dists = goal.controlled_points - np.flip(control_pts,0)
        if np.sum(dists) > np.sum(flip_dists):
            dists = flip_dists
        forces = dists * max_force * len(all_pts) / len(control_pts)
        guider_data["forces"] = forces
        return True
    return guider_fnc

def make_fail_condition_standard(movable_idx):
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
    control_idxs = copy.copy(control_idxs)
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        all_pts = guided_obj.position
        control_pts = all_pts[control_idxs]
        cost = cost_fnc(all_pts)
        if cost > max_cost:
            guider_data["give_up"] = True


        dists = goal.controlled_points - control_pts
        flip_dists = goal.controlled_points - np.flip(control_pts,0)
        if np.sum(dists) > np.sum(flip_dists):
            dists = flip_dists
        forces = dists * max_force * len(all_pts) / len(control_pts)
        guider_data["forces"] = forces
        return True
    return guider_fnc


