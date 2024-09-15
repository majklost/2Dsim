import numpy as np
from typing import cast

from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode
from ..assets.PM import *



def make_guider(movable_idx:int, allow_control_idxs:list, max_force:float, distance_thresh=8):
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        guider_data["give_up"] = True


        if not isinstance(guided_obj,Cable):
            raise ValueError("Trying to guide non-cable object with cable guarder: ", guided_obj)
        guided_obj= cast(Cable,guided_obj)

        vecs = list(map(lambda g, b: (g-b.position),goal,guided_obj.bodies))
        # print("vecs: ", vecs)
        distances = list(map(lambda x: np.linalg.norm(x), vecs))
        unit_vecs = list(map(lambda x: x/np.linalg.norm(x),vecs))
        ok_cnt = 0

        for i in allow_control_idxs:
            coef = 1
            if distances[i] < distance_thresh:
                ok_cnt += 1
                coef = distances[i]/distance_thresh

            guided_obj.bodies[i].apply_force(unit_vecs[i]*max_force/len(allow_control_idxs)*coef)


        if ok_cnt == len(allow_control_idxs):
            print("OK")
            return False

        return True





    return guider_fnc

def make_end_cond_all_vel(movable_idx:int,force_thresh=300,seg_vel_sum_t=50):
    def end_cond_vel(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        if cur_iter_cnt ==0:
            guider_data["last_vel"] = 0
            return False

        guided_obj = sim.movable_objects[movable_idx] #type 'Cable'
        if not isinstance(guided_obj,Cable):
            raise ValueError("Trying to guide non-cable object with cable guarder: ", guided_obj)
        guided_obj = cast(Cable, guided_obj)
        all_force = sum(map(lambda x: np.linalg.norm(x.get_manual_force()),guided_obj.bodies))

        if all_force >force_thresh:
            seg_vel_sum = sum(map(lambda x: np.linalg.norm(x.velocity),guided_obj.bodies))
            print(seg_vel_sum)
            if seg_vel_sum < seg_vel_sum_t and seg_vel_sum < guider_data["last_vel"]:
                print("Terminated")
                return True
            guider_data["last_vel"] = seg_vel_sum
        return False

    return end_cond_vel


def make_exporter(movable_idx:int):
    def exporter(sim:Simulator,start:SimNode|None,goal,cur_iter_cnt):
        return dict()
    return exporter