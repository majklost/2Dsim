import numpy as np
from typing import cast

from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode
from ..storages.brute_force import BruteForce
from ..storages.kd_tree import KDTree
from ..assets.PM import *


def make_guider(movable_idx: int, allow_control_idxs: list, max_force: float, distance_thresh=8):
    def guider_fnc(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        if cur_iter_cnt == 0:
            guider_data["mode"] = 0
        guided_obj = sim.movable_objects[movable_idx]
        guider_data["give_up"] = True
        if not isinstance(guided_obj, Cable):
            raise ValueError("Trying to guide non-cable object with cable guarder: ", guided_obj)
        guided_obj = cast(Cable, guided_obj)

        forces,ok_cnt = get_linear_forces(guided_obj, goal, max_force, allow_control_idxs)
        guider_data["forces"] = forces
        for i in allow_control_idxs:
            guided_obj.bodies[i].apply_force(forces[i])


        if ok_cnt == len(allow_control_idxs):
            # print("OK")
            return False

        return True

    def guider_dummy(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        for i in allow_control_idxs:
            rand_vec = np.random.rand(2)

            guided_obj.bodies[i].apply_force(rand_vec * max_force / len(allow_control_idxs))
            # if guided_obj.bodies[i].collision_data is no
        return True

    return guider_fnc

def make_reached_condition(movable_idx: int):
    def reached_condition(sim: Simulator, start: SimNode, goal, guider_data:dict, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        forces = guider_data.get("forces",None)
        if forces is None:
            return False

        dist_sum =np.inf
        for i in range(len(guided_obj.bodies)):
            guided_obj.bodies[i].apply_force(forces[i])
            dist_sum = min(dist_sum,np.linalg.norm(guided_obj.bodies[i].position-goal.points[i]))



        return dist_sum < 5

    return reached_condition


def make_end_cond_all_vel(movable_idx: int, force_thresh=300, seg_vel_max_t=50):
    def end_cond_vel(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        if cur_iter_cnt == 0:
            guider_data["last_vel"] = 0
            return False

        guided_obj = sim.movable_objects[movable_idx]  #type 'Cable'
        if not isinstance(guided_obj, Cable):
            raise ValueError("Trying to guide non-cable object with cable guarder: ", guided_obj)
        guided_obj = cast(Cable, guided_obj)
        all_force = sum(map(lambda x: np.linalg.norm(x.get_manual_force()), guided_obj.bodies))

        if all_force > force_thresh:
            seg_vel_max = max(map(lambda x: np.linalg.norm(x.velocity), guided_obj.bodies))
            # print(seg_vel_max)
            if seg_vel_max < seg_vel_max_t and seg_vel_max < guider_data["last_vel"]:
                # print("Terminated")
                return True
            guider_data["last_vel"] = seg_vel_max
        return False

    def end_cond_dummy(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        return len(sim.movable_objects[movable_idx].outer_collision_idxs) != 0

    return end_cond_dummy


def make_exporter(movable_idx: int):
    def exporter(sim: Simulator, start: SimNode | None, goal, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]  # type 'Cable'
        if not isinstance(guided_obj, Cable):
            raise ValueError("Trying to guide non-cable object with cable guarder: ", guided_obj)
        guided_obj = cast(Cable, guided_obj)
        points = list(map(lambda x: x.position, guided_obj.bodies))
        data = dict()
        data["points"] = points
        return data

    return exporter


def distance_fnc(a: 'Point', b: 'Point'):
    return np.linalg.norm(a.mean - b.mean)

def max_points_distance(a: 'Point', b: 'Point'):
    return np.linalg.norm(a.points - b.points, axis=1).max()


class Point:
    def __init__(self, points: np.array,node):
        self.points = np.array(points)
        self.mean = self._calc_mean()
        self.node = node

    def _calc_mean(self):
        return np.mean(self.points, axis=0)

    def __getitem__(self, item):
        return self.mean[item]


class StorageWrapper:
    def __init__(self, goal, threshold=250, goal_threshold=10):
        self.goal = goal
        self.threshold = threshold
        self.tree = KDTree(max_points_distance,2)
        self._end_node: SimNode | None = None
        self.best_dist = float("inf")
        self.want_next_iter = True
        self.try_goal = False
        self.goal_threshold = goal_threshold

    def save_to_storage(self, node: SimNode):
        points = node.exporter_data["points"]
        point = Point(points,node)
        dist = max_points_distance(point, self.goal)
        if dist < self.threshold:
            self.try_goal = True
            self.threshold /= 1.5
        if dist < self.goal_threshold:
            self.want_next_iter = False
        if dist < self.best_dist:
            self._end_node = node
            self.best_dist = dist
        self.tree.insert(point)

    def get_nearest(self, point: Point):
        return self.tree.nearest_neighbour(point).node

    def get_path(self):
        path = []
        if self._end_node is None:
            return path

        cur = self._end_node
        while True:
            path.append(cur)
            if cur.replayer.parent is None:
                break
            cur = cur.replayer.parent

        return list(reversed(path))




def get_linear_forces(guided_obj: Cable, goal, max_force, allow_control_idxs):
    vecs = list(map(lambda g, b: (g - b.position), goal.points, guided_obj.bodies))
    distances = list(map(lambda x: np.linalg.norm(x), vecs))
    unit_vecs = list(map(lambda x: x / np.linalg.norm(x), vecs))
    ok_cnt = 0
    forces = np.zeros((len(guided_obj.bodies), 2))
    for i in allow_control_idxs:
        coef = 1
        if distances[i] < 8:
            ok_cnt += 1
            coef = distances[i] / 8

        forces[i] = unit_vecs[i] * max_force * coef/len(allow_control_idxs)
    return forces, ok_cnt

def get_rotation_forces(guided_obj,forces,max_force, guider_data,allow_control_idxs):
    mid_segment_idx,left_shorter = guider_data["mode1_data"]
    mid_pos = guided_obj.bodies[mid_segment_idx].position
    left_pos = guided_obj.bodies[0].position
    right_pos = guided_obj.bodies[-1].position

    # if ccw(left_pos, mid_pos, right_pos):
    #     guider_data["mode"] = 0
    #     return

    if left_shorter:
        unit_m_l = (left_pos - mid_pos) / np.linalg.norm(left_pos - mid_pos)



        for i in allow_control_idxs:
            if i < mid_segment_idx:
                forces[i] *= 0
    else:
        for i in allow_control_idxs:
            if i >= mid_segment_idx:
                forces[i] *= 0

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

