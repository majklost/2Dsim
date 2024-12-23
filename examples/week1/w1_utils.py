"""
Approaches:
4 - main points of the cable - start, end, and two points in the middle
use GNAT to store the points
use the distance function to get the nearest point - distance will consider only the main points
Immediately after the collision, terminate the simulation
Need: Draw cables as these 4 main points

IF success - try to implement creased index

"""


import numpy as np

from deform_plan.messages.sim_node import SimNode
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.storages.GNAT import GNAT


MAIN_PTS_NUM = 4

def distance(p1: 'Point', p2:'Point'):
    return distance_inner(p1.main_points, p2.main_points)

def distance_inner(p1: np.array, p2:np.array):
    return np.linalg.norm(p1 - p2, axis=1).sum()/len(p1)

def calc_creased_index(body_positions):
    pass


def make_guider(movable_idx, controlled_idxs, max_force):
    def guider_fnc(sim: Simulator, start: SimNode, goal:Point, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        control_pts = np.array([guided_obj.bodies[i].position for i in controlled_idxs])
        dists = goal.controlled_points - control_pts
        forces = dists * max_force * len(guided_obj.bodies) / len(controlled_idxs)
        guider_data["forces"] = forces
        for i,idx in enumerate(controlled_idxs):
            guided_obj.bodies[idx].apply_force_middle(forces[i])
    return guider_fnc

def make_fail_condition(movable_idx):
    def end_cond_vel(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        return len(sim.movable_objects[movable_idx].outer_collision_idxs) != 0
    return end_cond_vel

def make_reached_condition(threshold, num_points,controlled_idxs):
    def reached_condition(sim: Simulator, start: SimNode, goal, guider_data: dict, cur_iter_cnt):
        guided_obj = sim.movable_objects[0]
        forces = guider_data.get("forces", None)
        if forces is None:
            return False
        for i,idx in enumerate(controlled_idxs):
            guided_obj.bodies[idx].apply_force_middle(forces[i])
        dist_sum =  distance_inner(guided_obj.position[get_idxs(num_points)], goal.main_points)
        return dist_sum < threshold
    return reached_condition

def get_idxs(num_points):
    SEG = num_points//MAIN_PTS_NUM
    return np.array([0, SEG, num_points-1-SEG,num_points-1])


def make_exporter(movable_idx: int):
    def exporter(sim: Simulator, start: SimNode | None, goal, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        return {
            "points": guided_obj.position
        }
    return exporter


class Point:
    def __init__(self, node: SimNode|None,controlled_idxs):
        self.node = node
        if node is not None:
            self.main_points = self._calc_main_pts()

            self.controlled_points = self._calc_controlled_points(controlled_idxs)

    def _calc_main_pts(self):
        points = self.node.exporter_data["points"]
        return points[get_idxs(len(points))]
    def _calc_controlled_points(self, controlled_idxs):
        points = self.node.exporter_data["points"]
        return points[controlled_idxs]

    @staticmethod
    def from_points(points,controlled_idxs):
        p = Point(None,controlled_idxs)
        p.main_points = points[get_idxs(len(points))]
        p.controlled_points = points[controlled_idxs]
        return p


class StorageWrapper:
    def __init__(self,goal, goal_threshold,controlled_idxs):
        self.goal = goal
        self.threshold = np.inf
        self.gnat = GNAT(distancefnc=distance,arity=5)
        self.goal_threshold =goal_threshold
        self.want_next_iter = True
        self.try_goal = True
        self.controlled_idxs = controlled_idxs
        self.best_dist = float("inf")
        self._end_node =None

    def save_to_storage(self,node):
        point = Point(node,self.controlled_idxs)
        dist = distance(point,self.goal)
        if dist < self.threshold:
            self.try_goal = True
            self.threshold /= 1.5
            if dist < self.threshold:
                print("Threshold set to dist")
                self.threshold = dist
        if dist < self.goal_threshold:
            self.want_next_iter = False
        if dist < self.best_dist:
            self.best_dist = dist
            self._end_node = node
        self.gnat.insert(point)

    def get_nearest(self,point):
        return self.gnat.nearest_neighbour(point)

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

    def get_all_points(self):
        return self.gnat.get_all_nodes()


