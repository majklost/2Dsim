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
from deform_plan.rrt_utils.config import CONFIG


MAIN_PTS_NUM = CONFIG["MAIN_PTS_NUM"]
MAX_CREASED_COST = CONFIG["MAX_CREASED_COST"]

def distance(p1: 'Point', p2:'Point'):
    return distance_inner(p1.main_points, p2.main_points)

def distance_inner(p1: np.array, p2:np.array):
    return np.linalg.norm(p1 - p2, axis=1).sum()/len(p1)

def calc_stretch_index(body_positions,distance_matrix):
    idxs = get_idxs(len(body_positions))
    stretch_index = 0
    num_tests = 0
    for i in range(len(idxs)):
        for j in range(i+1,len(idxs)):
            stretch_index += np.linalg.norm(body_positions[idxs[i]] - body_positions[idxs[j]]) / distance_matrix[i,j]
            num_tests += 1
    return 1-stretch_index/num_tests


def calc_distance_matrix(start_sim:Simulator,movable_idx):
    guided_obj = start_sim.movable_objects[movable_idx]
    idxs = get_idxs(len(guided_obj.position))
    positions = guided_obj.position[idxs]
    distance_matrix  = np.zeros((len(positions),len(positions)))
    for i in range(len(positions)):
        for j in range(i+1,len(positions)):
            distance_matrix[i,j] = np.linalg.norm(positions[i]-positions[j])
            distance_matrix[j,i] = distance_matrix[i,j]
    return distance_matrix




def make_guider(movable_idx, controlled_idxs, max_force,dist_matrix):
    def guider_fnc(sim: Simulator, start: SimNode, goal:Point, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        all_pts = guided_obj.position
        stretch_idx = calc_stretch_index(all_pts,dist_matrix)
        if stretch_idx > MAX_CREASED_COST and CONFIG["USE_MAX_CREASED"]:
            guider_data["give_up"] = True

        control_pts = all_pts[controlled_idxs]
        dists = goal.controlled_points - control_pts
        forces = dists * max_force * len(guided_obj.bodies) / len(controlled_idxs)
        guider_data["forces"] = forces
        for i,idx in enumerate(controlled_idxs):
            guided_obj.bodies[idx].apply_force_middle(forces[i])
    return guider_fnc

def make_fail_condition(movable_idx):
    def end_cond_vel(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        if "give_up" in guider_data:
            # print("Too much creased")
            return True

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
    def __init__(self, node: SimNode|None,controlled_idxs,arbitrary_points=None):
        self.node = node
        self.all_pts = self.node.exporter_data["points"] if node is not None else arbitrary_points
        self.main_points = self._calc_main_pts()
        self.controlled_points = self._calc_controlled_points(controlled_idxs)

    def _calc_main_pts(self):
        points = self.all_pts
        return points[get_idxs(len(points))]
    def _calc_controlled_points(self, controlled_idxs):
        points = self.all_pts
        return points[controlled_idxs]


    @staticmethod
    def from_points(points,controlled_idxs):
        p = Point(None,controlled_idxs,arbitrary_points=points)
        return p


class StorageWrapper:
    def __init__(self,goal, goal_threshold,controlled_idxs):
        self.goal = goal
        self.goal_bias = CONFIG["GOAL_BIAS"]
        self.threshold = np.inf
        self.gnat = GNAT(distancefnc=distance,arity=5)
        self.goal_threshold =goal_threshold
        self.want_next_iter = True
        self.try_goal = True
        self.controlled_idxs = controlled_idxs
        self.best_dist = float("inf")
        self._end_node =None

    def save_to_storage(self,node:SimNode):
        point = Point(node,self.controlled_idxs)
        dist = distance(point,self.goal)
        if dist < self.threshold:
            self.try_goal = True
            self.threshold /= 2
            print("New best: ", dist)


            if dist < self.threshold:
                print("Threshold set to dist")
                self.threshold = dist
        if dist < self.goal_threshold:
            self.want_next_iter = False
        if dist < self.best_dist:
            self.best_dist = dist

            self._end_node = node
        self.gnat.insert(point)
        return True

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
        return self.gnat.get_all_points()

class StorageWrapperTRRT(StorageWrapper):
    def __init__(self,goal, goal_threshold,controlled_idxs,stretch_matrix):
        super().__init__(goal,goal_threshold,controlled_idxs)
        self.stretch_matrix = stretch_matrix
        self.K = calc_stretch_index(goal.all_pts,stretch_matrix)
        self.T = CONFIG["TRRT"]["T"]
        self.nfail_max = CONFIG["TRRT"]["nfail_max"]
        self.alpha = CONFIG["TRRT"]["alpha"]
        self.nfail = 0
        self.overall_rejections = 0
        self.last_points = None


    def save_to_storage(self,node:SimNode):
        start = node.previous_node
        point = Point(node,self.controlled_idxs)
        if start is None:
            #start is total start
            super().save_to_storage(node)
            return True
        parent = Point(start,self.controlled_idxs)
        dist = distance(parent,point)
        stretch_index_point = calc_stretch_index(point.all_pts,self.stretch_matrix)
        stretch_index_parent = calc_stretch_index(parent.all_pts,self.stretch_matrix)


        if self.transition_test(stretch_index_parent,stretch_index_point,dist):
            super().save_to_storage(node)
            return True
        # print("Rejection", self.overall_rejections)
        return False

    def transition_test(self,c_start,c_end,dist):
        stretch_diff = c_end - c_start
        # print("Stretch diff: ", stretch_diff)
        if stretch_diff <= 0:
            return True
        prob = np.exp(-stretch_diff/(self.K*self.T*dist))
        if np.random.random() < prob:
            self.T/=self.alpha
            self.nfail = 0
            return True
        if self.nfail >= self.nfail_max:
            self.T *= self.alpha
            self.nfail = 0
        else:
            self.nfail += 1
        self.overall_rejections +=1
        return False






