import numpy as np


from src.cable_planner import CablePlanner
from src.helpers.goalspecifier import GoalSpecifier
from src.bezier_sampler import Sampler
from src.RRTNode import RRTNodeCable
from src.helpers.kd_tree import BruteForce

class RRT:
    def __init__(self,localPlanner: CablePlanner,goal_region: GoalSpecifier, sc:Sampler.SamplingConstraints,cable_length,cable_segments_num,controllable_indexes,seed=10):
        self.localPlanner = localPlanner
        self.goal_region = goal_region
        self.sampler = Sampler(cable_length,cable_segments_num,controllable_indexes, seed=seed)
        self.sampling_constraints = sc
        self.tree = BruteForce()
        self.node_cnt = 0
        self.cur_best_num = 0
        self.cur_best_node = None
        self.controllable_indexes = controllable_indexes


    def _check_validity(self,start:RRTNodeCable):
        if start is None:
            raise ValueError("Start or goal is None")
        if start.simSpace is None:
            raise ValueError("Start simSpace is None")
    @staticmethod
    def all_dist(candidate:RRTNodeCable, root:RRTNodeCable):
        # print(candidate)
        # print(root)
        # TODO stop using private variable, change dist, controllable_dist and filling points
        return np.linalg.norm(candidate._movable_bodies - root._movable_bodies)


    def controllable_dist(self,candidate:RRTNodeCable,root:RRTNodeCable):
        #not euclidean, just quickfix
        s = 0
        for i in self.controllable_indexes:
            s += np.linalg.norm(candidate.points[i] - root.points[i])
        return s



    def check_n_add(self,checkpoints):
        for n in checkpoints:
            n.added_cnt = self.node_cnt
            self.node_cnt += 1
            self.tree.insert(n)
            cnt = self.goal_region.quick_check_points(n._movable_bodies)
            if cnt > self.cur_best_num:
                self.cur_best_num = cnt
                self.cur_best_node = n
                print("Best node count: ",self.cur_best_num)

            if self.cur_best_num == self.goal_region.required_in_count:
                print("Goal reached")
                return True
        return False


    def find_path(self,start:RRTNodeCable,iters=10000):
        self._check_validity(start)
        self.tree.insert(start)
        for i in range(iters):
            cpoints = self.sampler.sample(self.sampling_constraints)
            all_points = self.sampler.extract_all_points()
            new_node = RRTNodeCable(cpoints)
            new_node._movable_bodies = all_points
            nearest_node = self.tree.nearestNeighbour(new_node,distancefnc=self.all_dist)
            checkpoints = self.localPlanner.check_path(nearest_node, new_node)
            if self.check_n_add(checkpoints):
                return self.cur_best_node

            if i % 10 == 0:
                print("Iteration: ",i)
                print("Best node count: ",self.cur_best_num)
        print("Best node count: ",self.cur_best_num)
        return self.cur_best_node










