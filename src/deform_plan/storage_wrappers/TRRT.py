#implements the TRRT _storage wrapper
# https://homepages.laas.fr/jcortes/Papers/jaillet_aaaiWS08.pdf
import numpy as np

from deform_plan.messages.sim_node import SimNode
from deform_plan.storage_wrappers.cable_wrapper import StorageWrapper
from deform_plan.helpers.seed_manager import manager


class TRRT(StorageWrapper):
    def __init__(self,utils, trrt_utils,goal,goal_threshold,controlled_idxs, verbose=False):
        """
        Implements the TRRT _storage wrapper
        :param utils: dict with keys "dist" - distance function, "cls_maker" - class that wraps the points, "_storage" - _storage object
        :param trrt_utils: dict with keys "cost_fnc" - cost function, "K" - TRRT parameter, "T" - TRRT parameter, "n_fail_max" - TRRT parameter, "alpha" - TRRT parameter
        """
        super().__init__(utils,goal,goal_threshold,controlled_idxs, verbose)
        self.cost_fnc = trrt_utils["cost_fnc"]
        self.K = trrt_utils["K"]
        self.T = trrt_utils["T"]
        self.n_fail_max = trrt_utils["n_fail_max"]
        self.alpha = trrt_utils["alpha"]
        self.n_fail = 0
        self.overall_rejections = 0
        self.last_points = None
        #analytics
        self.queries = 0
        self.rejections = 0
        self.rng = np.random.default_rng(manager().get_seed(self.__class__.__name__))

    def save_to_storage(self, node:SimNode):
        self.queries += 1
        point = self._make_point(node)
        parent = node.previous_node
        if parent is None:
            super().save_to_storage(node)
            return True
        parent = self._make_point(parent)
        dist = self.dist(parent,point)
        cost_point = self.cost_fnc(parent)
        cost_parent = self.cost_fnc(point)
        if self._transition_test(cost_parent,cost_point,dist):
            super().save_to_storage(node)
            return True
        self.rejections += 1
        return False

    def _transition_test(self,c_start,c_end,dist):
        cost_diff = c_end - c_start
        if cost_diff <= 0:
            return True
        prob = np.exp(-cost_diff/(self.K*self.T*dist))
        if self.rng.random() < prob:
            self.T/=self.alpha
            self.n_fail = 0
            return True
        if self.n_fail >= self.n_fail_max:
            self.T *= self.alpha
            self.n_fail = 0
        else:
            self.n_fail += 1
        self.overall_rejections +=1
        return False

