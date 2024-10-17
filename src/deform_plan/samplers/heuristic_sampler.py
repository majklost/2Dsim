import numpy as np

from deform_plan.samplers.base_sampler import BaseSampler
from deform_plan.storage_wrappers.base_wrapper import BaseWrapper
from deform_plan.helpers.seed_manager import manager


class HeuristicSampler(BaseSampler):
    def __init__(self, child_sampler:BaseSampler,child_wrapper:BaseWrapper,paths, reached_fnc ,path_prob=0.2,std_dev=10):
        """
        Heuristic sampler that samples from the paths
        :param child_sampler: child sampler - that samples
        :param paths: list of paths
        :param reached_fnc: function that checks if the current waypoint on path is reached
        :param path_prob: probability of sampling on one of the paths
        """

        super().__init__()
        self._child_wrapper = child_wrapper
        self._child_sampler = child_sampler
        self._path_prob = path_prob
        self._wrapper = None
        self._paths = [p for p in paths if p]
        self.rng = np.random.default_rng(manager().get_seed(self.__class__.__name__))
        self._cur_path_point = np.zeros(len(paths),dtype=int)
        self.std = std_dev
        self.reached_fnc = reached_fnc


    def create_wrapper(self):
        if self._wrapper is None:
            self._wrapper = HeuristicWrapper(self._child_wrapper,self.update_cur_path)
        return self._wrapper

    def sample(self):
        if self.rng.random() < self._path_prob and len(paths) >0:
            path_idx = self.rng.integers(0,len(self._paths))
            path = self._paths[path_idx]
            point = path[self._cur_path_point[path_idx]]
            x,y = self.rng.normal(point,[self.std,self.std],2)
            return self._child_sampler.sample(x,y)

        return self._child_sampler.sample()

    def update_cur_path(self,node):
        for i,path in enumerate(self._paths):
            cur_point = path[self._cur_path_point[i]]
            if self._cur_path_point[i] < len(path) and self.reached_fnc(node,cur_point):
                self._cur_path_point[i] += 1


    def analytics(self):
        return None





class HeuristicWrapper(BaseWrapper):
    def __init__(self, wrapper:BaseWrapper,sampler_clb):
        """
        2nd order wrapper used to wrap another _storage wrapper
        It is used to help _sampler track the progress
        """
        super().__init__()
        self._sampler_clb = sampler_clb
        self._wrapper = wrapper

    def save_to_storage(self,node):
        self._sampler_clb(node)
        self._wrapper.save_to_storage(node)

    def get_nearest(self,point):
        self._wrapper.get_nearest(point)

    def get_path(self):
        self._wrapper.get_path()

    def get_all_nodes(self):
        self._wrapper.get_all_nodes()
