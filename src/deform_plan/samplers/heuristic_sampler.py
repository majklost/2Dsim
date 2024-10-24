import numpy as np

from deform_plan.samplers.base_sampler import BaseSampler
from deform_plan.storage_wrappers.base_wrapper import BaseWrapper
from deform_plan.helpers.seed_manager import manager


class HeuristicSampler(BaseSampler):
    def __init__(self, child_sampler:BaseSampler,child_wrapper:BaseWrapper,paths ,path_prob=0.2,std_dev=10):
        """
        Heuristic _sampler that samples from the paths
        :param child_sampler: child _sampler - that samples
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

        self._last_path_idx = None
        self._path_done = False
        #analytics
        self._queries = 0
        self._non_heur_queries = 0


    def create_wrapper(self):
        if self._wrapper is None:
            self._wrapper = HeuristicWrapper(self._child_wrapper,self.update_cur_path)
        return self._wrapper

    def sample(self):
        self._queries += 1
        if self.rng.random() < self._path_prob and len(self._paths) >0 and not self._path_done:
            path_idx = self.rng.integers(0,len(self._paths))
            path = self._paths[path_idx]
            point = path[self._cur_path_point[path_idx]]
            x,y = self.rng.normal(point,[self.std,self.std],2)
            self._last_path_idx = path_idx
            angle = None
            # if self._cur_path_point[path_idx] != 0:
            #     vec = path[self._cur_path_point[path_idx]-1] - np.array((x,y))
            #     # print("vec: ", vec)
            #     angle = np.atan2(vec[0], vec[1])

            return self._child_sampler.sample(x,y,angle)
        self._last_path_idx = None
        self._non_heur_queries += 1
        return self._child_sampler.sample()

    def update_cur_path(self,node):
        if node.reached and self._last_path_idx is not None:
            print("REACHED: ", self._cur_path_point[self._last_path_idx], "/", len(self._paths[self._last_path_idx]))
            self._cur_path_point[self._last_path_idx]  +=1
            if self._cur_path_point[self._last_path_idx] >= len(self._paths[self._last_path_idx]):
                print("Full path explored")
                self._path_done = True
        # print("pathnum: ", len(self._paths))
    def last_chpoint(self):
        if self._last_path_idx is None:
            return (0,0)
        idx = self._cur_path_point[self._last_path_idx]
        return self._paths[self._last_path_idx][idx]

    def get_paths(self):
        return self._paths

    def analytics(self):
        return {
            "path_lengths": [len(p) for p in self._paths],
            "checkpoints_reached": self._cur_path_point,
            "queries": self._queries,
            "non_heur_queries": self._non_heur_queries,
            "child_analytics": self._child_sampler.analytics()
        }





class HeuristicWrapper(BaseWrapper):
    def __init__(self, wrapper:BaseWrapper,sampler_clb):
        """
        2nd order wrapper used to wrap another _storage wrapper
        It is used to help _sampler track the progress
        """
        super().__init__()
        self._sampler_clb = sampler_clb
        self._wrapper = wrapper
        self.want_next_iter = True

    def save_to_storage(self,node):
        self._sampler_clb(node)
        res = self._wrapper.save_to_storage(node)
        self.want_next_iter = self._wrapper.want_next_iter
        return res


    def get_nearest(self,point):
        return self._wrapper.get_nearest(point)

    def get_path(self):
        return self._wrapper.get_path()

    def get_all_nodes(self):
        return self._wrapper.get_all_nodes()
