# a 2nd order _sampler that accepts another _sampler and wraps goal_points biasing around it

import numpy as np

from deform_plan.samplers.base_sampler import BaseSampler
from deform_plan.helpers.seed_manager import manager

class GoalBiasSampler(BaseSampler):
    def __init__(self,sampler, goal_points,goal_bias=0.01, verbose=False):
        """

        :param sampler: _sampler wrapped by this
        :param goal_points: goal_points to bias towards
        :param goal_bias:  probability of sampling the goal_points
        :param verbose: print goal_points biasing
        """
        super().__init__()
        self.sampler = sampler
        self.goal_bias = goal_bias
        self.goal_points = goal_points
        self.verbose = verbose
        self.rng = np.random.default_rng(manager().get_seed(self.__class__.__name__))
        self.bias_cnt = 0
        self.unbias_cnt = 0

    def sample(self,x=None,y=None,angle=None):
        if self.rng.random() < self.goal_bias:
            self.bias_cnt += 1
            if self.verbose:
                print("Goal biasing")
            return self.goal_points
        self.unbias_cnt += 1
        return self.sampler.sample(x,y,angle)

    def analytics(self):
        return {
            "name": self.__class__.__name__,
            "bias_cnt":self.bias_cnt,
            "unbias_cnt":self.unbias_cnt,
            "inner_sampler":self.sampler.analytics()
                }
