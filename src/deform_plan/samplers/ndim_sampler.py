"""given n upper and lower bounds, sample uniformly in the n-dimensional box"""
import numpy as np

class NDIMSampler:
    def __init__(self, lower_bounds:np.array, upper_bounds:np.array, seed=None):
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        self.ndim = len(lower_bounds)
        self.ranges = np.array(upper_bounds) - np.array(lower_bounds)
        if seed is not None:
            np.random.seed(seed)
    def sample(self):
        return np.random.rand(self.ndim) * self.ranges + self.lower_bounds