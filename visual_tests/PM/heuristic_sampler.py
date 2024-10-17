from matplotlib import pyplot as plt
import numpy as np

from deform_plan.samplers.heuristic_sampler import HeuristicSampler


class DummySampler:
    def __init__(self):
        pass

    def sample(self,x=None,y=None):
        if x is None or y is None:
            raise ValueError("x and y must be provided")
        return x,y
class DummyWrapper:
    def save_to_storage(self,point):
        pass


paths = [[[1,1],[2,2],[3,3]]]
ds = DummySampler()
dw = DummyWrapper()



sampler = HeuristicSampler(ds,dw,paths,1,1)
sampled_points = []
for i in range(1000):
    sampled_points.append(sampler.sample())
paths = np.array(paths)

sampled_points = np.array(sampled_points)
print(sampled_points.shape)
plt.scatter(sampled_points[:,0],sampled_points[:,1],s=1)
plt.scatter(paths[0][0],paths[0][1],c='r',s=100)
plt.show()
