from src.cable_planner import CablePlanner
from src.helpers.goalspecifier import GoalSpecifier

import numpy as np

class RRT:
    def __init__(self,xMax,yMax,localPlanner: CablePlanner,goal_region: GoalSpecifier,seed=10):
        self.xmax = xMax
        self.ymax = yMax
        self.localPlanner = localPlanner
        self.goal_region = goal_region
        np.random.seed(seed)








