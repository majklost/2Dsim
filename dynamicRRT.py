from kd_tree import KD_Tree,Node


import random
from dynamicLocalPlanner import LocalPlannerCalc
from RRTNode import RRTNodeCalc


class RRT:
    def __init__(self,xMax,yMax,angleMax,maxTime, local_planner: LocalPlannerCalc,near_radius=10,seed=10):
        random.seed(seed)
        self.xMax = xMax
        self.yMax = yMax
        self.angleMax = angleMax
        self.maxTime = maxTime
        self.local_planner = local_planner
        self.vertTree = None
        self.node_cnt = 0
        self.last = None
        self.near_radius = near_radius
        KD_Tree.k = 4

    #now with time, distance is not symmetric
    @staticmethod
    def dist(candidate:RRTNodeCalc, root:RRTNodeCalc):
        #impossible to reach
        if candidate.time < root.time:
            return float("inf")
        else:
            return ((candidate.x - root.x) ** 2 + (candidate.y - root.y) ** 2 + 0.01*(candidate.angle - root.angle) ** 2) ** 0.5

    def random_conf(self)-> RRTNodeCalc:
        x = random.uniform(0, self.xMax)
        y = random.uniform(0, self.yMax)
        angle = random.uniform(0, self.angleMax)
        time = random.uniform(0,self.maxTime)
        return RRTNodeCalc(x, y, angle, time, None)

    def check_n_add(self,checkpoints,goal):
        for n in checkpoints:
            #for pretty rendering only
            n.added_cnt = self.node_cnt
            self.node_cnt += 1
            self.vertTree = KD_Tree.insert(self.vertTree, n)
            if self.dist(goal, n) < self.near_radius:
                ls = self.local_planner.check_path(n, goal)
                if ls[-1] == goal:
                    pass
                # self.last = ls[-1]
                goal.parent = n
                self.last = goal
                print("Goal reached")
                return True

        return False

    def find_path(self,start:RRTNodeCalc,goal:RRTNodeCalc, iters=10000):
        self.vertTree = Node(start)
        for i in range(iters):
            q_rand = self.random_conf()
            q_near = KD_Tree.nearestNeighbour(self.vertTree, q_rand,distancefnc=self.dist)
            checkpoints = self.local_planner.check_path(q_near, q_rand)
            if self.check_n_add(checkpoints, goal):
                print(f"Goal reached in {i} iterations")
                return self.get_path()
            if i % 100 == 0:
                print(f"Iteration {i}")
        print("Goal not reached")
        return []

    def get_path(self):
        path = []
        goal = self.last
        while goal is not None:
            path.append(goal)
            goal = goal.parent
        return list(reversed(path))

    def get_verts(self):
        return self.vertTree