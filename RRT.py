from kd_tree import Node, insert, nearestNeighbour

#not written so generally will need refactoring
#TODO refactor to be more general
#TODO use KD-tree
import random
from staticLocalPlanner import LocalPlanner
from RRTNode import RRTNode


class RRT:
    def __init__(self, xMax,yMax,angleMax, local_planner: LocalPlanner,near_radius=10):
        # random.seed(10)
        self.xMax = xMax
        self.yMax = yMax
        self.angleMax = angleMax
        self.local_planner = local_planner
        self.vertTree = None
        self.node_cnt = 0

        self.near_radius = near_radius

    def dist(self, a:RRTNode, b:RRTNode):
        return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.angle - b.angle) ** 2) ** 0.5

    def rand_conf(self) -> RRTNode:
        x = random.uniform(0, self.xMax)
        y = random.uniform(0, self.yMax)
        angle = random.uniform(0, self.angleMax)
        return RRTNode(x, y, angle, None)
    def check_n_add(self,checkpoints,goal):
        for n in checkpoints:
            #for pretty rendering only
            n.added_cnt = self.node_cnt
            self.node_cnt += 1
            self.vertTree = insert(self.vertTree, n)
            if self.dist(n, goal) < self.near_radius:
                print("Goal reached")
                return True

        return False


    def find_path(self, start:RRTNode, goal:RRTNode, iters=10000):
        self.vertTree = Node(start)
        for i in range(iters):
            q_rand = self.rand_conf()
            q_near = nearestNeighbour(self.vertTree, q_rand)
            checkpoints = self.local_planner.check_path(q_near, q_rand)
            if self.check_n_add(checkpoints, goal):
                return self.get_verts()
            if i % 100 == 0:
                print(f"Iteration {i}")
        print("Goal not reached")
        return self.get_verts()


    def get_verts(self):
        return self.vertTree


