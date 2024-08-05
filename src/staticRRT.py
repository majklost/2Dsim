from src.helpers.kd_tree import KD_Tree, Node

# not written so generally will need refactoring
# TODO refactor to be more general
import random
from src.staticLocalPlanner import LocalPlanner
from src.RRTNode import RRTNode


class RRT:
    def __init__(self, xMax, yMax, angleMax, local_planner: LocalPlanner, near_radius=10):
        random.seed(10)
        self.xMax = xMax
        self.yMax = yMax
        self.angleMax = angleMax
        self.local_planner = local_planner
        self.vertTree = None
        self.node_cnt = 0
        self.last = None
        self.near_radius = near_radius

    @staticmethod
    def dist(a: RRTNode, b: RRTNode):
        return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.angle - b.angle) ** 2) ** 0.5

    def rand_conf(self) -> RRTNode:
        x = random.uniform(0, self.xMax)
        y = random.uniform(0, self.yMax)
        angle = random.uniform(0, self.angleMax)
        return RRTNode(x, y, angle, None)

    def check_n_add(self, checkpoints, goal):
        for n in checkpoints:
            # for pretty rendering only
            n.added_cnt = self.node_cnt
            self.node_cnt += 1
            self.vertTree = KD_Tree.insert(self.vertTree, n)
            if self.dist(n, goal) < self.near_radius:
                ls = self.local_planner.check_path(n, goal)
                if self.dist(ls[-1], goal) < 1e-1:
                    # self.last = ls[-1]
                    goal.parent = n
                    self.last = goal
                    print("Goal reached")
                    return True

        return False

    def find_path(self, start: RRTNode, goal: RRTNode, iters=10000):
        self.vertTree = Node(start)
        for i in range(iters):
            q_rand = self.rand_conf()
            q_near = KD_Tree.nearestNeighbour(self.vertTree, q_rand, distancefnc=self.dist)
            checkpoints = self.local_planner.check_path(q_near, q_rand)
            if self.check_n_add(checkpoints, goal):
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
