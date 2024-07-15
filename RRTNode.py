#Data sturcture for communication between RRT and LocalPlanner
#Similar to vec2D but I want to be independent of pymunk
from math import isclose

CLOSE = 1e-1
class RRTNode:
    def __init__(self, x, y, angle, parent=None, added_cnt=0):
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = parent
        self.added_cnt = added_cnt

    def __eq__(self, other):
        return isclose(self.x, other.x,abs_tol=CLOSE) and isclose(self.y, other.y,abs_tol=CLOSE) and isclose(self.angle, other.angle,abs_tol=CLOSE)

    def __str__(self):
        return f"Node: {self.x}, {self.y}, {self.angle}"

    def __sub__(self, other):
        return self.x - other.x, self.y - other.y, self.angle - other.angle

    def __repr__(self):
        return self.__str__()

    def get_pos(self):
        return self.x, self.y

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        elif index == 2:
            return self.angle
        else:
            raise IndexError("Index out of bounds")

    def get_comparable_point(self):
        return self.x, self.y, self.angle
