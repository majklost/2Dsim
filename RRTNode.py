#Data sturcture for communication between RRT and LocalPlanner
#Similar to vec2D but I want to be independent of pymunk
class RRTNode:
    def __init__(self, x, y, angle, parent=None):
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = parent

    def get_pos(self):
        return self.x, self.y