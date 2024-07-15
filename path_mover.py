import pymunk
from typing import List
from RRTNode import RRTNode
#Move shape on path given by nodes
VELOCITY = 100

class PathMover:
    def __init__(self, path : List[RRTNode], body : pymunk.Body, FPS):
        self.path = path
        self.current = 0 # current waypoint trying to reach
        self.body = body
        self.threshdist = VELOCITY /2 * 1/FPS # distance to waypoint to consider it reached

    def move(self):
        if self.current >= len(self.path):
            self.body.velocity = 0,0
            self.body.angular_velocity = 0
            return
        target = self.path[self.current]
        dir = target.x - self.body.position.x, target.y - self.body.position.y
        plength = self.get_length(dir)
        angle = target.angle - self.body.angle
        if plength < self.threshdist:
            self.current += 1
        else:
            dir = dir[0]/ plength*VELOCITY, dir[1]/ plength*VELOCITY
            self.body.velocity = dir
        self.body.angular_velocity = angle * VELOCITY / plength

    @staticmethod
    def get_length(tp):
        return (tp[0] ** 2 + tp[1] ** 2) ** 0.5