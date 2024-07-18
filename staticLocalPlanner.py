# this should yield new nodes to RRT
# we give initial position and goal position to it, planner tries to morph between these two
# if it encounters an obstacle, it stops
# during the process, it adds new nodes to the tree
import pymunk  # uses pymunk for checking validity of path
from pymunk import Vec2d
from RRTNode import RRTNode
from typing import List
#TODO outsource granularity of checkpoint creation and num steps

MINSTEPS = 100
class LocalPlanner:
    def __init__(self, space: pymunk.Space, shape: pymunk.Shape):
        self.space = space
        self.shape = shape
        print("Local planner initialized")

    @staticmethod
    def extend_checkpoints(start: RRTNode, checkpoints: List[RRTNode], pos: Vec2d, angle):
        if len(checkpoints) == 0:
            checkpoints.append(RRTNode(pos.x, pos.y, angle, start))
        else:
            checkpoints.append(RRTNode(pos.x, pos.y, angle, checkpoints[-1]))

    def check_path(self, start: RRTNode, goal: RRTNode) -> List[RRTNode]:
        old_pos = self.shape.body.position
        old_angle = self.shape.body.angle
        # print(f"Old pos: {old_pos}")
        checkpoints = []
        direction = goal.x - start.x, goal.y - start.y
        angle_morph = goal.angle - start.angle
        num_steps = max(int((direction[0] ** 2 + direction[1] ** 2) / 1000), MINSTEPS)
        angle_step = angle_morph / num_steps
        # print(f"Num steps: {num_steps}")
        # print(f"Angle morph: {angle_morph}")
        for i in range(num_steps):
            self.shape.body.position = start.x + direction[0] * i / num_steps, start.y + direction[1] * i / num_steps
            self.shape.body.angle = start.angle + angle_step * i
            self.space.reindex_shape(self.shape)
            if self.space.shape_query(self.shape):
                # print("Obstacle encountered")
                break
            if (i % 20 == 0 and i != 0) or i == num_steps - 1:
                self.extend_checkpoints(start, checkpoints, self.shape.body.position, self.shape.body.angle)
        self.shape.body.angle = old_angle
        self.shape.body.position = old_pos
        self.space.reindex_shape(self.shape)
        return checkpoints

    def list_bodies(self):
        for b in self.space.bodies:
            print(b)

    @staticmethod
    def node_from_shape( shape: pymunk.Shape):
        return RRTNode(shape.body.position.x, shape.body.position.y, shape.body.angle, None)


