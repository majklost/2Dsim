import pymunk
import pygame
import pymunk.constraints
import random
import math

# prepared classes for the objects in the simulation
# IMPORTANT
# for agent and obstacle, each agent shape has collision type 1, each obstacle shape has collision type 2
# this is important for working with simulated dynamic planner

class AbstractObj:
    def __init__(self, x, y, body_type, mass=0, moment=0):
        self.body = pymunk.Body(mass, moment, body_type=body_type)
        self.body.position = x, y
        self.shapes = []

    def add(self, space: pymunk.Space):
        space.add(self.body)
        for shape in self.shapes:
            space.add(shape)

    @property
    def shape(self):
        return self.shapes[0]

    @shape.setter
    def shape(self, value):
        if self.shapes:
            self.shapes[0] = value
        else:
            self.shapes.append(value)

    def set_body_type(self, body_type):
        self.body.body_type = body_type


class Agent(AbstractObj):
    def __init__(self, x, y, w=10, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.mass = 1
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = 1


class Obstacle(AbstractObj):
    def __init__(self, x=450, y=400, w=700, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.STATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.mass = 1
        self.shape.color = pygame.Color("red")
        self.shape.collision_type = 2


# body is formed from two shapes
class Cross(AbstractObj):
    def __init__(self, x, y, w, av):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angular_velocity = av
        self.shape1 = pymunk.Segment(self.body, (-w, 0), (w, 0), 10)
        self.shape1.mass = 1
        self.shape2 = pymunk.Segment(self.body, (0, -w), (0, w), 10)
        self.shape2.mass = 1
        self.shape1.collision_type = self.shape2.collision_type = 2

    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape1, self.shape2)


class RandomBlock(AbstractObj):
    """
    Generates a random block by picking 8 points in radius r from given center
    """
    def __init__(self,x,y,r,seed=2):
        super().__init__(x, y, pymunk.Body.STATIC)
        self.r = r
        random.seed(seed)
        vertices = [self._get_vertex() for _ in range(8)]
        self.shape = pymunk.Poly(self.body, vertices)
        self.shape.collision_type = 2

    def _get_vertex(self):
        r = random.random() * self.r
        theta = random.random() * 2 * 3.141592
        return r * math.cos(theta), r * math.sin(theta)
