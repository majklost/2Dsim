import pymunk
import pygame


# prepared classes for the objects in the simulation
# IMPORTANT
# for agent and obstacle, each agent shape has collision type 1, each obstacle shape has collision type 2
# this is important for working with simulated dynamic planner

class AbstractObj:
    def __init__(self, x, y, body_type, mass=1, moment=100):
        self.body = pymunk.Body(mass, moment, body_type=body_type)
        self.body.position = x, y
        self.shape = None

    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape)

    def set_body_type(self, body_type):
        self.body.body_type = body_type


class Agent(AbstractObj):
    def __init__(self, x, y, w=10, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = 1


class Obstacle(AbstractObj):
    def __init__(self, x=450, y=400, w=700, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.STATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("red")
        self.shape.collision_type = 2


# body is formed from two shapes
class Cross(AbstractObj):
    def __init__(self, x, y, w, av):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angular_velocity = av
        self.shape1 = pymunk.Segment(self.body, (-w, 0), (w, 0), 10)
        self.shape2 = pymunk.Segment(self.body, (0, -w), (0, w), 10)
        self.shape1.collision_type = self.shape2.collision_type = 2

    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape1, self.shape2)
