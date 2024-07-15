import pymunk
import pygame
class AbstractObj:
    def __init__(self,x,y,body_type,mass=1,moment=100):
        self.body = pymunk.Body(mass, 100, body_type=body_type)
        self.body.position = x, y
        self.shape = None
    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape)


class Agent(AbstractObj):
    def __init__(self, x, y,w=10,h=100, angle=0):
        super().__init__(x,y,pymunk.Body.DYNAMIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = 1

class Obstacle(AbstractObj):
    def __init__(self, x=450, y=400, w=700, h=100,angle=0):
        super().__init__(x,y,pymunk.Body.STATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("red")
        self.shape.collision_type = 2
