import pymunk
import numpy as np
import random

from deform_plan.assets.PM import *

class RandomBlock(PMSingleBodyObject):
    def __init__(self, pos:np.array, r:float, body_type, seed=None):
        super().__init__(body_type=body_type)
        # self.position = pos
        # self.orientation = 0
        # self.velocity = np.array([0, 0])
        # self.angular_velocity = 1
        # self.color = (0, 255, 0, 255)
        # self.r = r
        # random.seed(seed)
        # vertices = [self._get_vertex() for _ in range(8)]
        # # shape_dummy = pymunk.Poly(None, vertices)
        # # tx,ty =shape_dummy.center_of_gravity
        # # print(tx,ty)
        # w, h = 100, 200
        # vs = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
        # shape = pymunk.Poly(None, vs)
        # shape.body = self.body
        # print(shape.center_of_gravity)
        # self.body.collision_type = 2
        # self.shapes.add(shape)
        # self.body = pymunk.Body(mass=0, moment=0, body_type=STATIC)
        self.position = pos
        # self.shapes = []
        self.r = r
        random.seed(seed)
        vertices = [self._get_vertex() for _ in range(8)]
        shape = pymunk.Poly(self.body, vertices)
        shape.collision_type = 2
        self.shapes.append(shape)
    # def set_ID(self,ID,moveable):
    #     if moveable:
    #         self.body.moveId = ID
    #     else:
    #         self.body.fixedId = ID

    # def add_to_space(self, space):
    #     self.shape.density=0.01
    #     space.add(self.body, self.shape)


    def _get_vertex(self):
        r = random.random() * self.r
        # r = self.r
        theta = random.random() * 2 * 3.141592
        return r * np.cos(theta), r * np.sin(theta)