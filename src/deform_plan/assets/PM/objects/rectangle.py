import pymunk
import numpy as np


from .pm_singlebody import PMSingleBodyObject
class Rectangle(PMSingleBodyObject):
    def __init__(self,
                 pos:np.array,
                 w:float,
                 h:float,
                 body_type):
        super().__init__(body_type=body_type)
        shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shapes = [shape]
        self.position = pos
