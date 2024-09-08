
import numpy as np

from .pm_singlebody import PMSingleBodyObject
from ..configs.pymunk_env_cfg import *

class Cross(PMSingleBodyObject):
    """
    A cross that can rotate around its center.
    """
    def __init__(self,
                 pos:np.array,
                 w:float,
                 radius:float,
                 body_type):
        super().__init__(body_type=body_type)
        shape1 = pymunk.Segment(self.body, (-w, 0), (w, 0), radius)
        shape1.density = 1
        shape2 = pymunk.Segment(self.body, (0, -w), (0, w), radius)
        shape2.density = 1
        self.shapes = [shape1, shape2]
        self.position = pos

