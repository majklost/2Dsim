import numpy as np
import pymunk

from  .cable import Cable,STANDARD_LINEAR_PARAMS,STANDARD_ROTARY_PARAMS,SpringParams
from .rectangle import Rectangle

class OneEndCable(Cable):
    def __init__(self, pos:np.array, length:float, num_links:int, thickness:int=2,
                 linear_params:'SpringParams'=STANDARD_LINEAR_PARAMS,
                 rotary_params:'SpringParams'=STANDARD_ROTARY_PARAMS,
                 track_colisions=True
                 ):
        super().__init__(pos, length, num_links, thickness, linear_params, rotary_params, track_colisions)


    def _create_objects(self,pos):
        for i in range(self.num_links):
            if i == 0:
                r = Rectangle(pos + np.array([i * self.segment_length, 0]), self.segment_length, self.thickness*10,
                          pymunk.Body.DYNAMIC)
                r.density /= 10
            else:
                r = Rectangle(pos + np.array([i * self.segment_length, 0]), self.segment_length, self.thickness,
                          pymunk.Body.DYNAMIC)
            self.append(r)


