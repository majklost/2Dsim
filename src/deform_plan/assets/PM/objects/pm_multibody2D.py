from typing import Tuple

from .pm_multibody import PMMultiBodyObject

class PMMultibody2D(PMMultiBodyObject):
    def __init__(self):
        super().__init__()
        self.bodies = [[]]


    def link_body(self, body,index: Tuple[int,...]):
        if len(index) ==2:
            self.bodies[index[0]][index[1]].collision_data = None
            self.bodies[index[0]][index[1]].body = body
        else:
            raise NotImplementedError("Linking 2D bodies not implemented")

    def get_body(self, index: Tuple[int,...]):
        return self.bodies[index[0]][index[1]]