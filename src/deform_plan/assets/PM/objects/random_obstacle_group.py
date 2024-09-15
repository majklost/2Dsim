import numpy as np
import pymunk

from .pm_multibody import PMMultiBodyObject
from .random_block import RandomBlock
class RandomObstacleGroup(PMMultiBodyObject):
    def __init__(self,pos:np.array,
                 VSep:int,
                 HSep:int,
                 VNum:int,
                 HNum:int,
                 btype=pymunk.Body.STATIC,
                 radius=150,
                 seed=None):
        super().__init__()
        self.VSep = VSep
        self.HSep = HSep
        self.VNum = VNum
        self.HNum = HNum
        self._btype = btype
        self._position = pos
        self.radius = radius
        self.seed = seed
        self._create_obstacle_group()

    def _create_obstacle_group(self):
        if self.seed is not None:
            np.random.seed(self.seed)
        for i in range(self.VNum):
            for j in range(self.HNum):
                r = RandomBlock(self.position + np.array([i * self.VSep, j * self.HSep]), self.radius, self._btype, seed=np.random.randint(0, 100000))
                self.append(r)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, pos):
        self._position = pos