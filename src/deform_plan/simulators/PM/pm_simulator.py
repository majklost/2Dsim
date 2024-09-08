import pymunk

from typing import List
from dataclasses import dataclass


from deform_plan.assets.PM import PMSingleBodyObject,PMMultiBodyObject, PMConfig
from ..base_simulator import Simulator as BaseSimulator, BaseSimulatorExport

import random


class Simulator(BaseSimulator):

    def __init__(self,
                 config: PMConfig,
                 movable_objects: List[PMSingleBodyObject| PMMultiBodyObject],
                 fixed_objects: List[PMSingleBodyObject| PMMultiBodyObject],
                 ):
        super().__init__(movable_objects, fixed_objects)
        self._FPS = None
        self._height = None
        self._width = None
        self.movable_objects = movable_objects
        self.fixed_objects = fixed_objects
        self._space = pymunk.Space()
        self._process_config(config)
        self._add_objects_to_space()
        self._steps = 0
        self._fingerprint = random.randint(0, 1000000)
        self.debugger = None



    def _process_config(self, config: PMConfig):
        self._space.gravity = (0,config.gravity)
        self._space.damping = config.damping
        self._FPS = config.FPS
        self._width = config.width
        self._height = config.height

    def _add_objects_to_space(self):
        for i,obj in enumerate(self.movable_objects):
            obj.set_ID(i,moveable=True)
            obj.add_to_space(self._space)
        for i,obj in enumerate(self.fixed_objects):
            obj.set_ID(i,moveable=False)
            obj.add_to_space(self._space)

    def step(self):
        self._space.step(1/self._FPS)
        self._steps += 1
        if self.debugger is not None:
            self.debugger.update_cur(self._space)
            if self.debugger.want_end():
                return True

        return False

    def apply_forces(self, forces: List, indexes: List[int]=None):
        """
        Apply forces to movable objects in the simulator
        :param forces: forces to apply to the objects
        :param indexes: indexes of the objects to apply forces to
        :return:
        """
        pass



    def import_from(self, simulator: 'PMExport'):
        """
        Create a simulator object from the exported simulator
        :param simulator:
        :return: Simulator object
        """
        if self._fingerprint != simulator.sim_fingerprint:
            raise ValueError("Simulator fingerprint does not match")

        self._space = simulator.space
        self._steps = simulator.steps
        self._width = simulator.width
        self._height = simulator.height
        self._FPS = simulator.FPS
        self._collect_objects()
        if simulator.movable_data is not None or simulator.fixed_data is not None:
            raise NotImplementedError("Importing of the data is not implemented yet")



    def export(self)->'PMExport':
        return PMExport(self._space.copy(), self._steps, self._width, self._height, self._FPS, self._fingerprint)

    def _collect_objects(self):
        for b in self._space.bodies:
            if hasattr(b, 'moveId'):
                cid = b.moveId
                if cid is tuple:
                    self.movable_objects[cid[0]][cid[1]].body = b
                else:
                    self.movable_objects[cid].body = b

            if hasattr(b, 'fixedId'):
                cid = b.fixedId
                if cid is tuple:
                    self.fixed_objects[cid[0]][cid[1]].body = b
                else:
                    self.fixed_objects[cid].body = b

    def get_debug_data(self):
        return self._width, self._height, self._FPS




@dataclass
class PMExport(BaseSimulatorExport):
    space: pymunk.Space
    steps: int
    width: int
    height: int
    FPS: int
    sim_fingerprint: int
    movable_data: List =None #TODO: Implement parsing of the data
    fixed_data: List =None



