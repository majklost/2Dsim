import numpy as np
import pymunk

from typing import List
from dataclasses import dataclass


from deform_plan.assets.PM import PMSingleBodyObject,PMMultiBodyObject, PMConfig
from ..base_simulator import Simulator as BaseSimulator, BaseSimulatorExport
from .collision_data import CollisionData

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
        self.movable_objects = movable_objects #overload so that the type checker does not complain
        self.fixed_objects = fixed_objects
        self._space = pymunk.Space()

        self._process_config(config)
        self._add_objects_to_space()
        self._collision_handling()
        self._steps = 0
        self._fingerprint = random.randint(0, 1000000)
        self.debugger = None  # For PM_debug_viewer

    @property
    def damping(self):
        return self._space.damping

    @damping.setter
    def damping(self, value):
        self._space.damping = value


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
    def _begin_collision(self, arbiter:pymunk.Arbiter, space, data):
        b1 = arbiter.shapes[0]
        b2 = arbiter.shapes[1]
        print("Collision detected")

        o1 = self._identify_object(b1.body)
        o2 = self._identify_object(b2.body)
        data1 = CollisionData(arbiter.normal, b2, o2)
        data2 = CollisionData(-arbiter.normal, b1, o1)
        o1.collision_data = data1
        o2.collision_data = data2

        return True

    def _end_collision(self, arbiter, space, data):
        b1 = arbiter.shapes[0]
        b2 = arbiter.shapes[1]
        print("Collision ended")

        o1 = self._identify_object(b1.body)
        o2 = self._identify_object(b2.body)
        if o1.collision_data is not None and o1.collision_data.stamp == o2.collision_data.stamp:
            o1.collision_data = None
        if o2.collision_data is not None and o2.collision_data.stamp == o2.collision_data.stamp:
            o2.collision_data = None




    def _identify_object(self, body):
        if hasattr(body, 'moveId'):
            cid = body.moveId
            if cid is tuple:
                return self.movable_objects[cid[0]][cid[1]]
            else:
                return self.movable_objects[cid]
        elif hasattr(body, 'fixedId'):
            cid = body.fixedId
            if cid is tuple:
                return self.fixed_objects[cid[0]][cid[1]]
            else:
                return self.fixed_objects[cid]
        raise ValueError("Object with no ID")

    def _collision_handling(self):
        handler = self._space.add_default_collision_handler()
        handler.begin = self._begin_collision
        handler.separate = self._end_collision


    def step(self):
        """
        Make a step in the simulation
        :return:
        """
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
        """
        Exports data from the simulator so it can be reused later
        :return:
        """
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
        """
        Useful when PM_debug_viewer is attached
        :return:
        """
        return self._width, self._height, self._FPS, self._space

    # def get_force_template(self):
    #     return [b.get_force_template() for b in self.movable_objects]




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





