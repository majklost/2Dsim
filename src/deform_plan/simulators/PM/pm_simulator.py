
import pymunk
from pymunk.pygame_util import DrawOptions
import random
from typing import List
from dataclasses import dataclass
import time


from deform_plan.assets.PM import PMSingleBodyObject,PMMultiBodyObject, PMConfig
from ..base_simulator import Simulator as BaseSimulator, BaseSimulatorExport
from .collision_data import CollisionData

def placer_start(a,s,d):
    return d['obj']._begin_collision(a, s, d)

def placer_end(a,s,d):
    return d['obj']._end_collision(a, s, d)




class Simulator(BaseSimulator):

    def __init__(self,
                 config: PMConfig,
                 movable_objects: List[PMSingleBodyObject| PMMultiBodyObject],
                 fixed_objects: List[PMSingleBodyObject| PMMultiBodyObject],
                 threaded=False
                 ):
        super().__init__(movable_objects, fixed_objects)
        self.movable_objects : List[PMSingleBodyObject| PMMultiBodyObject]
        self._FPS = None
        self._height = None
        self._width = None
        self.movable_objects = movable_objects #overload so that the type checker does not complain
        self.fixed_objects = fixed_objects
        self._space = pymunk.Space(threaded=threaded)

        self.SIMTIME = 0
        self._process_config(config)
        self._add_objects_to_space()
        self._collision_handling()
        self._steps = 0
        self._fingerprint = random.randint(0, 1000000)
        self.debuggerclb = None  # For PM_debug_viewer

    @property
    def damping(self):
        return self._space.damping

    @damping.setter
    def damping(self, value):
        self._space.damping = value

    @property
    def fps(self):
        return self._FPS

    def _process_config(self, config: PMConfig):
        self._space.gravity = (0,config.gravity)
        self._space.damping = config.damping
        # self._space.co
        self._FPS = config.FPS
        self._width = config.width
        self._height = config.height
        self._space.collision_slop = config.collision_slope

    def _add_objects_to_space(self):
        for i,obj in enumerate(self.movable_objects):
            obj.set_ID((i,),moveable=True)
            obj.add_to_space(self._space)
        for i,obj in enumerate(self.fixed_objects):
            obj.set_ID((i,),moveable=False)
            obj.add_to_space(self._space)
    def _begin_collision(self, arbiter:pymunk.Arbiter, space, data):
        b1 = arbiter.shapes[0]
        b2 = arbiter.shapes[1]
        # print("Collision detected")


        cid1,movable1 = self._get_id(b1.body)
        cid2,movable2 = self._get_id(b2.body)
        # same_object = movable1 == movable2 and cid1[0] == cid2[0]
        o1 = self._get_object(cid1, movable1)
        o2 = self._get_object(cid2, movable2)

        if o1.track_colisions:
            data1 = CollisionData(arbiter.normal, b2, o2, cid1,cid2)
            o1.collision_start(data1)
        if o2.track_colisions:
            data2 = CollisionData(-arbiter.normal, b1, o1, cid2,cid1)
            o2.collision_start(data2)

        # o1.color = (255,0,0,0)
        # o2.color = (0,255,0,0)

        return True
    def __deepcopy__(self, memodict={}):
        raise NotImplementedError("Deepcopy not implemented")

    def _end_collision(self, arbiter, space, data):
        b1 = arbiter.shapes[0]
        b2 = arbiter.shapes[1]
        # print("Collision ended")

        cid1,movable1 = self._get_id(b1.body)
        cid2,movable2 = self._get_id(b2.body)
        # same_object = movable1 == movable2 and cid1[0] == cid2[0]

        o1 = self._get_object(cid1, movable1)
        o2 = self._get_object(cid2, movable2)
        if o1.track_colisions:
            data1 = CollisionData(arbiter.normal, b2, o2, cid1,cid2)
            o1.collision_end(data1)
        if o2.track_colisions:
            data2 = CollisionData(-arbiter.normal, b1, o1, cid2,cid1)
            o2.collision_end(data2)
    def _get_object(self, cid, movable):
        if movable:
            return self.movable_objects[cid[0]]
        else:
            return self.fixed_objects[cid[0]]

    @staticmethod
    def _get_id( body):
        if hasattr(body, 'moveId'):
            cid = body.moveId
            return cid,True

        elif hasattr(body, 'fixedId'):
            cid = body.fixedId
            return cid,False

        raise ValueError("Object with no ID")

    def _collision_handling(self):
        handler = self._space.add_default_collision_handler()
        # handler.data['obj'] = self
        begin_fnc = lambda a,s,d : self._begin_collision(a,s,d)
        sep_fnc = lambda a,s,d : self._end_collision(a,s,d)
        # begin_fnc = placer_start(a,s,d,self)
        # end_fnc = placer_end(a,s,d,self)
        handler.begin = begin_fnc
        handler.separate = sep_fnc




    def step(self):
        """
        Make a step in the simulation
        :return:
        """
        # t1 = time.time()
        self._space.step(1/self._FPS)
        # t2 = time.time()
        # self.SIMTIME += t2-t1
        self._steps += 1
        if self.debuggerclb is not None:
            if self.debuggerclb(self._space):
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

    def _untested_copy(self, foreign_space:pymunk.Space):
        self._space.iterations = foreign_space.iterations
        self._space.gravity = foreign_space.gravity
        self._space.damping = foreign_space.damping
        self._space.idle_speed_threshold = foreign_space.idle_speed_threshold
        self._space.sleep_time_threshold = foreign_space.sleep_time_threshold
        self._space.collision_slop = foreign_space.collision_slop
        self._space.collision_bias = foreign_space.collision_bias
        self._space.collision_persistence = foreign_space.collision_persistence
        assert  self._space.threads == foreign_space.threads
        assert self._space.threaded == foreign_space.threaded



        for i in range(len(foreign_space.bodies)):
            self._space.bodies[i].position = foreign_space.bodies[i].position
            self._space.bodies[i].velocity = foreign_space.bodies[i].velocity
            self._space.bodies[i].angle = foreign_space.bodies[i].angle
            self._space.bodies[i].angular_velocity = foreign_space.bodies[i].angular_velocity
            self._space.bodies[i].force = foreign_space.bodies[i].force
            self._space.bodies[i].torque = foreign_space.bodies[i].torque


            # self._space.bodies[i].
            # self._space.bodies[i] = foreign_space.bodies[i].copy()
            # print(self._space.bodies[i],foreign_space.bodies[i])


    def import_from(self, simulator: 'PMExport'):
        """
        Create a simulator object from the exported simulator
        :param simulator:
        :return: Simulator object
        """
        # print(self.movable_objects[0])
        # if self._fingerprint != simulator.sim_fingerprint:
        #     raise ValueError("Simulator fingerprint does not match")


        self._space = simulator.space.copy()

        # self._untested_copy(simulator.space)
        self._steps = simulator.steps
        self._width = simulator.width
        self._height = simulator.height
        self._FPS = simulator.FPS
        for b in self.movable_objects:
            b.collision_clear()
        for b in self.fixed_objects:
            b.collision_clear()
        # print(self.movable_objects[0])
        self._collect_objects()
        if simulator.movable_data is not None or simulator.fixed_data is not None:
            raise NotImplementedError("Importing of the nodes is not implemented yet")



    def export(self)->'PMExport':
        """
        Exports nodes from the simulator so it can be reused later
        :return:
        """
        return PMExport(self._space.copy(), self._steps, self._width, self._height, self._FPS, self._fingerprint)

    def _collect_objects(self):
        for b in self._space.bodies:
            if hasattr(b, 'moveId'):
                cid = b.moveId
                self.movable_objects[cid[0]].link_body(b,cid[1:])

            else:
                cid = b.fixedId
                self.fixed_objects[cid[0]].link_body(b,cid[1:])



    def get_debug_data(self):
        """
        Useful when PM_debug_viewer is attached
        :return:
        """
        return self._width, self._height, self._FPS

    # def get_force_template(self):
    #     return [b.get_force_template() for b in self.movable_objects]
    def draw_on(self, dops: DrawOptions):
        """
        Draw the simulator on the screen
        :param dops: DrawOptions object
        :return:
        """
        self._space.debug_draw(dops)




@dataclass
class PMExport(BaseSimulatorExport):
    space: pymunk.Space
    steps: int
    width: int
    height: int
    FPS: int
    sim_fingerprint: int
    movable_data: List =None #TODO: Implement parsing of the nodes
    fixed_data: List =None





