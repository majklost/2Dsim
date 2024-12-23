import pymunk
import numpy as np


from .pm_multibody import PMMultiBodyObject
from .rectangle import Rectangle
from ....utils.math_utils import rot_matrix

class SpringParams:
    def __init__(self, stiffness, damping):
        self.stiffness = stiffness
        self.damping = damping

STANDARD_LINEAR_PARAMS = SpringParams(2500, 10)
STANDARD_ROTARY_PARAMS = SpringParams(200, 10)

class Cable(PMMultiBodyObject):
    def __init__(self, pos:np.array, length:float, num_links:int, thickness:int=2,
                 linear_params:'SpringParams'=STANDARD_LINEAR_PARAMS,
                 rotary_params:'SpringParams'=STANDARD_ROTARY_PARAMS,
                 track_colisions=True,
                 angle=0
                 ):
        super().__init__(track_colisions=track_colisions)
        self.length = length
        self.num_links = num_links
        self.thickness = thickness
        self.linear_params = linear_params
        self.rotary_params = rotary_params
        self.linear_springs = []
        self.angular_springs = []
        self.density = 0.005
        self.friction = 0.5
        self.color = (0, 0, 255, 0)
        self.segment_length = self.length / self.num_links
        self.angle = angle
        self._create_cable(pos)




    def _create_cable(self,pos):
        self._create_objects(pos)
        self._create_linear_springs()
        self._create_angular_springs()



    def _create_objects(self,pos):
        rm =rot_matrix(self.angle)
        # positions = np.array([pos + np.array([i * self.segment_length, 0]) for i in range(self.num_links)])



        for i in range(self.num_links):
            r = Rectangle(pos + rm @ np.array([i * self.segment_length, 0]), self.segment_length, self.thickness,
                          pymunk.Body.DYNAMIC)
            r.orientation = self.angle
            self.append(r)

    def _create_linear_springs(self):

        for i in range(self.num_links - 1):
            spring = pymunk.constraints.DampedSpring(self.bodies[i].body, self.bodies[i + 1].body, (0.3 * self.segment_length, 0),
                                                     (-0.3 * self.segment_length, 0), 0.4 * self.segment_length,
                                                     self.linear_params.stiffness,
                                                     self.linear_params.damping)
            self.linear_springs.append(spring)

    def _create_angular_springs(self):
        for i in range(self.num_links - 1):
            spring = pymunk.constraints.DampedRotarySpring(self.bodies[i].body, self.bodies[i + 1].body, 0,
                                                           self.rotary_params.stiffness, self.rotary_params.damping)
            self.angular_springs.append(spring)

    def add_to_space(self, space):
        super().add_to_space(space)
        space.add(*self.linear_springs)
        space.add(*self.angular_springs)


    @property
    def position(self):
        """Returns position of all segments"""
        return np.array([b.position for b in self.bodies])


    @property
    def orientation(self):

        raise NotImplementedError

    @property
    def velocity(self):
        """Returns sum of velocities of all segments"""
        return sum([np.linalg.norm(b.velocity) for b in self.bodies])

    @property
    def angular_velocity(self):
        raise NotImplementedError












