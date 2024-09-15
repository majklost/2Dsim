import pymunk
import numpy as np


from .pm_multibody import PMMultiBodyObject
from .rectangle import Rectangle

class SpringParams:
    def __init__(self, stiffness, damping):
        self.stiffness = stiffness
        self.damping = damping

STANDARD_LINEAR_PARAMS = SpringParams(5000, 10)
STANDARD_ROTARY_PARAMS = SpringParams(3000, 10)

class Cable(PMMultiBodyObject):
    def __init__(self, pos:np.array, length:float, num_links:int, thickness:int=2,
                 linear_params:'SpringParams'=STANDARD_LINEAR_PARAMS,
                 rotary_params:'SpringParams'=STANDARD_ROTARY_PARAMS
                 ):
        super().__init__()
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

        self._create_cable(pos)



    def _create_cable(self,pos):
        segment_length = self.length / self.num_links
        for i in range(self.num_links):
            r = Rectangle(pos + np.array([i * segment_length, 0]), segment_length, self.thickness, pymunk.Body.DYNAMIC)
            self.append(r)
        for i in range(self.num_links - 1):
            spring = pymunk.constraints.DampedSpring(self.bodies[i].body, self.bodies[i + 1].body, (0.3 * segment_length, 0),
                                                     (-0.3 * segment_length, 0), 0.4 * segment_length,
                                                     self.linear_params.stiffness / segment_length,
                                                     self.linear_params.damping)
            self.linear_springs.append(spring)

        for i in range(self.num_links - 1):
            spring = pymunk.constraints.DampedRotarySpring(self.bodies[i].body, self.bodies[i + 1].body, 0, self.rotary_params.stiffness, self.rotary_params.damping)
            self.angular_springs.append(spring)



    def add_to_space(self, space):
        super().add_to_space(space)
        space.add(*self.linear_springs)
        space.add(*self.angular_springs)


    @property
    def position(self):
        """Returns position CoM of all segments"""
        return np.mean([b.position for b in self.bodies], axis=0)


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












