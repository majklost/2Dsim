import pymunk
import numpy as np

from ...base_singlebody import BaseSingleBodyObject
from deform_plan.utils.math_utils import rot_matrix


class PMSingleBodyObject(BaseSingleBodyObject):
    def __init__(self,body_type):
        super().__init__()
        self._collision_object = None #what collided with my object
        self.colliding = False
        self.shapes = []
        self._body = pymunk.Body(body_type=body_type)
        self._color = (0, 0, 0, 0)
        self._density = .01


    @property
    def body(self) -> pymunk.Body:
        """
        Pymunk representation of the object
        :return:
        """
        return self._body

    @body.setter
    def body(self, body):
        self._body = body

    @property
    def density(self):
        return self._density

    @density.setter
    def density(self, density):
        self._density = density
        for s in self.shapes:
            s.density = density

    @property
    def position(self):
        return np.array(self._body.position)
    @position.setter
    def position(self, value:np.array):
        self._body.position = value[0], value[1]

    @property
    def orientation(self):
        return self._body.angle
    @orientation.setter
    def orientation(self, value):
        self._body.angle = value

    @property
    def velocity(self):
        return np.array(self._body.velocity)
    @velocity.setter
    def velocity(self, value):
        self._body.velocity = value[0], value[1]

    @property
    def angular_velocity(self):
        return self._body.angular_velocity
    @angular_velocity.setter
    def angular_velocity(self, value):
        self._body.angular_velocity = value

    @property
    def friction(self):
        return self._body.friction

    @friction.setter
    def friction(self, value):
        self._body.friction = value

    def collision_handler(self, arbiter, space, data):
        raise NotImplementedError

    def add_to_space(self, space):
        """
        Simulator provides the space, and the object adds itself to the space
        Pymunk does not allow add same shape to multiple spaces
        :param space:
        :return:
        """
        space.add(self._body)
        for shape in self.shapes:
            shape.density = self._density
            space.add(shape)

    def set_ID(self, ID: int | tuple[int,int], moveable: bool = True):
        """
        Used by simulator to set the ID of the object, so that it can be identified in the simulator
        :param ID:
        :param moveable:
        :return:
        """
        if moveable:
            self._body.moveId = ID
        else:
            self._body.fixedId = ID



    @property
    def color(self):
        return self._color  # Return the _color attribute

    @color.setter
    def color(self, color):
        self._color = color
        for s in self.shapes:
            s.color = color

    # @staticmethod
    # def get_force_template():
    #     """
    #     Returns the force template for the object
    #     :return:
    #     """
    #     return np.zeros(4)

    def apply_force(self, force: np.array, global_coords=True):

        direction = force[:2]
        pos = force[2:]



        if global_coords:
            rm = rot_matrix(-self.orientation)
            direction = np.dot(rm, direction)




        self._body.apply_force_at_local_point(direction.tolist(), force[2:].tolist())