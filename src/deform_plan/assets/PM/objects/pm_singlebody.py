import pymunk
import numpy as np

from ...base_singlebody import BaseSingleBodyObject


class PMSingleBodyObject(BaseSingleBodyObject):
    def __init__(self,body_type):
        super().__init__()
        self._collision_object = None #what collided with my object
        self.colliding = False
        self.shapes = []
        self._body = pymunk.Body(body_type=body_type)
        self._color = (0, 0, 0, 0)
        self._density = 1


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
            space.add(shape)

    def set_ID(self, ID: int, moveable: bool = True):
        """
        Used by simulator to set the ID of the object, so that it can be identified in the simulator
        :param ID:
        :param moveable:
        :return:
        """
        if moveable:
            self._body.moveId = int(ID)
        else:
            self._body.fixedId = int(ID)



    @property
    def color(self):
        return self._color  # Return the _color attribute

    @color.setter
    def color(self, color):
        self._color = color
        for s in self.shapes:
            s.color = color
