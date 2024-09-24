from typing import List, Tuple
from abc import  abstractmethod

from ...base_multibody import BaseMultiBodyObject
from .pm_singlebody import PMSingleBodyObject

class PMMultiBodyObject(BaseMultiBodyObject):
    """abstract class for multi body object, preimplemented for linear
    objects - e.g. Cable"""
    def __init__(self):
        super().__init__()
        self.bodies = [] # type: List[PMSingleBodyObject]
        self._density = 1
        self._friction = 0.5
        self._color = (0, 0, 0, 0)

    def __deepcopy__(self, memodict={}):
        raise NotImplementedError("Deepcopy not implemented")

    def __getitem__(self, item) -> PMSingleBodyObject:
        return self.bodies[item]

    def __len__(self):
        return len(self.bodies)

    def __iter__(self):
        return iter(self.bodies)

    def __setitem__(self, key, value):
        self.bodies[key] = value

    @property
    def collision_data(self):
        raise NotImplementedError("MultiBodyObject does not have collision data")


    def append(self, body:PMSingleBodyObject):
        body.color = self.color
        body.density = self.density
        body.friction = self.friction
        self.bodies.append(body)

    def set_ID(self, base_ID : tuple[int], moveable=bool):
        """
        Default setId that expects linear indexing
        Must be overloaded if the object has different indexing (e.g) square indexing
        :param base_ID:
        :param moveable:
        :return:
        """
        if moveable:
            for i,b in enumerate(self.bodies):
                b.set_ID((base_ID[0],i),moveable=True)
        else:
            for i,b in enumerate(self.bodies):
                b.set_ID((base_ID[0],i),moveable=False)

    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, color):
        self._color = color
        for b in self.bodies:
            b.color = color

    # def get_force_template(self):
    #     return [b.get_force_template() for b in self.bodies]

    def add_to_space(self, space):
        for b in self.bodies:
            b.add_to_space(space)

    @property
    def density(self):
        return self._density

    @density.setter
    def density(self, density):
        self._density = density
        for b in self.bodies:
            b.density = density

    @property
    def friction(self):
        return self._friction

    @friction.setter
    def friction(self, friction):
        self._friction = friction
        for b in self.bodies:
            b.friction = friction

    def link_body(self, body,index: Tuple[int,...]):
        if len(index) == 1:
            self.bodies[index[0]].collision_data = None
            self.bodies[index[0]].body = body
            self.bodies[index[0]].shapes = body.shapes
        else:
            raise ValueError("Linking of multidimensional bodies not implemented")

    def get_body(self, index: Tuple[int,...]):
        return self.bodies[index[0]]

    def save_manual_forces(self):
        for b in self.bodies:
            b.save_manual_forces()


    @property
    def position(self):
        raise NotImplementedError("This is abstract class")

    @property
    def orientation(self):
        raise NotImplementedError("This is abstract class")

    @property
    def velocity(self):
        raise NotImplementedError("This is abstract class")

    @property
    def angular_velocity(self):
        raise NotImplementedError("This is abstract class")



