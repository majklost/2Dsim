from typing import List
from abc import  abstractmethod

from ...base_multibody import BaseMultiBodyObject
from .pm_singlebody import PMSingleBodyObject

class PMMultiBodyObject(BaseMultiBodyObject):
    def __init__(self):
        super().__init__()
        self.bodies = [] # type: List[PMSingleBodyObject]
        self._density = 1
        self._friction = 0.5
        self._color = (0, 0, 0, 0)

    def __getitem__(self, item) -> PMSingleBodyObject:
        return self.bodies[item]

    def __len__(self):
        return len(self.bodies)

    def __iter__(self):
        return iter(self.bodies)

    def __setitem__(self, key, value):
        self.bodies[key] = value


    def append(self, body:PMSingleBodyObject):
        body.color = self.color
        body.density = self.density
        body.friction = self.friction
        self.bodies.append(body)

    def set_ID(self, ID : int, moveable=bool):
        if moveable:
            for i,b in enumerate(self.bodies):
                b.set_ID((ID,i),moveable=True)
        else:
            for i,b in enumerate(self.bodies):
                b.set_ID((ID,i),moveable=False)

    def apply_force(self, force, index=None):
        # self.bodies[index[1]].apply_force(force)
        if index is None:
            for i in range(len(self.bodies)):
                self.bodies[i].apply_force(force[i])
        else:
            for i in index:
                self.bodies[i].apply_force(force[i])
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


    @property
    @abstractmethod
    def position(self):
        raise NotImplementedError("This is abstract class")

    @property
    @abstractmethod
    def orientation(self):
        raise NotImplementedError("This is abstract class")

    @property
    @abstractmethod
    def velocity(self):
        raise NotImplementedError("This is abstract class")

    @property
    @abstractmethod
    def angular_velocity(self):
        raise NotImplementedError("This is abstract class")



