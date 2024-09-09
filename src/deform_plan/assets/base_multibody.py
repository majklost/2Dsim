from abc import ABC, abstractmethod

class BaseMultiBodyObject(ABC):
    def __init__(self):
        pass

    @property
    @abstractmethod
    def position(self):
        """
        Position of the object
        Be careful each multibody object can interpret position differently
        :return:
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def orientation(self):
        """
        Orientation of the object  - angle or quaternion
        Be careful each multibody object can interpret orientation differently

        :return:
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def velocity(self):
        """
        Velocity of the object
        Be careful each multibody object can interpret velocity differently

        :return:
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def angular_velocity(self):
        """
        Angular velocity of the object radians per second and direction
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def apply_force(self, force):
        """
        Apply force to the object
        :param force: force to apply
        :return:
        """
        pass