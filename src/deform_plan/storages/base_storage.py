from abc import ABC, abstractmethod

class BaseStorage(ABC):
    @abstractmethod
    def insert(self, point):
        raise NotImplementedError

    @abstractmethod
    def nearest_neighbour(self, point, distancefnc):
        raise NotImplementedError