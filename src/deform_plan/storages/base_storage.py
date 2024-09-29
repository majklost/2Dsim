from abc import ABC, abstractmethod

class BaseStorage(ABC):
    def __init__(self,distancefnc):
        self.distancefnc = distancefnc

    @abstractmethod
    def insert(self, point):
        raise NotImplementedError

    @abstractmethod
    def nearest_neighbour(self, point):
        raise NotImplementedError