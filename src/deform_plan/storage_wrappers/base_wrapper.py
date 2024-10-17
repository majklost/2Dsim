from abc import ABC, abstractmethod
class BaseWrapper(ABC):
    @abstractmethod
    def save_to_storage(self, node):
        pass
    @abstractmethod
    def get_nearest(self, point):
        pass
    @abstractmethod
    def get_path(self):
        pass
    @abstractmethod
    def get_all_nodes(self):
        pass