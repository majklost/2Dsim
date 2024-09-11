from deform_plan.storages.base_storage import BaseStorage

class SimpleMW:
    def __init__(self, storage:BaseStorage, cmp_fnc):
        self.storage = storage
        self.cmp_fnc = cmp_fnc


    def insert(self, point):
        self.storage.insert(point)

    def nearest_neighbour(self, point):
        return self.storage.nearest_neighbour(point, self.cmp_fnc)


