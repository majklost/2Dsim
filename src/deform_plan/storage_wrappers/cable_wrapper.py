#standard _storage wrapper for cable
from abc import abstractmethod
from typing import TypedDict,Callable, TypeVar, Any
from deform_plan.messages.sim_node import SimNode
from deform_plan.storage_wrappers.base_wrapper import BaseWrapper
from deform_plan.storages.base_storage import BaseStorage

T= TypeVar('T')
KW = TypeVar('KW')

class TypeUtils(TypedDict):
    dist: Callable[[T,Any],int]
    cls_maker: Callable[[Any,KW], T]
    storage: BaseStorage
    cls_kwargs: KW



class StorageWrapper(BaseWrapper):
    def __init__(self,utils:TypeUtils,goal,goal_threshold,controlled_idxs, verbose=False):
        """
        Standard _storage wrapper for cable
        :param utils: dict with keys "dist" - distance function, "cls_maker" - class that wraps the points, "storage" - _storage object
        :param goal: _goal_points to reach in format cls_maker
        :param goal_threshold: how close to the _goal_points should the solution be
        :param controlled_idxs: indices of the points that are controlled
        """
        self.dist = utils["dist"]
        self.cls_maker = utils["cls_maker"]
        self.cls_kwargs = utils["cls_kwargs"]
        self.storage = utils["storage"]
        self.goal = goal
        self.goal_threshold = goal_threshold
        self.controlled_idxs = controlled_idxs
        self.best_dist = float("inf")
        self._end_node = None
        self.want_next_iter = True
        self.verbose = verbose
        self._save_queries_num = 0
        self._nn_queries_num = 0


    def save_to_storage(self,node:SimNode):
        self._save_queries_num += 1
        point = self._make_point(node)
        dist = self.dist(point,self.goal)
        if dist < self.goal_threshold:
            self.want_next_iter = False
            if self.verbose:
                print("Goal reached")
        if dist < self.best_dist:
            self.best_dist = dist
            self._end_node = node

        self.storage.insert(point=point)
        return True

    def _make_point(self, node):
        return self.cls_maker(node,**self.cls_kwargs)

    def get_nearest(self,point):
        """
        Get nearest point in the _storage
        :param point: type same as cls_maker
        :return: point from _storage
        """
        self._nn_queries_num += 1
        return self.storage.nearest_neighbour(point)

    def get_path(self):
        path = []
        if self._end_node is None:
            return path
        cur = self._end_node
        while True:
            path.append(cur)
            if cur.replayer.parent is None:
                break
            cur = cur.replayer.parent
        return list(reversed(path))

    def get_all_nodes(self):
        return self.storage.get_all_nodes()

    def analytics(self) -> dict:
        return {
            "save_queries": self._save_queries_num,
            "nn_queries": self._nn_queries_num,
            "best_dist": self.best_dist
        }
