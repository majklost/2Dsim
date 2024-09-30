import sys
# from numpy.linalg import norm
norm = lambda x:x
from  .base_storage import BaseStorage

sys.setrecursionlimit(10000)
# code taken from https://www.geeksforgeeks.org/search-and-insertion-in-k-dimensional-tree/
# nearest neighbour search implemented myself
# A structure to represent node of kd tree
class Node:
    def __init__(self, point):
        self.point = point
        self.left = None
        self.right = None

    def __iter__(self):
        if self.left:
            yield from self.left
        yield self.point
        if self.right:
            yield from self.right

class KDTree(BaseStorage):
    def __init__(self, distancefnc, k=3):
        super().__init__(distancefnc)
        self.root = None
        self.tree = _KDTree
        self.tree.k = k

    def insert(self,point):
        self.root = self.tree.insert(self.root,point)

    def nearest_neighbour(self,point):
        return self.tree.nearestNeighbour(self.root,point,distancefnc=self.distancefnc)


    def get_all_points(self):
        return self.tree.get_all_points(self.root)

class _KDTree:

    k = 3 # Number of dimensions

    # Inserts a new node and returns root of modified tree
    # The parameter depth is used to decide axis of comparison
    @classmethod
    def _insertRec(cls, root, point, depth):
        # Tree is empty?
        if not root:
            return Node(point)

        # Calculate current dimension (cd) of comparison
        cd = depth % cls.k

        # Compare the new point with root on current dimension 'cd'
        # and decide the left or right subtree
        if norm(point[cd]) < norm(root.point[cd]):
            root.left = cls._insertRec(root.left, point, depth + 1)
        else:
            root.right = cls._insertRec(root.right, point, depth + 1)

        return root

    # Function to insert a new point with given point in
    # KD Tree and return new root. It mainly uses above recursive
    # function "insertRec()"
    @classmethod
    def insert(cls, root, point):
        return cls._insertRec(root, point, 0)

    # A utility method to determine if two Points are same
    # in K Dimensional space
    @classmethod
    def arePointsSame(cls, point1, point2):
        # Compare individual coordinate values
        for i in range(cls.k):
            if point1[i] != point2[i]:
                return False

        return True

    # Searches a Point represented by "point[]" in the K D tree.
    # The parameter depth is used to determine current axis.
    @classmethod
    def _searchRec(cls, root, point, depth):
        # Base cases
        if not root:
            return False
        if cls.arePointsSame(root.point, point):
            return True

        # Current dimension is computed using current depth and total
        # dimensions (k)
        cd = depth % cls.k

        # Compare point with root with respect to cd (Current dimension)
        if norm(point[cd]) < norm(root.point[cd]):
            return cls._searchRec(root.left, point, depth + 1)

        return cls._searchRec(root.right, point, depth + 1)

    @classmethod
    def _nearestNeighbourRec(cls, root, point, distancefnc, depth=0, best_dist=float('inf'), best_node=None):
        if root is None:
            return best_node, best_dist
        cd = depth % cls.k
        # cur dist to current node's point
        current_dist = distancefnc(point, root.point)

        if current_dist < best_dist:
            best_dist = current_dist
            best_node = root.point

        # choose the branch that is closer to the point
        # if cd ==3:
        #     if point[cd] < root.point[cd]:
        #         return best_node, best_dist
        if norm(point[cd]) < norm(root.point[cd]):
            next_branch = root.left
            opposite_branch = root.right
        else:
            next_branch = root.right
            opposite_branch = root.left

        # recurse down the chosen branch
        best_node, best_dist = cls._nearestNeighbourRec(next_branch, point, distancefnc, depth + 1, best_dist=best_dist,
                                                        best_node=best_node)

        # check if the other branch may have a closer point
        if norm(point[cd] - root.point[cd]) < best_dist:
            best_node, best_dist = cls._nearestNeighbourRec(opposite_branch, point, distancefnc, depth + 1,
                                                            best_dist=best_dist,
                                                            best_node=best_node)

        return best_node, best_dist

    @classmethod
    def nearestNeighbour(cls, root, point, distancefnc):
        return cls._nearestNeighbourRec(root, point, distancefnc)[0]

    # Searches a Point in the K D tree. It mainly uses
    # searchRec()
    @classmethod
    def search(cls, root, point):
        # Pass current depth as 0
        return cls._searchRec(root, point, 0)

    @classmethod
    def get_all_points(cls, root):
        ll = []
        rl = []
        if root.left:
            ll = cls.get_all_points(root.left)
        if root.right:
            rl = cls.get_all_points(root.right)
        return ll + [root.point] + rl