


import numpy as np
from queue import PriorityQueue

from pygame.gfxdraw import pixel

from deform_plan.storages.base_storage import BaseStorage

class GNATNode:


    def __init__(self, distance_fnc,point=None,arity=5):
        self.distance_fnc = distance_fnc
        self.arity = arity
        self.point = point
        self.num_children = 0         # Number of children
        self.R = None                 # R matrix
        self.children = []            # Children of this node
    def insert(self, child):
        if self.R is None:
            self.R = np.zeros((self.arity, self.arity,2))
            self.R[:,:,1] = 0
            self.R[:,:,0] = np.inf
        min_dist = np.inf
        min_idx = -1
        distances = np.zeros(self.num_children)
        for i in range(self.num_children):
            distances[i] = self.distance_fnc(child.point, self.children[i].point)
            if distances[i] < min_dist:
                min_dist = distances[i]
                min_idx = i

        if self.num_children < self.arity:
            cur_idx = self.num_children
            self.children.append(child)
            self.num_children += 1
            #we know we are in a leaf, so it is easy to update the R matrix - low and upper are same - it is distance
            #between points
            self.R[cur_idx,cur_idx,:] = 0 #distance to itself is 0
            self.R[:cur_idx,cur_idx,0] = distances
            self.R[:cur_idx,cur_idx,1] = distances
            self.R[cur_idx,:cur_idx,0] = distances
            self.R[cur_idx,:cur_idx,1] = distances


        else:
            #we are in full node
            #calc distance to centers and find the closest one
            distances = np.array(distances)
            cur_idx = min_idx
            self.children[cur_idx].insert(child)
            self.R[:,cur_idx,0] = np.minimum(self.R[:,cur_idx,0],distances)
            self.R[:,cur_idx,1] = np.maximum(self.R[:,cur_idx,1],distances)

        #update the R matrix
        #I know into which child I called insert recursively, so I can update the R matrix
        #e.g for all other children, check if the distance to the new child changes lower or upper bound
    def validate(self):
        #function to test that the R matrix is correct
        #called after all insertions, it computes the distance between all children and checks if the R matrix is correct
        if self.R is None:
            return True

        for i in range(self.num_children):
            cur_child_point = self.children[i].point
            for j in range(self.num_children):
                min_dist, max_dist = self.children[j].validate_min_max(cur_child_point)
                if abs(min_dist- self.R[i,j,0]) > 1e-10 or abs(max_dist - self.R[i,j,1]) > 1e-10:
                    print(self.R[:,:,0])
                    print(self.R[:,:,1])
                    raise ValueError("R matrix is incorrect: ", min_dist, max_dist, self.R[i,j,0], self.R[i,j,1])
        for i in range(self.num_children):
            self.children[i].validate()


    def validate_min_max(self, test_point):
        #called on another node, it calculates distances to test_point and returns min and max distance
        min_dist = np.inf
        max_dist = -np.inf
        point_dist = self.distance_fnc(test_point, self.point)
        if point_dist < min_dist:
            min_dist = point_dist
        if point_dist > max_dist:
            max_dist = point_dist
        for i in range(self.num_children):
            cur_child = self.children[i]
            cur_min, cur_max = cur_child.validate_min_max(test_point)
            if cur_min < min_dist:
                min_dist = cur_min
            if cur_max > max_dist:
                max_dist = cur_max
        return min_dist, max_dist
    def nearest_neighbour(self, point, r):

        pruned = [False]*self.num_children

        cur_best = point

        for i in range(self.num_children):
            if pruned[i]:
                continue
            cur_child = self.children[i]
            cur_dist = self.distance_fnc(point, cur_child.point)
            if cur_dist <= r:
                cur_best = cur_child
                r = cur_dist

            rec_node, rec_best_dist = cur_child.nearest_neighbour(point, r)
            if rec_best_dist < r:
                r = rec_best_dist
                cur_best = rec_node

            for j in range(self.num_children):
                if not pruned[j]:
                    if cur_dist-r > self.R[i,j,1] or cur_dist+r < self.R[i,j,0]:
                        pruned[j] = True
        return cur_best, r
    def get_all_points(self):
        pts = [self.point]
        for child in self.children:
            pts += child.get_all_points()
        return pts




    def __str__(self):
        s = f"Point: {self.point}, childnum: {self.num_children}\n"
        if self.R is not None:
            pass
        s+= "-----------START CHILDREN\n"
        for i in range(self.num_children):
            s += f"-Child {i}: {self.children[i]}\n"
        s+= "-----------END CHILDREN\n"
        return s
    def __repr__(self):
        return self.__str__()

# class Bucket:
#     def __init__(self, bucket_size):
#         self.bucket_size = bucket_size
#         self.points = []
#         self.num_points = 0
#     def insert(self, point):
#         if self.num_points < self.bucket_size:
#             self.points.append(point)
#             self.num_points += 1
#         else:
#             raise ValueError("Bucket is full")



class GNAT(BaseStorage):
    def __init__(self, distancefnc, arity=5):
        super().__init__(distancefnc)
        self.arity = arity
        self.root = GNATNode(distancefnc, arity=arity)

    def insert(self, point):
        node = GNATNode(self.distancefnc, point, self.arity)
        self.root.insert(node)


    def nearest_neighbour(self, point):
        if self.root is None:
            return None
        return self.root.nearest_neighbour(point, np.inf)[0].point

    def get_all_points(self):
        if self.root is None:
            raise ValueError("Root is None")
        pts = []
        for ch in self.root.children:
            pts += ch.get_all_points()
        return pts

if __name__ == "__main__":

    def distance_fnc(a,b):
        return np.linalg.norm(a-b)

    gnat = GNAT(distance_fnc,arity=2)
    gnat.insert(np.array([1,1]))
    gnat.insert(np.array([1,2]))
    gnat.insert(np.array([2,1]))
    gnat.insert(np.array([2,2]))
    gnat.insert(np.array([3,3]))
    gnat.insert(np.array([3,4]))
    import random
    for i in range(5000):
        x = random.randint(0, 10000)
        y = random.randint(0, 10000)
        gnat.insert(np.array([x,y]))
    gnat.root.validate()
    # print(gnat.root)