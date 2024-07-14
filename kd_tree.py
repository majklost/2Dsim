#code taken from https://www.geeksforgeeks.org/search-and-insertion-in-k-dimensional-tree/

# Number of dimensions
k = 3


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



# Inserts a new node and returns root of modified tree
# The parameter depth is used to decide axis of comparison
def _insertRec(root, point, depth):
    # Tree is empty?
    if not root:
        return Node(point)

    # Calculate current dimension (cd) of comparison
    cd = depth % k

    # Compare the new point with root on current dimension 'cd'
    # and decide the left or right subtree
    if point[cd] < root.point[cd]:
        root.left = _insertRec(root.left, point, depth + 1)
    else:
        root.right = _insertRec(root.right, point, depth + 1)

    return root


# Function to insert a new point with given point in
# KD Tree and return new root. It mainly uses above recursive
# function "insertRec()"
def insert(root, point):
    return _insertRec(root, point, 0)


# A utility method to determine if two Points are same
# in K Dimensional space
def arePointsSame(point1, point2):
    # Compare individual coordinate values
    for i in range(k):
        if point1[i] != point2[i]:
            return False

    return True


# Searches a Point represented by "point[]" in the K D tree.
# The parameter depth is used to determine current axis.
def _searchRec(root, point, depth):
    # Base cases
    if not root:
        return False
    if arePointsSame(root.point, point):
        return True

    # Current dimension is computed using current depth and total
    # dimensions (k)
    cd = depth % k

    # Compare point with root with respect to cd (Current dimension)
    if point[cd] < root.point[cd]:
        return _searchRec(root.left, point, depth + 1)

    return _searchRec(root.right, point, depth + 1)


def distancefnc(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2) ** 0.5


def _nearestNeighbourRec(root, point, depth=0, distancefnc=distancefnc, best_dist=float('inf'), best_node=None):
    if root is None:
        return best_node, best_dist
    cd = depth % k
    #cur dist to current node's point
    current_dist = distancefnc(point, root.point)

    if current_dist < best_dist:
        best_dist = current_dist
        best_node = root.point

    #choose the branch that is closer to the point
    if point[cd] < root.point[cd]:
        next_branch = root.left
        opposite_branch = root.right
    else:
        next_branch = root.right
        opposite_branch = root.left

    #recurse down the chosen branch
    best_node, best_dist = _nearestNeighbourRec(next_branch, point, depth + 1, best_dist=best_dist, best_node=best_node)

    #check if the other branch may have a closer point
    if abs(point[cd] - root.point[cd]) < best_dist:
        best_node, best_dist = _nearestNeighbourRec(opposite_branch, point, depth + 1, best_dist=best_dist,
                                                best_node=best_node)

    return best_node, best_dist

def nearestNeighbour(root, point, depth=0, distancefnc=distancefnc, best_dist=float('inf'), best_node=None):
    return _nearestNeighbourRec(root, point, depth, distancefnc, best_dist, best_node)[0]


# Searches a Point in the K D tree. It mainly uses
# searchRec()
def search(root, point):
    # Pass current depth as 0
    return _searchRec(root, point, 0)



