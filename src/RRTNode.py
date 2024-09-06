#Data sturcture for communication between RRT and LocalPlanner
#Similar to vec2D but I want to be independent of pymunk
import numpy as np
from src.helpers.helperFunctions import get_bodies_from_space, get_points_from_bodies
CLOSE = 2

#TODO: refactor to better inheritance

#basic node for static planning
class RRTNode:
    def __init__(self, x, y, angle, parent=None, added_cnt=0):
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = parent
        self.added_cnt = added_cnt

    def __str__(self):
        return f"Node: {self.x}, {self.y}, {self.angle}"

    def __sub__(self, other):
        return self.x - other.x, self.y - other.y, self.angle - other.angle

    def __repr__(self):
        return self.__str__()

    def get_pos(self):
        return self.x, self.y

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        elif index == 2:
            return self.angle
        else:
            raise IndexError("Index out of bounds")

    def get_comparable_point(self):
        return self.x, self.y, self.angle

class RRTNodeTimed(RRTNode):
    def __init__(self, x,y,angle,time,parent=None,added_cnt=0):
        super().__init__(x,y,angle,parent,added_cnt)
        self.time = time

    def __str__(self):
        return f"Node: {self.x}, {self.y}, {self.angle}, {self.time}"

    def __repr__(self):
        return self.__str__()

    def __getitem__(self, index):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        elif index == 2:
            return self.angle
        elif index == 3:
            return self.time
        else:
            raise IndexError("Index out of bounds")


class RRTNodeSim(RRTNodeTimed):
    def __init__(self,x,y,angle,time,parent=None,added_cnt=0, simSpace=None):
        super().__init__(x,y,angle,time,parent,added_cnt)
        self.simSpace = simSpace

    @staticmethod
    def from_timed(node:RRTNodeTimed, simSpace):
        return RRTNodeSim(node.x, node.y, node.angle, node.time, node.parent, node.added_cnt, simSpace)


class RRTNodeCable:
    def __init__(self, pairs:np.array=None,simSpace=None,  replayer: 'RRTNodeCable.Replayer' =None):
        """
        Create Node, it has either a simSpace or replayer or both
        TODO when simSpace here only parent from replayer is needed
        :param pairs: x,y pairs for each controllable segment
        :param simSpace: copy of the space
        :param replayer: object with information to reproduce the path

        """
        self.points = pairs #type: np.array
        self.simSpace = simSpace
        self.replayer = replayer
        self._movable_bodies = None

        if self.simSpace is None and self.points is None:
            raise ValueError("No points or simSpace")


    def __eq__(self, other):
        self.fill_points()
        if self.points is None or other.points is None:
            return False
        return np.allclose(self.points, other.points, atol=CLOSE)


    def fill_points(self):
        """
        Fills the points from the space
        But it is costly to do it every time - better use only at start
        :return:
        """
        print("Filling points")
        if self.points is None:
            bodies = get_bodies_from_space(self.simSpace, "controlledID")
            self.points = np.array(get_points_from_bodies(bodies))
            movables = get_bodies_from_space(self.simSpace, "movedID")
            print("Movables", len(movables))
            self._movable_bodies = np.array(get_points_from_bodies(movables))

        print("Filled points")
        print(len(self.points))




    def __getitem__(self, item):
        return self.points[item]

    def __str__(self):
        if self.replayer:
            return f"Node:\n {self.points},\n iter_count: {self.replayer.iter_cnt},\n ---------"
        else:
            return "Node: \n" + str(self.points) + " Start NODE"
    def __repr__(self):
        return self.__str__()

    class Replayer:
        """
        Class to replay the path
        """
        def __init__(self, iter_cnt, real_goal:np.array,parent):
            """

            :param iter_cnt:
            :param real_goal: goal node :np.array
            :param parent: parent node of this node
            """
            self.iter_cnt :int = iter_cnt
            self.real_goal :np.array = real_goal
            self.parent :RRTNodeCable = parent





