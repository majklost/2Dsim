from src.RRTNode import RRTNodeCable
import pymunk
from typing import List
class CablePlannerSingle:
    """
    Plans a segment of cable from one position to another
    """
    def __init__(self,space: pymunk.Space, max_force, FPS=80,verbose=False):
        """

        :param space: current space, new will be copied from this,
        it is assumed that bodies which can be controlled has attribute "controlled" and
        bodies that are moved by the cable has attribute "moved"
        :param max_force: maximum cumulative force that can be applied to the cable (if multiple controllable segments, force is divided among them)
        :param FPS:
        :param verbose: if debug info should be printed
        """

        self.space = space.copy()
        self.max_force = max_force
        self.moveable_bodies = self._identify_atrrib_body("movedID")
        self.controlled_bodies = self._identify_atrrib_body("controlledID")
        print("Moveable bodies count: ", len(self.moveable_bodies), " Controlled bodies count: ", len(self.controlled_bodies))

    def _identify_atrrib_body(self, atrrib: str):
        """
        Identifies bodies with the given attribute
        :return:
        """
        attrib_bodies = []
        for b in self.space.bodies:
            if hasattr(b, atrrib):
                attrib_bodies.append(b)
        return sorted(attrib_bodies, key=lambda x: getattr(x, atrrib))

    def check_path(self, goal: List[RRTNodeCable]) -> List[RRTNodeCable]:
        """

        :param goal: positions for controlled segments - number of them must be same as number of controlled bodies
        :return: new checkpoints to tree
        """
        pass
