"""path is replayable if guider was not changed from the time of creation"""
import dill
import os
import datetime
from ..messages.sim_node import SimNode
from ..simulators.PM.pm_simulator import Simulator

from typing import List,Callable,Any

class ReplayablePath:
    def __init__(self, simulator:Simulator,path:List[SimNode],
                 goal:Any,
                 guider:Callable[[Simulator,SimNode,Any,dict,int],bool],
                 reached_condition:Callable[[Simulator,SimNode,Any,int],bool],
                 guider_period:int,
                 additional_data:dict|None=None ):
        """
        Create a replayable path
        :param simulator: simulator that simulated the path
        :param path: list of nodes
        :param goal: goal_points node
        :param guider:
        :param additional_data:
        """
        self.simulator = simulator
        self.path = path
        self.goal = goal
        self.guider = guider
        self.guider_period = guider_period
        self.reached_condition = reached_condition
        self.additional_data = additional_data
        self.creation_date = datetime.date.today()
        # self.creation_date = datetime.date.today()
        if additional_data is None:
            self.additional_data = {}

    def save(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'wb') as f:
            dill.dump(self, f)
        print(f"Path saved to {filepath}")
