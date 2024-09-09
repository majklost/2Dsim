import numpy as np

from deform_plan.simulators.PM.pm_simulator import PMExport

class NodeReached:
    def __init__(self,iter_cnt,sim_export=None, replayer: 'Replayer'=None):
        self.sim_export = sim_export # type: PMExport
        self.replayer = replayer # type: NodeReached.Replayer
        self.iter_cnt :int = iter_cnt
        if sim_export is None and replayer is None:
            raise ValueError("Either sim_export or replayer should be provided")



    class Replayer:
        """
        Class to replay the path
        """
        def __init__(self, real_goal:np.array,parent):
            """
            :param iter_cnt:
            :param real_goal: goal node :np.array
            :param parent: parent node of this node
            """
            self.real_goal :NodeGoal = real_goal
            self.parent :NodeReached = parent

class NodeGoal:
    def __init__(self, info_vec: np.array):
        self.info_vec = info_vec #e.g x,y,rot,iter_cnt,force

