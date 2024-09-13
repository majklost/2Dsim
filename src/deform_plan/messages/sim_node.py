from deform_plan.simulators.PM.pm_simulator import PMExport

class SimNode:
    def __init__(self, exporter_data:dict,guider_data:dict,all_iter_cnt:int, replayer: 'Replayer',sim_export:PMExport=None):
        self.all_iter_cnt = all_iter_cnt
        self.sim_export = sim_export
        self.replayer = replayer
        self.exporter_data = exporter_data
        self.guider_data = guider_data

    def __str__(self):
        # return f"SimNode: {self.all_iter_cnt}, {self.exporter_data},\n PARENT: {self.replayer.parent}"
        return f"SimNode: {self.all_iter_cnt}, {self.exporter_data}"
class Replayer:
    """
    Class to replay the path
    """
    def __init__(self, segment_iter_cnt:int,real_goal,parent):
        """
        :param segment_iter_cnt:
        :param real_goal: goal node :np.array
        :param parent: parent node of this node
        """
        self.segment_iter_cnt = segment_iter_cnt
        self.real_goal = real_goal
        self.parent :SimNode = parent

# class NodeReached:
#     def __init__(self,iter_cnt,sim_export=None, replayer: 'Replayer'=None):
#         self.sim_export = sim_export # type: PMExport
#         self.replayer = replayer # type: NodeReached.Replayer
#         self.iter_cnt :int = iter_cnt
#         if sim_export is None and replayer is None:
#             raise ValueError("Either sim_export or replayer should be provided")
#
#
#
#     class Replayer:
#         """
#         Class to replay the path
#         """
#         def __init__(self, real_goal:np.array,parent):
#             """
#             :param iter_cnt:
#             :param real_goal: goal node :np.array
#             :param parent: parent node of this node
#             """
#             self.real_goal :NodeGoal = real_goal
#             self.parent :NodeReached = parent
#
#
#
#
# @dataclass
# class NodeGoal:
#         pos: np.array
#         rot : float
#         force : float
#         iter_cnt : int
#         # self.info_vec = info_vec #e.g x,y,rot,force,iter_cnt

