# import numpy as np
#
# from deform_plan.assets.PM import *
# from deform_plan.assets.PM.objects.boundings import Boundings
# from deform_plan.messages.sim_node import SimNode
# from deform_plan.planners.fetchable_planner import FetchAblePlanner
# from deform_plan.samplers.ndim_sampler import NDIMSampler
# from deform_plan.simulators.PM.pm_simulator import Simulator
# from deform_plan.storages.GNAT import GNAT
# from deform_plan.utils.PM_space_visu import show_sim, make_draw_circle, make_draw_line
#
#
# def make_guider(movable_idx, velocity):
#     def guider_fnc(sim: Simulator, start: SimNode, _goal_points, guider_data, cur_iter_cnt):
#         guided_obj = sim.movable_objects[movable_idx]
#         direction = _goal_points.pos - guided_obj.position
#         rot_diff = _goal_points.rot - guided_obj.orientation
#         if rot_diff > np.pi:
#             rot_diff -= 2 * np.pi
#         time_for_dir = np.linalg.norm(direction) / velocity
#         velocity_vec = direction / time_for_dir
#         angular_velocity = rot_diff / time_for_dir
#         guided_obj.velocity = velocity_vec
#         guided_obj.angular_velocity = angular_velocity
#         return True
#     return guider_fnc
#
# def make_fail_condition(movable_idx):
#     def fail_condition(sim: Simulator, start: SimNode, _goal_points, guider_data,cur_iter_cnt):
#         guided_obj = sim.movable_objects[movable_idx]
#         return guided_obj.collision_data is not None
#     return fail_condition
#
# def make_reached_condition(movable_idx,threshold):
#     def reached_condition(sim: Simulator, start: SimNode, _goal_points, guider_data,cur_iter_cnt):
#         guided_obj = sim.movable_objects[movable_idx]
#         return distance_inner(guided_obj.position,_goal_points.pos,guided_obj.orientation,_goal_points.rot) < threshold
#     return reached_condition
#
# def make_exporter(movable_idx):
#     def exporter(sim: Simulator, start: SimNode, _goal_points, cur_iter_cnt):
#         guided_obj = sim.movable_objects[movable_idx]
#         return {"pos": guided_obj.position, "rot": guided_obj.orientation}
#     return exporter
#
#
# class _Point:
#     def __init__(self,node,pos,rot):
#         self.pos = pos
#         self.rot = rot
#         self.node = node
#         if self.pos is None:
#             self.pos = node.exporter_data["pos"]
#         if self.rot is None:
#             self.rot = node.exporter_data["rot"]
#
# def distance_fnc(p1, p2):
#     return distance_inner(p1.pos, p2.pos, p1.rot, p2.rot)
#
# def distance_inner(p1p,p2p,p1r,p2r):
#     return np.linalg.norm(p1p - p2p)+np.abs(p1r - p2r)
#
# class Storage:
#     def __init__(self,_goal_points,threshold):
#         self.gnat = GNAT(distance_fnc)
#         self._goal_points = _goal_points
#         self.threshold = threshold
#         self.path = None
#         self.want_next_iter = True
#     def save_to_storage(self,node:SimNode):
#         point = _Point(node,node.exporter_data["pos"],node.exporter_data["rot"])
#         dist = distance_fnc(point,self._goal_points)
#         if dist < self.threshold:
#             self.want_next_iter = False
#             self.path = self.get_path(node)
#             return
#
#         self.gnat.insert(point)
#     def get_nearest(self,point):
#         return self.gnat.nearest_neighbour(point)
#     @staticmethod
#     def get_path(peak:SimNode):
#         path = []
#         while True:
#             path.append(peak)
#             if peak.replayer.parent is None:
#                 break
#             peak = peak.replayer.parent
#         return list(reversed(path))
# def get_pos_path(path):
#     if path is None:
#         return []
#     return [p.exporter_data["pos"] for p in path]
#
#
# #Sample paths with simpler object, then use them to guide the more complex object
# class PathSamplerRect:
#     def __init__(self,fixed,config,path_sampler_data,show_sim_bool=False,seed=None):
#         """
#         :param fixed: list of fixed objects
#         :param config: dict with configuration
#         :param path_sampler_data: dict with additional data for path sampling
#         :return: paths - lists of points
#         """
#         self.subsampler_config = config["SUBSAMPLER"]
#         start_raw = path_sampler_data["start"]
#         self.show_sim_bool = show_sim_bool
#         movables = Rectangle(start_raw, self.subsampler_config['W'], self.subsampler_config['H'], KINEMATIC)
#         movables.orientation = 0
#         self.sim = Simulator(config["CFG"], [movables], fixed, threaded=config['THREADED_SIM'],unstable_sim=False)
#         if self.show_sim_bool:
#             show_sim(self.sim)
#         w = config["CFG"].width
#         h = config["CFG"].height
#         lb = np.array([0, 0, 0])
#         ub = np.array([w, h, 2 * np.pi])
#         self._sampler = NDIMSampler(lb, ub, seed)
#         goal_raw = path_sampler_data["_goal_points"]
#         self._goal_points = _Point(None, np.array(goal_raw), 0)
#         self.storage = Storage(self._goal_points, self.subsampler_config["THRESHOLD"])
#         planning_fncs = {
#             "guider": make_guider(0, self.subsampler_config["VELOCITY"]),
#             "fail_condition": make_fail_condition(0),
#             "reached_condition": make_reached_condition(0, self.subsampler_config["THRESHOLD"]),
#             "exporter": make_exporter(0)
#         }
#         self.planner = FetchAblePlanner(
#             self.sim,
#             planning_fncs,
#             max_iter_cnt=1000,
#             only_simuls=False,
#             sampling_period=100,
#             guider_period=20,
#             track_analytics=False
#         )
#         self.storage.save_to_storage(self.planner.form_start_node())
#     def run(self):
#         for i in range(self.subsampler_config["ITERATIONS"]):
#             if not self.storage.want_next_iter:
#                 break
#             x, y, rot = self._sampler.sample()
#             g = _Point(None, np.array([x, y]), rot)
#             q_near = self.storage.get_nearest(g)
#             response = self.planner.check_path(q_near.node, g)
#             for r in response.checkpoints:
#                 self.storage.save_to_storage(r)
#             if i % 200 == 0:
#                 print("heurisitc iter: ", i)
#         path = get_pos_path(self.post_process(self.storage.path))
#         if self.show_sim_bool:
#             all_pts = self.storage.gnat.get_all_nodes()
#             show_sim(self.sim, clb=draw_tree(all_pts,self._goal_points.pos))
#             # show_sim(self._sim, clb=draw_paths(get_pos_path(self._storage.path)))
#             show_sim(self.sim,clb=draw_paths(path,self._goal_points.pos))
#
#         return path
#     def post_process(self,path,seed=None):
#         if not path:
#             return []
#         rng = np.random.RandomState(seed)
#         for i in range(self.subsampler_config["POST_PROC_ITER"]):
#             p1 =rng.randint(0,len(path))
#             p2 = rng.randint(0,len(path))
#             if abs(p1-p2)<=1:
#                 continue
#             startI = min(p1,p2)
#             endI = max(p1,p2)
#             start = path[startI]
#             end = path[endI]
#             self.planner.sampling_period = 5
#             response = self.planner.check_path(start,_Point(end,None,None))
#             if response.reached_goal:
#                 path = path[:startI+1]+response.checkpoints+path[endI+1:]
#         return path
#
#
# def draw_tree(all_pts,_goal_points):
#     def clb(surf):
#         make_draw_circle(_goal_points, 20, (255, 0, 0))(surf)
#         for i, n in enumerate(all_pts):
#             # print(n)
#             make_draw_circle(n.pos, 5, color=(0, 255, 0))(surf)
#             parent = n.node.replayer.parent
#             if parent is not None:
#                 make_draw_line(n.pos, parent.exporter_data["pos"], 2)(surf)
#     return clb
#
# def draw_paths(path,_goal_points):
#     def draw(surf):
#         make_draw_circle(_goal_points, 10, (255, 0, 0))(surf)
#         for i,p in enumerate(path):
#             if i == 0:
#                 make_draw_circle(p,10)(surf)
#             else:
#                 make_draw_circle(p,10)(surf)
#                 make_draw_line(path[i-1],p,5)(surf)
#     return draw
#
# if __name__ == "__main__":
#     CONFIG = {
#         "CFG": PMConfig(),
#         "THREADED_SIM": False,
#         "UNSTABLE_SIM": False,
#         "ONLY_SIMULS": False,
#         "SUBSAMPLER": {
#             "SEED": None,
#             "ITERATIONS": 2000,
#             "THRESHOLD": 30,
#             "VELOCITY": 1000,
#             "POST_PROC_ITER": 200,
#             'W': 100,
#             'H': 20,
#         }
#     }
#     subsampler_config = CONFIG["SUBSAMPLER"]
#     fixed = [Boundings(CONFIG["CFG"].width, CONFIG["CFG"].height), Rectangle([400, 400], 200, 200, STATIC), RandomObstacleGroup(np.array([50, 300]), 200, 200, 4, 2, radius=100, seed=20)]
#     start = (100,100)
#     _goal_points = (700,700)
#     path_sampler = PathSamplerRect(fixed,CONFIG,{"start":start,"_goal_points":_goal_points},show_sim_bool=True)
#     path = path_sampler.run()
