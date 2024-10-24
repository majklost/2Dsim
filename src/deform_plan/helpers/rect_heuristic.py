import numpy as np
from copy import  deepcopy

from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.samplers.ndim_sampler import NDIMSampler
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.messages.sim_node import SimNode
from deform_plan.storages.GNAT import GNAT
from deform_plan.assets.PM import *
from deform_plan.helpers.seed_manager import manager
from deform_plan.utils.PM_space_visu import show_sim, make_draw_circle, make_draw_line


def make_guider(movable_idx, velocity):
    def guider_fnc(sim: Simulator, start: SimNode, goal, guider_data, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        direction = goal.pos - guided_obj.position
        rot_diff = goal.rot - guided_obj.orientation
        if rot_diff > np.pi:
            rot_diff -= 2 * np.pi
        time_for_dir = np.linalg.norm(direction) / velocity
        velocity_vec = direction / time_for_dir
        angular_velocity = rot_diff / time_for_dir
        guided_obj.velocity = velocity_vec
        guided_obj.angular_velocity = angular_velocity
        return True
    return guider_fnc

def make_fail_condition(movable_idx):
    def fail_condition(sim: Simulator, start: SimNode, goal, guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        return guided_obj.collision_data is not None
    return fail_condition

def make_reached_condition(movable_idx,threshold):
    def reached_condition(sim: Simulator, start: SimNode, goal, guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        return distance_inner(guided_obj.position,goal.pos,guided_obj.orientation,goal.rot) < threshold
    return reached_condition

def make_exporter(movable_idx):
    def exporter(sim: Simulator, start: SimNode, goal, cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        return {"pos": guided_obj.position, "rot": guided_obj.orientation}
    return exporter


class _Point:
    def __init__(self,node,pos,rot):
        self.pos = pos
        self.rot = rot
        self.node = node
        if self.pos is None:
            self.pos = node.exporter_data["pos"]
        if self.rot is None:
            self.rot = node.exporter_data["rot"]

def distance_fnc(p1, p2):
    return distance_inner(p1.pos, p2.pos, p1.rot, p2.rot)

def distance_inner(p1p,p2p,p1r,p2r):
    return np.linalg.norm(p1p - p2p)+np.abs(p1r - p2r)

class _Storage:
    def __init__(self,goal,threshold):
        self.gnat = GNAT(distance_fnc)
        self.goal = goal
        self.threshold = threshold
        self.path = None
        self.want_next_iter = True
    def save_to_storage(self,node:SimNode):
        point = _Point(node,node.exporter_data["pos"],node.exporter_data["rot"])
        dist = distance_fnc(point,self.goal)
        if dist < self.threshold:
            self.want_next_iter = False
            self.path = self.get_path(node)
            return

        self.gnat.insert(point)
    def get_nearest(self,point):
        return self.gnat.nearest_neighbour(point)
    @staticmethod
    def get_path(peak:SimNode):
        path = []
        while True:
            path.append(peak)
            if peak.replayer.parent is None:
                break
            peak = peak.replayer.parent
        return list(reversed(path))
def get_pos_path(path):
    if path is None:
        return []
    return [p.exporter_data["pos"] for p in path]




class RectHeuristic:
    def __init__(self, fixed, path_sampler_data, sim_config,show_sim_bool=False, verbose=False):
        self._path_sampler_data = path_sampler_data
        self.goal = self._prepare_goal()
        self._sim_config = sim_config
        self._show_sim_bool =show_sim_bool
        self._sim = self._prepare_sim(fixed)
        self._sampler = self._prepare_sampler()
        self._storage = self._prepare_storage(self.goal)
        self._planner = self._prepare_planner(self._sim)
        self.verbose =verbose
        self.rng = np.random.default_rng(manager().get_seed(self.__class__.__name__))

        # if self._show_sim_bool:
        #     show_sim(self._sim)


    def _prepare_sim(self,fixed):
        start_raw = self._path_sampler_data["start"]
        movables = Rectangle(start_raw, self._path_sampler_data['W'], self._path_sampler_data['H'], KINEMATIC)
        movables.orientation = 0
        return Simulator(self._sim_config, [movables], deepcopy(fixed), threaded=False, unstable_sim=False)

    def _prepare_sampler(self):
        w = self._sim_config["width"]
        h = self._sim_config["height"]
        lb = np.array([0, 0, 0])
        ub = np.array([w, h, 2 * np.pi])
        return NDIMSampler(lb,ub)
    def _prepare_goal(self):
        goal_raw = self._path_sampler_data["_goal_points"]
        return _Point(None,np.array(goal_raw),0)

    def _prepare_storage(self,goal):
        return _Storage(goal, self._path_sampler_data["THRESHOLD"])

    def _prepare_planner(self,sim):
        planning_fncs = {
            "guider": make_guider(0, self._path_sampler_data["VELOCITY"]),
            "fail_condition": make_fail_condition(0),
            "reached_condition": make_reached_condition(0, self._path_sampler_data["THRESHOLD"]),
            "exporter": make_exporter(0)
        }
        return FetchAblePlanner(sim, planning_fncs, max_step_cnt=1000, only_simuls=False, sampling_period=50,
                                guider_period=20)

    def run(self):
        self._storage.save_to_storage(self._planner.form_start_node())
        for i in range(self._path_sampler_data["ITERATIONS"]):
            if not self._storage.want_next_iter:
                print("goal reached")
                break
            x, y, rot = self._sampler.sample()
            g = _Point(None, np.array([x, y]), rot)
            q_near = self._storage.get_nearest(g)
            response = self._planner.check_path(q_near.node, g)
            for r in response.checkpoints:
                self._storage.save_to_storage(r)
            if self.verbose and i % 200 == 0:
                print("heuristic iter: ", i)
        path = get_pos_path(self.post_process(self._storage.path))[2:]
        if self._show_sim_bool:
            self._show_sim_paths(path)

        return path
    def _show_sim_paths(self,path):
        all_pts = self._storage.gnat.get_all_nodes()
        show_sim(self._sim,clb=draw_tree(all_pts,self.goal.pos))
        show_sim(self._sim,clb=draw_paths(path,self.goal.pos))

    def post_process(self,path):
        if not path:
            return []

        for i in range(self._path_sampler_data["POST_PROC_ITER"]):
            p1 = self.rng.integers(0,len(path))
            p2 = self.rng.integers(0,len(path))
            if abs(p1-p2)<=1:
                continue
            startI = min(p1,p2)
            endI = max(p1,p2)
            start = path[startI]
            end = path[endI]
            self._planner.sampling_period = 3
            response = self._planner.check_path(start,_Point(end,None,None))
            if response.reached_goal:
                rest = endI+1 if endI != len(path)-1 else endI
                path = path[:startI+1]+response.checkpoints+path[rest:]

        return path

def draw_tree(all_pts,goal):
    def clb(surf):
        make_draw_circle(goal, 20, (255, 0, 0))(surf)
        for i, n in enumerate(all_pts):
            # print(n)
            make_draw_circle(n.pos, 5, color=(0, 255, 0))(surf)
            parent = n.node.replayer.parent
            if parent is not None:
                make_draw_line(n.pos, parent.exporter_data["pos"], 2)(surf)
    return clb

def draw_paths(path,goal):
    def draw(surf):
        make_draw_circle(goal, 10, (255, 0, 0))(surf)
        for i,p in enumerate(path):
            if i == 0:
                make_draw_circle(p,10)(surf)
            else:
                make_draw_circle(p,10)(surf)
                make_draw_line(path[i-1],p,5)(surf)
    return draw