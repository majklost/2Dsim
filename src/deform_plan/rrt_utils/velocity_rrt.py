import numpy as np

from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode
from ..storages.kd_tree import KDTree
# from ..storages.brute_force import BruteForce as KDTree
def make_guider(movable_idx,max_vel,min_vel):
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data:dict,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if cur_iter_cnt==0:
            if not np.any(np.isclose(guided_obj.position,start.exporter_data["pos"])):
                if start.exporter_data["cur_iter"] != start.all_iter_cnt:
                    raise ValueError("iter cnt differs: ", start.exporter_data["cur_iter"], start.all_iter_cnt)

                raise ValueError("start pos differs: ", guided_obj.position, start.exporter_data["pos"])
            if goal.iter_cnt - start.all_iter_cnt <=0:
                raise ValueError("Impossible iter diff given: ", goal.iter_cnt, start.all_iter_cnt)



        pos = goal.pos
        rot = goal.rot
        iter_cnt = goal.iter_cnt
        rot_diff = rot - sim.movable_objects[movable_idx].orientation
        direction = pos - sim.movable_objects[movable_idx].position
        iter_diff = iter_cnt - (start.all_iter_cnt+cur_iter_cnt)
        time_diff = iter_diff * 1/sim.fps
        vel = direction / time_diff

        vel_scalar = np.linalg.norm(vel)
        vel_length_one = vel / vel_scalar
        coef = max(min(vel_scalar,max_vel),min_vel)
        vel = coef * vel_length_one
        iter_diff = vel_scalar/coef * iter_diff
        time_diff = iter_diff * 1 / sim.fps

        guider_data["prev_vel"] = vel
        angu_vel = rot_diff / time_diff
        # print("angu_vel: ", angu_vel)

        guided_obj.velocity = vel
        guided_obj.angular_velocity = angu_vel
        return True
    return guider_fnc

def make_fail_condition(movable_idx):
    def fail_condition(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if guided_obj.collision_data is not None:
            return True
        return False
    return fail_condition
def make_reached_condition(movable_idx):
    def reached_condition(sim:Simulator,start:SimNode,goal,guider_data,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        direction = goal.pos - sim.movable_objects[movable_idx].position
        return np.linalg.norm(direction) < 5
    return reached_condition

def make_exporter(movable_idx):
    def exporter(sim:Simulator,start:SimNode|None,goal,cur_iter_cnt):
        data = dict()
        all_iter_cnt = 0
        if start is not None:
            all_iter_cnt = start.all_iter_cnt
        guided_obj = sim.movable_objects[movable_idx]
        data["pos"] = guided_obj.position
        data["angle"] = guided_obj.orientation
        data["cur_iter"] = all_iter_cnt+cur_iter_cnt
        return data

    return exporter
class Point:
    def __init__(self,node,x,y,rot,t):
        self.indexable =[x,y,rot,t]
        self.node = node
    def __getitem__(self, item):
        return self.indexable[item]


class Goal:
    def __init__(self):
        self.pos: np.array = None
        self.rot: float = 0
        self.iter_cnt: int = 0

    def __getitem__(self, item):
        if item ==0:
            return self.pos[0]
        elif item ==1:
            return self.pos[1]
        elif item ==2:
            return self.rot
        else:
            return  self.iter_cnt



def distance_fnc(g:Goal,p:Point):
    if g.iter_cnt <= p[3]:
        return float("inf")
    else:
        return (sum((g[i] - p[i]) ** 2 for i in range(3))) **0.5
    # return (sum((g[i] - p[i]) ** 2 for i in range(3))) ** 0.5

class StorageWrapper:
    def __init__(self, goal, threshold=2):
        self.goal = goal
        self.tree =KDTree(distance_fnc,3)
        self.want_next_iter = True
        self.threshold = threshold
        self._end_node :SimNode|None = None
        self.best_dist = float("inf")


    def save_to_storage(self, node:SimNode):
        x,y = node.exporter_data["pos"]
        r = node.exporter_data["angle"]
        iter = node.exporter_data["cur_iter"]
        point = Point(node,x,y,r,iter)
        dist = distance_fnc(self.goal,point)

        if dist < self.best_dist:
            if dist<10:
                self.want_next_iter = False
            self._end_node = node
            self.best_dist = dist
        self.tree.insert(point)

    def get_nearest(self,g:Goal):
        # x,y = g.pos
        # r =g.rot
        # iter = g.iter_cnt
        # point = Point(None,x,y,r,iter)
        # print(self.tree.root)
        res =self.tree.nearest_neighbour(g).node
        return res

    def get_all_points(self):
        return [x.node  for x in self.tree.get_all_points()]

    def get_path(self):
        path =[]
        if self._end_node is None:
            return path

        cur = self._end_node
        while True:
            path.append(cur)
            if cur.replayer.parent is None:
                break
            cur = cur.replayer.parent

        return list(reversed(path))
