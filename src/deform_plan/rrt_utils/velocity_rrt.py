import numpy as np
from ..simulators.PM.pm_simulator import Simulator
from ..messages.sim_node import SimNode
from ..storages.kd_tree import KDTree
def make_guider(movable_idx,max_vel):
    def guider_fnc(sim:Simulator,start:SimNode,goal,guider_data:dict,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if cur_iter_cnt==0 and not np.any(np.isclose(guided_obj.position,start.exporter_data["pos"])):
            if start.exporter_data["cur_iter"] != start.all_iter_cnt:
                raise ValueError("iter cnt differs: ", start.exporter_data["cur_iter"], start.all_iter_cnt)

            raise ValueError("start pos differs: ", guided_obj.position, start.exporter_data["pos"])

        pos = goal.pos
        rot = goal.rot
        iter_cnt = goal.iter_cnt
        rot_diff = rot - sim.movable_objects[movable_idx].orientation
        direction = pos - sim.movable_objects[movable_idx].position
        iter_diff = iter_cnt - (start.all_iter_cnt+cur_iter_cnt)
        time_diff = iter_diff * 1/sim.fps
        if iter_diff <=0:
            guided_obj.velocity = np.array([0,0])
            guided_obj.angular_velocity = 0
            return False
        vel = direction / time_diff

        vel_scalar  = np.linalg.norm(vel)
        vel_length_one = vel / vel_scalar
        coef = min(vel_scalar,max_vel)
        vel = coef * vel_length_one


        guider_data["prev_vel"] = vel
        angu_vel = rot_diff / time_diff

        guided_obj.velocity = vel
        guided_obj.angular_velocity = angu_vel
        return True
    return guider_fnc

def make_end_condition(movable_idx):
    def end_condition(sim:Simulator,start:SimNode,goal,cur_iter_cnt):
        guided_obj = sim.movable_objects[movable_idx]
        if guided_obj.collision_data is not None:
            return True
        return False
    return end_condition


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
        return ((g.pos[0]- p[0])**2 + (g.pos[1]-p[1])**2 +0*(g.rot-p[2])**2)**0.5

class StorageWrapper:
    def __init__(self, goal, threshold=2):
        self.goal = goal
        self.tree =KDTree(4)
        self.want_next_iter = True
        self.threshold = threshold
        self._end_node :SimNode|None = None


    def save_to_storage(self, node:SimNode):
        x,y = node.exporter_data["pos"]
        r = node.exporter_data["angle"]
        iter = node.exporter_data["cur_iter"]
        point = Point(node,x,y,r,iter)
        dist = distance_fnc(self.goal,point)

        if dist < 10:
            self.want_next_iter = False
            self._end_node = node
        self.tree.insert(point)

    def get_nearest(self,g:Goal):
        # x,y = g.pos
        # r =g.rot
        # iter = g.iter_cnt
        # point = Point(None,x,y,r,iter)
        # print(self.tree.root)
        res =self.tree.nearest_neighbour(g,distance_fnc).node
        return res

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
