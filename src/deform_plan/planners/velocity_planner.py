import numpy as np

from .fetchable_planner import FetchAblePlanner
from deform_plan.messages.nodes.sim_node import NodeGoal



class VelocityPlanner(FetchAblePlanner):
    def _guide_to_goal(self, goal: NodeGoal):
        pos  = goal.pos
        rot = goal.rot
        iter_cnt = goal.iter_cnt
        rot_diff = rot - self.simulator.movable_objects[self.movable_idx].orientation

        direction = pos - self.simulator.movable_objects[self.movable_idx].position
        iter_diff = iter_cnt - self.iter_cnt
        time_diff = iter_diff * 1/self.simulator.fps
        if iter_diff <=0 :
            self.simulator.movable_objects[self.movable_idx].velocity = np.array([0,0])
            self.simulator.movable_objects[self.movable_idx].angular_velocity = 0
            return False
        vel = direction / time_diff
        angu_vel = rot_diff / time_diff
        self.simulator.movable_objects[self.movable_idx].velocity = vel
        self.simulator.movable_objects[self.movable_idx].angular_velocity = angu_vel
        return True


