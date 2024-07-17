#Three ways to implement it
# 1. give all velocities and test path with recalculation of positions
# 2. make each move a new copy of space, let the physics engine do the work
# 3 for each checking, let physics engine start from beginning, let agent follow the so far path
# requires more memory but is more accurate

import pymunk
from pymunk import Vec2d
from typing import List, Tuple
from RRTNode import RRTNodeSim, RRTNodeCalc

MAX_VEL = 160

#this calculates the path with movement
#now only for one obstacle and one agent
#will probably scale badly, so it is just PoC
class LocalPlannerCalc:
    def __init__(self, space: pymunk.Space, agent: pymunk.Shape,moving_obstacle: pymunk.Body, obstacle_velocity: Tuple[float,float,float],dt=1/80):
        self.space = space
        self.agent = agent
        self.moving_obstacle = moving_obstacle
        self.obstacle_velocity = obstacle_velocity
        self.clb = None
        self.verbose = False

        #decides how many steps to take
        self.dt = dt
    @staticmethod
    def extend_checkpoints(start: RRTNodeCalc, checkpoints: List[RRTNodeCalc], pos: Vec2d, angle,time:float):
        if len(checkpoints) == 0:
            checkpoints.append(RRTNodeCalc(pos.x, pos.y, angle, time, start))
        else:
            checkpoints.append(RRTNodeCalc(pos.x, pos.y, angle, time, checkpoints[-1]))


    def check_path(self,start:RRTNodeCalc, goal: RRTNodeCalc) -> List[RRTNodeCalc]:
        #old positions
        a_old_pos = self.agent.body.position
        a_old_angle = self.agent.body.angle
        ob_old_pos = self.moving_obstacle.position
        ob_old_angle = self.moving_obstacle.angle

        #now calculate the path
        ob_vx, ob_vy, ob_vangle = self.obstacle_velocity
        checkpoints = []
        direction = goal.x - start.x, goal.y - start.y
        angle_morph = goal.angle - start.angle
        time_for_morph = goal.time - start.time
        vel_estim = self.get_length(direction) / time_for_morph
        if vel_estim > MAX_VEL:
            coef = MAX_VEL / vel_estim
            direction = direction[0] * coef, direction[1] * coef
        # print(f"Time for morph: {time_for_morph}")
        num_steps = max(int(time_for_morph / self.dt),100)
        realDT = time_for_morph / num_steps
        # print(f"Num steps: {num_steps}")
        dir_step = direction[0] / num_steps, direction[1] / num_steps
        angle_step = angle_morph / num_steps
        ob_start_pos = ob_old_pos[0] + ob_vx * start.time,ob_old_pos[1] + ob_vy * start.time
        ob_dir_step = ob_vx * realDT, ob_vy * realDT
        ob_start_angle =ob_vangle * start.time
        ob_angle_step = ob_vangle * realDT
        self.moving_obstacle.velocity = 0,0
        self.moving_obstacle.angular_velocity = 0


        for i in range(num_steps):
            self.agent.body.position = start.x + dir_step[0] * i, start.y + dir_step[1] * i
            self.agent.body.angle = start.angle + angle_step * i
            self.moving_obstacle.position = ob_start_pos[0] + ob_dir_step[0] * i, ob_start_pos[1] + ob_dir_step[1] * i
            self.moving_obstacle.angle = ob_start_angle + ob_angle_step * i
            # print(f"Obstacle angle: {self.moving_obstacle.angle}")
            self.space.reindex_shape(self.agent)
            for s in self.moving_obstacle.shapes:
                self.space.reindex_shape(s)

            if self.space.shape_query(self.agent):
                if self.verbose:
                    print("Obstacle encountered")
                break
            if (i % 20 == 0 and i != 0) or i == num_steps - 1:
                if self.clb:
                    self.clb(self.space)
                self.extend_checkpoints(start, checkpoints, self.agent.body.position, self.agent.body.angle, start.time + i * realDT)
        self.agent.body.angle = a_old_angle
        self.agent.body.position = a_old_pos
        self.moving_obstacle.angle = ob_old_angle
        self.moving_obstacle.position = ob_old_pos
        self.moving_obstacle.velocity = self.obstacle_velocity[0], self.obstacle_velocity[1]
        self.moving_obstacle.angular_velocity = self.obstacle_velocity[2]
        self.space.reindex_shape(self.agent)
        for s in self.moving_obstacle.shapes:
            self.space.reindex_shape(s)

        return checkpoints

    @staticmethod
    def get_length(tp):
        return (tp[0] ** 2 + tp[1] ** 2) ** 0.5

    @staticmethod
    def node_from_shape(shape: pymunk.Shape, time=0, parent=None):
        return RRTNodeCalc(shape.body.position.x, shape.body.position.y, shape.body.angle, time, parent)

    def set_debug_callback(self, clb):
        self.clb = clb


#this creates a new space for each move and simualtes the movement
class LocalPlannerSim:
    def __init__(self, space: pymunk.Space,agent: pymunk.Shape):
        self.space = space
        self.agent = agent

    def check_path(self,start:RRTNodeSim, goal: RRTNodeSim) -> List[RRTNodeSim]:
        pass