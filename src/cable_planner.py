
import numpy as np

from src.RRTNode import RRTNodeCable
import pymunk
from pymunk import vec2d
from src.helpers.pygameRenderer import pygameRenderer
from src.helpers.helperFunctions import render_goal,get_points_from_space
from typing import List

FORCE_APPLY_THRESHOLD = 5
# CHECKPOINT_REDUCE_FACTOR = 10
CHECKPOINT_REDUCE_FACTOR = 1

class CablePlanner:
    """
    Plans segments of cable from one position to another, reusable object - can be used for multiple queries
    """



    def __init__(self, max_force, FPS=80,verbose=False,rendered=False):
        self.max_force = max_force
        self.FPS = FPS
        self.verbose = verbose
        if rendered:
            self.renderer = pygameRenderer(800,800,FPS)

    def _fetch_simSpace(self, node:RRTNodeCable) -> '_SimulatorData':
        if node.simSpace is not None:
            if self.verbose:
                print("Using existing simSpace")
            return self._SimulatorData(node.simSpace,self.FPS)
        if node.replayer is None:
            raise ValueError("No replayer in node without simSpace")
        parent_simSpace = node.replayer.parent.simSpace
        if parent_simSpace is None:
            raise ValueError("No simSpace in parent")
        sim_data = self._SimulatorData(parent_simSpace, self.FPS)


        #move to wanted position
        for i in range(node.replayer.iter_cnt):
            sim_data.one_iter(node.replayer.real_goal,self._force_divider_equal(len(sim_data.controlled_bodies)))

        return sim_data

    def _force_divider_equal(self,num_of_controllabes):
        return [self.max_force/num_of_controllabes for _ in range(num_of_controllabes)]


    def check_path(self, start:RRTNodeCable, goals:RRTNodeCable) -> List[RRTNodeCable]:
        sim_data = self._fetch_simSpace(start)
        checkpoints = []
        iter_cnt = 0
        forces = self._force_divider_equal(len(sim_data.controlled_bodies))
        while True:
            res = sim_data.one_iter(goals.points,forces,lambda: sim_data.check_end_custom(goals.points))
            if res:
                break
            if self.renderer is not None:
                self.renderer.update_cur(sim_data.space)
                if not self.renderer.want_running:
                    break
            iter_cnt += 1
            new_node = RRTNodeCable(sim_data.get_positions(),replayer=RRTNodeCable.Replayer(iter_cnt,goals.points,start))
            checkpoints.append(new_node)
        print("Iter: ", iter_cnt)


        return checkpoints[::CHECKPOINT_REDUCE_FACTOR]

    class _SimulatorData:
        def __init__(self, space:pymunk.Space, FPS):
            self.space =space.copy()
            self.moveable_bodies = get_points_from_space(self.space,"movedID")
            self.controlled_bodies = get_points_from_space(self.space,"controlledID")
            self.FPS = FPS
            self.prev_vel = 0
        def _apply_force(self,forces,vecs,distances):
            for idx,b in enumerate(self.controlled_bodies):
                if distances[idx] > FORCE_APPLY_THRESHOLD:
                    b.apply_force_at_local_point(forces[idx]* vecs[idx],(0,0))


        def _get_distances(self, goals):
            distances = []
            for i in range(len(self.controlled_bodies)):
                cur = self.controlled_bodies[i].position
                goal = goals[i][0], goals[i][1]
                dist = cur.get_distance(goal)
                distances.append(dist)
            return distances

        def one_iter(self,goal, forces, end_check_clb=None):
            vecs = self._get_vecs(goal)
            distances = self._get_distances(goal)
            self._apply_force(forces,vecs, distances)
            if end_check_clb is not None and end_check_clb():
                return True
            self.space.step(1 / self.FPS)
            return False

        def _get_vecs(self,goals):
            """
            Returns normalized vectors from current position of each controlled segment to goal
            :param goals:
            :return:
            """
            vecs = []
            for i in range(len(self.controlled_bodies)):
                cur = self.controlled_bodies[i].position
                goal = goals[i]
                vec = vec2d.Vec2d( goal[0]- cur.x, goal[1] - cur.y)
                good_vec = vec.rotated(-self.controlled_bodies[i].angle).normalized()
                vecs.append(good_vec)
            return vecs

        def get_positions(self):
            points = []
            for b in self.controlled_bodies:
                points.append([b.position.x, b.position.y])
            return np.array(points)



        def check_end_custom(self,goals):
            f = sum(map(lambda x: x.force.length, self.controlled_bodies))
            reached = True
            for b in self.controlled_bodies:
                cond =  b.position.get_distance((goals[b.controlledID][0],goals[b.controlledID][1])) < 3
                reached = reached and cond
            if reached:
                print("reached")
                return True

            if f > 300:
                seg_vel_sum = sum(map(lambda x: x.velocity.length, self.moveable_bodies))
                if seg_vel_sum < 50 and seg_vel_sum < self.prev_vel:
                    print("terminate", seg_vel_sum)
                    return True
                self.prev_vel = seg_vel_sum
            return False












if __name__ == "__main__":
    GOAL = 100,500
    GOAL2 = 50,500
    CABLE_LENGTH = 200
    CABLE_SEGMENTS = 30
    MOVING_FORCE = 1000
    from src.helpers.cables import MultibodyCable
    from src.helpers.objectLibrary import RandomBlock
    space = pymunk.Space()
    space.damping = .1
    cable = MultibodyCable(0, 20, CABLE_LENGTH, CABLE_SEGMENTS, MultibodyCable.standardParams, thickness=5)
    cable.add(space)

    obstacle = RandomBlock(100, 100, 50, 0)
    obstacle.add(space)

    for i, s in enumerate(cable.segments):
        s.movedID = i
    cable.segments[-1].controlledID = 0
    cable.segments[0].controlledID = 1
    for x in cable.segments[-1].shapes:
        x.color = (0,255,0,255)

    def draw_goals(display,space):
        for g in GOALS:
            render_goal(display,g)

    START = RRTNodeCable(simSpace=space)
    planner = CablePlanner(MOVING_FORCE,verbose=False,rendered=True)
    # planner.renderer.update_cur_clb = draw_goals
    GOALS = RRTNodeCable(np.array([GOAL,GOAL2]))



    res_nodes = planner.check_path(START,GOALS)
    # print("Nodes: \n", res_nodes)
    print("Nodes len: " , len(res_nodes))
    # print(res_node.parent)
    # print(res_node.points)
    # full_node = CablePlannerSingle(RRTNodeCable(simSpace=space),MOVING_FORCE,verbose=False)
    # diffs = []
    # predicted_points = []
    # simulated_points = []
    # SAMPLES = 100
    # for i in range(SAMPLES):
    #     predicted = res_node.parent.points[1] + (res_node.points[1]-res_node.parent.points[1])/SAMPLES*i
    #     print("Predicted: ", predicted)
    #     simulated = full_node.check_path(GOALS,maxIter=res_node.replayer.iter_cnt//SAMPLES)
    #     print("Simulated: ", simulated.points[1])
    #     diff = np.sqrt(np.sum((predicted - simulated.points[1])**2))
    #     print("Diff: ", diff)
    #     diffs.append(diff)
    #     predicted_points.append(predicted)
    #     simulated_points.append(simulated.points[1])
    # from matplotlib import pyplot as plt
    # predicted_points_x = [x[0] for x in predicted_points]
    # predicted_points_y = [x[1] for x in predicted_points]
    # simulated_points_x = [x[0] for x in simulated_points]
    # simulated_points_y = [x[1] for x in simulated_points]
    # plt.plot(predicted_points_x,predicted_points_y)
    # plt.plot(simulated_points_x,simulated_points_y,linestyle="--")
    # plt.show()


