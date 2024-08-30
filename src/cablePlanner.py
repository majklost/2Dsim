
import numpy as np

from src.RRTNode import RRTNodeCable
import pymunk
from pymunk import vec2d
from src.helpers.pygameRenderer import pygameRenderer
from src.helpers.helperFunctions import render_goal,get_points_from_space
from typing import List




class CablePlannerSingle:
    """
    Plans a segment of cable from one position to another
    """
    def __init__(self,start:RRTNodeCable, max_force, FPS=80,verbose=False):
        """

        :param start: current node with space, new space will be copied from this,
        it is assumed that bodies which can be controlled has attribute "controlled" and
        bodies that are moved by the cable has attribute "moved"
        :param max_force: maximum cumulative force that can be applied to the cable (if multiple controllable segments, force is divided among them)
        :param FPS:
        :param verbose: if debug info should be printed
        """
        self.start_node = start
        self.space = start.simSpace.copy()
        self.max_force = max_force
        self.moveable_bodies = get_points_from_space(self.space,"movedID")
        self.controlled_bodies = get_points_from_space(self.space,"controlledID")
        self.renderer = None
        if verbose:
            print("Moveable bodies count: ", len(self.moveable_bodies), " Controlled bodies count: ", len(self.controlled_bodies))
            self.renderer = pygameRenderer(800,800,FPS)
        self.FPS = FPS

        self.prev_vel = 0



    def check_path(self, goals: RRTNodeCable,maxIter=-1) -> List[RRTNodeCable]:

        assert goals.points.shape[0] == len(self.controlled_bodies), "Number of goals must be same as number of controlled bodies"
        checkpoints = []
        """

        :param goals: positions for controlled segments - number of them must be same as number of controlled bodies
        :return: new checkpoints to tree
        It is expected that goals do not have simulator in it
        """
        partial_force = self.max_force/len(self.controlled_bodies)
        iter_cnt = 0
        #TODO rewrite, so when one force is zero the another one can be applied
        while True:
            if maxIter != -1 and iter_cnt > maxIter:
                break
            vecs = self._get_vecs(goals)
            distances = self._get_distances(goals)
            for idx,b in enumerate(self.controlled_bodies):
                if distances[idx] >5 : # 5 was choosen arbitrarily
                    b.apply_force_at_local_point(partial_force*vecs[idx],(0,0))
                for s in b.shapes:
                    s.color = (255,0,0,255)
            if self._check_end(goals) and self.renderer is None:
                break
            self.space.step(1/self.FPS)

            if self.renderer is not None:
                self.renderer.update_cur(self.space)
                if not self.renderer.want_running:
                    break
            iter_cnt += 1
            self._extend_checkpoints(iter_cnt, goals.points, checkpoints)
            # if iter_cnt % 10 == 0:
            #     print("Iter: ", iter_cnt)
        print("Iter: ", iter_cnt)
        return self._shrink_list(checkpoints)

    def _get_distances(self, goals):
        distances = []
        for i in range(len(self.controlled_bodies)):
            cur = self.controlled_bodies[i].position
            goal = goals[i][0], goals[i][1]
            dist = cur.get_distance(goal)
            distances.append(dist)
        return distances

    def _extend_checkpoints(self, iter_cnt,goal_points,checkpoints: List[RRTNodeCable]) -> None:
        """
        Forms a new node from current positions of controlled segments
        :return:
        """
        points = []
        for b in self.controlled_bodies:
            points.append([b.position.x, b.position.y])
        new_node = RRTNodeCable(np.array(points),replayer=RRTNodeCable.Replayer(iter_cnt,goal_points), parent=self.start_node)
        checkpoints.append(new_node)


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
            # print("Vec: ", vec)
            good_vec = vec.rotated(-self.controlled_bodies[i].angle).normalized()
            # print("Good vec: ", good_vec)
            vecs.append(good_vec)
            # vecs.append(vec)
        return vecs

    def _check_end(self,goals):
        f = sum(map(lambda x: x.force.length, self.controlled_bodies))
        # print("Force: ", f)
        reached = True
        for b in self.controlled_bodies:
            cond =  b.position.get_distance((goals[b.controlledID][0],goals[b.controlledID][1])) < 3
            reached = reached and cond
        if reached:
            print("reached")
            return True

        if f > 300:
            seg_vel_sum = sum(map(lambda x: x.velocity.length, self.moveable_bodies))
            # print("Force: ", f, "Vel sum: ", seg_vel_sum)
            if seg_vel_sum < 50 and seg_vel_sum < self.prev_vel:
                print("terminate", seg_vel_sum)
                return True
            self.prev_vel = seg_vel_sum
        return False
    @staticmethod
    def _shrink_list(ls, reduce_factor=10):
        """
        keeps each reduce_factor element
        :param ls:
        :return:
        """
        return ls[::reduce_factor]


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

    planner = CablePlannerSingle(RRTNodeCable(simSpace=space),MOVING_FORCE,verbose=False)
    # planner.renderer.update_cur_clb = draw_goals

    GOALS = RRTNodeCable(np.array([GOAL,GOAL2]))



    res_nodes = planner.check_path(GOALS)
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


