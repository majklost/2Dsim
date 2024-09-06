
import numpy as np

from src.RRTNode import RRTNodeCable
import pymunk
from pymunk import vec2d
from src.helpers.PygameRenderer import PygameRenderer
from src.helpers.helperFunctions import render_goal,get_bodies_from_space
from typing import List

FORCE_APPLY_THRESHOLD = 5
CHECKPOINT_REDUCE_FACTOR = 10
# CHECKPOINT_REDUCE_FACTOR = 1

class CablePlanner:
    """
    Plans segments of cable from one position to another, reusable object - can be used for multiple queries
    """



    def __init__(self, max_force, FPS=80,verbose=False,rendered=False, auto_stop=True, max_iter_cnt=1000):
        self.max_force = max_force
        self.FPS = FPS
        self.verbose = verbose
        self.renderer = None
        self.auto_stop = auto_stop
        self.max_iter_cnt = max_iter_cnt

        self.num_called = 0
        self.avg_steps = 0
        if rendered:
            self.renderer = PygameRenderer(800, 800, FPS)

    def _fetch_simSpace(self, node:RRTNodeCable) -> '_SimulatorData':
        if node.simSpace is not None:
            if self.verbose:
                print("Using existing simSpace")
            return self._SimulatorData(node.simSpace,self.FPS,self.verbose)
        if node.replayer is None:
            raise ValueError("No replayer in node without simSpace")
        parent_simSpace = node.replayer.parent.simSpace
        if parent_simSpace is None:
            raise ValueError("No simSpace in parent")
        sim_data = self._SimulatorData(parent_simSpace, self.FPS,self.verbose)


        #move to wanted position
        if self.verbose:
            print("Moving to start position with iters: ", node.replayer.iter_cnt)
        for i in range(node.replayer.iter_cnt):
            sim_data.one_iter(node.replayer.real_goal,self._force_divider_equal(len(sim_data.controlled_bodies)))

        #giving simSpace to node, so it can be reused
        node.simSpace = sim_data.space.copy()

        return sim_data

    def _force_divider_equal(self,num_of_controllabes):
        return [self.max_force/num_of_controllabes for _ in range(num_of_controllabes)]


    def check_path(self, start:RRTNodeCable, goals:RRTNodeCable) -> List[RRTNodeCable]:
        self._check_validity(start,goals)
        self.num_called += 1
        sim_data = self._fetch_simSpace(start)

        if len(goals.points) != len(sim_data.controlled_bodies):
            raise ValueError("Different number of goals and controlled bodies")


        checkpoints = []
        iter_cnt = 0
        forces = self._force_divider_equal(len(sim_data.controlled_bodies))
        while True:
            if iter_cnt > self.max_iter_cnt:
                break
            res = sim_data.one_iter(goals.points,forces,lambda: sim_data.check_end_custom(goals.points))
            if res and self.auto_stop:
                break
            if self.renderer is not None:
                self.renderer.update_cur(sim_data.space)
                if not self.renderer.want_running:
                    break
            iter_cnt += 1
            new_node = RRTNodeCable(sim_data.get_positions(),replayer=RRTNodeCable.Replayer(iter_cnt,goals.points,start))
            new_node._movable_bodies = sim_data.get_positions(controlled=False) #for rendering and debugging only
            checkpoints.append(new_node)
        # print("Iter: ", iter_cnt)
        self.avg_steps += 1/self.num_called * (iter_cnt - self.avg_steps)

        return checkpoints[::CHECKPOINT_REDUCE_FACTOR]
    @staticmethod
    def _check_validity(start,goals):
        if start.points is None and start.replayer is None:
            raise ValueError("No points and replayer in start")
        if goals.points is None:
            raise ValueError("No points in goals")
        if start.points is not None and goals.points is not None:
            if len(start.points) != len(goals.points):
                raise ValueError("Different number of points in start and goals")


    class _SimulatorData:
        def __init__(self, space:pymunk.Space, FPS,verbose=False):
            self.space =space.copy()
            self.moveable_bodies = get_bodies_from_space(self.space, "movedID")
            self.controlled_bodies = get_bodies_from_space(self.space, "controlledID")
            self.FPS = FPS
            self.prev_vel = 0
            self.verbose = verbose
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

        def get_positions(self,controlled=True):
            points = []
            if controlled:
                for b in self.controlled_bodies:
                    points.append([b.position.x, b.position.y])
            else:
                for b in self.moveable_bodies:
                    points.append([b.position.x, b.position.y])
            return np.array(points)



        def check_end_custom(self,goals):
            f = sum(map(lambda x: x.force.length, self.controlled_bodies))
            reached = True
            for b in self.controlled_bodies:
                cond =  b.position.get_distance((goals[b.controlledID][0],goals[b.controlledID][1])) < 3
                reached = reached and cond
            if reached:
                if self.verbose:
                    print("reached")
                return True

            if f > 300:
                seg_vel_sum = sum(map(lambda x: x.velocity.length, self.moveable_bodies))
                if seg_vel_sum < 50 and seg_vel_sum < self.prev_vel:
                    if self.verbose:
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
    planner = CablePlanner(MOVING_FORCE,verbose=True,rendered=True)
    # planner.renderer.update_cur_clb = draw_goals
    GOALS = RRTNodeCable(np.array([GOAL,GOAL2]))



    res_nodes = planner.check_path(START,GOALS)
    # NEW_START = res_nodes[30]
    # NEW_GOAlS = RRTNodeCable(NEW_START.replayer.real_goal)
    # print("Replanned")
    # replanned = planner.check_path(NEW_START,NEW_GOAlS)
    #
    # print(res_nodes[-1])
    # print("-"*20)
    # print(replanned[-1])
    #
    # again = planner.check_path(NEW_START,NEW_GOAlS)
    # print(again[-1])
    #
    # assert res_nodes[-1] == replanned[-1]
    # assert replanned[-1] == again[-1]



