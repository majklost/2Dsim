import numpy as np

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.planners.fetchable_planner import FetchAblePlanner, FetchablePlannerRequest
from deform_plan.messages.sim_node import NodeReached, NodeGoal

class TestPlanner:
    def prepare(self):
        cfg = PMConfig()
        rect = Rectangle([50,50],10,10,DYNAMIC)
        obstacle = Rectangle([400,200],100,100,KINEMATIC)
        obstacle.velocity = np.array([0,-5])
        self.sim = Simulator(cfg, [rect], [obstacle])


    def test_reload(self):
        self.prepare()
        start = NodeReached(0, sim_export=self.sim.export())
        planner = FetchAblePlanner(self.sim, 0, max_iter_cnt=1000, only_simuls=True)

        goal = NodeGoal(np.array([600,50]),0,500,0)

        req = FetchablePlannerRequest(start=start, goal=goal)
        response = planner.check_path(req)
        assert len(response.checkpoints) > 0
        iters = response.checkpoints[-3].iter_cnt
        c2 = response.checkpoints[-3]
        req2 = FetchablePlannerRequest(start=c2, goal=goal)
        res2 = planner.check_path(req2)
        assert response.checkpoints[-1].iter_cnt > iters > 0
        assert response.checkpoints[-1].iter_cnt == res2.checkpoints[-1].iter_cnt

    def test_moving_obstacles(self):
        self.prepare()
        self.sim.fixed_objects[0].velocity = np.array([0,-5])
        start = NodeReached(0, sim_export=self.sim.export())
        planner = FetchAblePlanner(self.sim, 0, max_iter_cnt=1000, only_simuls=True)
        goal = NodeGoal(np.array([600,50]),0,500,0)
        req = FetchablePlannerRequest(start=start, goal=goal)
        response = planner.check_path(req)
        iters = response.checkpoints[-1].iter_cnt
        assert iters > 0
        assert iters < 1000

        def shit(_):
            self.sim.fixed_objects[0].velocity = np.array([0, -50])

        planner.after_load_clb =shit

        req2 = FetchablePlannerRequest(start=start, goal=goal)
        response2 = planner.check_path(req2)
        assert response2.checkpoints[-1].iter_cnt < iters


