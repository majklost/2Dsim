import numpy as np

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.messages.sim_node import SimNode

from deform_plan.rrt_utils.velocity_rrt import Goal,make_guider,make_end_condition,make_exporter



class TestPlanner:
    def prepare(self):
        cfg = PMConfig()
        rect = Rectangle([50,50],10,10,DYNAMIC)
        obstacle = Rectangle([400,200],100,100,KINEMATIC)
        obstacle.velocity = np.array([0,-5])
        self.sim = Simulator(cfg, [rect], [obstacle])
        self.planner = FetchAblePlanner(self.sim, make_guider(0, 100), make_end_condition(0), make_exporter(0),
                                        sampling_period=10)


    def test_reload(self):
        self.prepare()



        start = self.planner.form_start_node()

        goal = Goal()
        goal.pos = np.array([600,600])
        goal.rot = 0
        goal.iter_cnt = 500

        response = self.planner.check_path(start,goal)
        assert len(response.checkpoints) > 0
        iters = response.checkpoints[-3].all_iter_cnt
        c2 = response.checkpoints[-3]
        res2 = self.planner.check_path(start=c2, goal=goal)
        # res2 = planner.check_path(req2)
        assert response.checkpoints[-1].all_iter_cnt > iters > 0
        assert response.checkpoints[-1].all_iter_cnt == res2.checkpoints[-1].all_iter_cnt

    def test_moving_obstacles(self):
        self.prepare()
        self.sim.fixed_objects[0].velocity = np.array([0,-5])
        start = self.planner.form_start_node()
        goal = Goal()
        goal.pos = np.array([600,50])
        goal.rot = 0
        goal.iter_cnt = 500

        response = self.planner.check_path(start=start, goal=goal)
        iters = response.checkpoints[-1].all_iter_cnt
        assert iters > 0
        assert iters < 1000

        def shit(_):
            self.sim.fixed_objects[0].velocity = np.array([0, -20])

        self.planner.after_load_clb =shit


        response2 = self.planner.check_path(start,goal)
        assert response2.checkpoints[-1].all_iter_cnt < iters


