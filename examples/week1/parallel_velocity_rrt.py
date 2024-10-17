from pathos.pp import ParallelPool


import numpy as np
import time


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.samplers.ndim_sampler import NDIMSampler
import deform_plan.rrt_utils.velocity_rrt as vutils
from deform_plan.utils.PM_space_visu import show_sim, make_draw_circle,make_draw_line
NUM_PLANNERS = 5
pool = ParallelPool(nodes=NUM_PLANNERS)

def make_planners(num_planners):
    planners = []
    for n in range(num_planners):
        cfg = PMConfig()
        rect = Rectangle([50, 50], 60, 10, DYNAMIC)
        rect.track_colisions = True
        obstacle = Rectangle([150, 300], 300, 50, STATIC)
        obstacle2 = Rectangle([650, 300], 300, 50, STATIC)
        obstacle3 = Rectangle([350, 500], 700, 50, STATIC)
        obstacle4 = Rectangle([800 - 350, 600], 700, 50, STATIC)
        cross = Cross([400, 300], 90, 10, KINEMATIC)
        cross.angular_velocity = np.pi / 4
        GUIDER_PERIOD = 10

        sim = Simulator(cfg, [rect], [obstacle, obstacle2, obstacle3, obstacle4, cross])
        guider = vutils.make_guider(0, 2000, 100)
        planning_fncs = {
            'guider': guider,
            'fail_condition': vutils.make_fail_condition(0),
            'reached_condition': vutils.make_reached_condition(0),
            'exporter': vutils.make_exporter(0)


        }
        planner = FetchAblePlanner(sim, planning_fncs, max_step_cnt=5000, only_simuls=False, sampling_period=2000,
                                   guider_period=GUIDER_PERIOD)
        planners.append(planner)
    return planners

pos = [700,700]

def draw(surt):
    make_draw_circle(pos,5)(surt)

def prepare_samples(num_planners):
    tasks = []
    for i in range(num_planners):
        x, y, rot, iter_cnt = sampler.sample()
        # print(x,y,rot,iter_cnt)
        g = vutils.Goal()
        g.pos = np.array([x, y])
        g.rot = rot

        g.iter_cnt = iter_cnt
        q_near = storage.get_nearest(g)
        tasks.append((q_near,g))
    return tasks

def run(planner,task):
    response = planner.check_path(task[0],task[1])
    return response

planners = make_planners(NUM_PLANNERS)
show_sim(planners[0].simulator,clb=draw)
pos = [700,700]
goal = vutils.Goal()
goal.pos = np.array(pos)
goal.rot = np.pi/2
goal.iter_cnt = 1000000
start = planners[0].form_start_node()

lower_b = np.array([0,0,0,1])
upper_b = np.array([800,800,2*np.pi,10000])

sampler = NDIMSampler(lower_b,upper_b,5)

storage = vutils.StorageWrapper(goal)
storage.save_to_storage(start)
t = time.time()
for i in range(100):
    if i%100 == 0:
        print("time: ",time.time()-t)
        print("iter: ", i)
    tasks = prepare_samples(NUM_PLANNERS)
    results = pool.imap(run,planners,tasks)

    for response in results:
        for r in response.checkpoints:
            storage.save_to_storage(r)
        if not storage.want_next_iter:
            print("Reached goal_points in iter: ", i)
            break
tt = time.time()-t
print("Done, time taken: ",tt)
print("Best dist: ", storage.best_dist)



