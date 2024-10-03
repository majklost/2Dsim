"""Planning with bezier curves"""

import numpy as np
import time

from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import show_sim,make_draw_line,make_draw_circle
from deform_plan.utils.analytics import print_analytics


from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
import deform_plan.rrt_utils.cable_rrt as vutils

from helpers.cable_map import get_standard_simulator



CABLE_LENGTH = 400
SEGMENT_NUM = 10
MAX_FORCE =1200
cfg = PMConfig()
cable= Cable([100,50],400,SEGMENT_NUM,thickness=5)
obstacle_g = RandomObstacleGroup(np.array([100,200]),150,400,3,3,seed=20)
sim = get_standard_simulator(cable,obstacle_g)
GUIDER_PERIOD = 10
SHITOKOLO = 0
STEPS = 1000
lb = np.array([0,0,0])
ub = np.array([800,800,2*np.pi])
sampler = BezierSampler(CABLE_LENGTH,SEGMENT_NUM,lb,ub,seed=16)
goal_points = sampler.sample(x=550,y=700,angle=np.pi)
# control_idxs = [i for i in range(SEGMENT_NUM)]
control_idxs = [0,SEGMENT_NUM-1]
# control_idxs = [0]
guider = vutils.make_guider(0,control_idxs,MAX_FORCE)
ender = vutils.make_end_cond_all_vel(0,MAX_FORCE/3,5)

planning_fncs = {
    "guider": guider,
    "fail_condition": ender,
    "reached_condition": vutils.make_reached_condition(0),
    "exporter": vutils.make_exporter(0)
}

planner = FetchAblePlanner(sim,planning_fncs,max_iter_cnt=300,only_simuls=False,sampling_period=1000, guider_period=GUIDER_PERIOD,track_analytics=True)

GOAL = vutils.Point(goal_points,None)
def draw(surf):
    points = goal_points
    for i in range(len(points)-1):
        make_draw_line(points[i],points[i+1],2)(surf)
    for i in control_idxs:
        make_draw_circle(points[i],4,(255,0,0))(surf)
        cable.bodies[i].color = (255,0,0,255)
# db = DebugViewer(sim,realtime=False)
# db.draw_clb = draw
show_sim(sim,clb=draw)
start = planner.form_start_node()
storage = vutils.StorageWrapper(GOAL)
storage.save_to_storage(start)
st = time.time()
REAL_STEPS = STEPS
for i in range(STEPS):
    t1 = time.time()
    if i % 40 == 0:
        print("iter: ",i)
        print("time_in_sim: ", sim.SIMTIME)
    points = sampler.sample()
    # t2 = time.time()
    if storage.try_goal:
        print("Trying goal")
        points = goal_points
        storage.try_goal = False
    q_rand = vutils.Point(points,None)
    q_near = storage.get_nearest(q_rand)
    t4 = time.time()
    response = planner.check_path(q_near,q_rand)
    t5 = time.time()
    for res in response.checkpoints:
        storage.save_to_storage(res)
    if not storage.want_next_iter:
        print("Found path")
        REAL_STEPS = i
        break
    t6 = time.time()
    SHITOKOLO += t4 - t1 + t6-t5

endt = time.time()
print("Time: ",endt-st)




print("Best distance: ",storage.best_dist)
path = storage.get_path()
print(path)
rp = ReplayablePath(sim,path,GOAL,guider,reached_condition=vutils.make_reached_condition(0),guider_period=GUIDER_PERIOD)
rp.additional_data["time"] = endt-st
rp.save("./data/cable.rpath")

print("ShitOKOLO: ", SHITOKOLO)

print_analytics(planner.analytics,REAL_STEPS)
