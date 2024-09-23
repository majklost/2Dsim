"""Planning with bezier curves"""

import numpy as np
import time

from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import show_sim,make_draw_line,make_draw_circle

from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
import deform_plan.rrt_utils.cable_rrt as vutils
CABLE_LENGTH = 400
SEGMENT_NUM = 50
MAX_FORCE =800
cfg = PMConfig()
cable= Cable([20,20],400,SEGMENT_NUM,thickness=5)
obstacle_g = RandomObstacleGroup(np.array([100,200]),200,200,3,3,seed=25)
top = Rectangle([400,0],800,20,STATIC)
bottom = Rectangle([400,800],800,20,STATIC)
left = Rectangle([0,400],20,800,STATIC)
right = Rectangle([800,400],20,800,STATIC)
sim = Simulator(cfg, [cable], [obstacle_g,top,bottom,left,right])
lb = np.array([0,0,0])
ub = np.array([800,800,2*np.pi])
sampler = BezierSampler(CABLE_LENGTH,SEGMENT_NUM,lb,ub,seed=16)
goal_points = sampler.sample(x=300,y=750,angle=0)
control_idxs = [i for i in range(SEGMENT_NUM)]
# control_idxs = [0,SEGMENT_NUM-1]
# control_idxs = [0]
guider = vutils.make_guider(0,control_idxs,MAX_FORCE)
ender = vutils.make_end_cond_all_vel(0,MAX_FORCE/3,5)
planner = FetchAblePlanner(sim,guider,ender,vutils.make_exporter(0),max_iter_cnt=300,only_simuls=True,sampling_period=2000)

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
for i in range(2000):
    if i % 10 == 0:
        print("iter: ",i)
    points = sampler.sample()
    q_rand = vutils.Point(points,None)

    q_near = storage.get_nearest(q_rand)
    # print("qrand: ",q_rand)
    response = planner.check_path(q_near,q_rand)
    for res in response.checkpoints:
        storage.save_to_storage(res)
    if not storage.want_next_iter:
        print("Found path")
        break
endt = time.time()
print("Time: ",endt-st)
print("Best distance: ",storage.best_dist)
path = storage.get_path()
print(path)
rp = ReplayablePath(sim,path,GOAL,guider)
rp.additional_data["time"] = endt-st
rp.save("./data/cable.rpath")
