"""Planning with bezier curves"""

import numpy as np


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import show_sim,make_draw_line,make_draw_circle

from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
import deform_plan.rrt_utils.cable_rrt as vutils


CABLE_LENGTH = 400
SEGMENT_NUM = 70
MAX_FORCE =800
cfg = PMConfig()
cable= Cable([20,20],400,SEGMENT_NUM,thickness=5)
obstacle_g = RandomObstacleGroup(np.array([100,200]),200,200,4,3,seed=25)
top = Rectangle([400,0],800,20,STATIC)
bottom = Rectangle([400,800],800,20,STATIC)
left = Rectangle([0,400],20,800,STATIC)
right = Rectangle([800,400],20,800,STATIC)


# sim = Simulator(cfg, [cable], [top,bottom,left,right])
sim = Simulator(cfg, [cable], [obstacle_g,top,bottom,left,right])

lb = np.array([300,300,0])
ub = np.array([500,500,2*np.pi])
sampler = BezierSampler(CABLE_LENGTH,SEGMENT_NUM,lb,ub,seed=16)
points = sampler.sample(x=300,y=750,angle=0)
# control_idxs = [i for i in range(SEGMENT_NUM)]
control_idxs = [0, 10,SEGMENT_NUM-1]
guider = vutils.make_guider(0,control_idxs,MAX_FORCE)
ender = vutils.make_end_cond_all_vel(0,MAX_FORCE/3,5)
planner = FetchAblePlanner(sim,guider,ender,vutils.make_exporter(0),max_iter_cnt=50000)
print(points)

def draw(surf):
    for i in range(len(points)-1):
        make_draw_line(points[i],points[i+1],2)(surf)
    for i in control_idxs:
        make_draw_circle(points[i],4,(255,0,0))(surf)
        cable.bodies[i].color = (255,0,0,255)


db = DebugViewer(sim)
db.draw_clb = draw

start = planner.form_start_node()

planner.check_path(start,vutils.Point(points, None))






