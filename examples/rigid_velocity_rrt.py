from math import trunc

import numpy as np
import time


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.samplers.ndim_sampler import NDIMSampler
import deform_plan.rrt_utils.velocity_rrt as vutils
from deform_plan.utils.PM_space_visu import show_sim, make_draw_circle,make_draw_line
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.utils.analytics import print_analytics

#obstacles
cfg = PMConfig()
rect = Rectangle([50,50],60,10,DYNAMIC)
rect.track_colisions = True
obstacle = Rectangle([150,300],300,50,STATIC)
obstacle2 = Rectangle([650,300],300,50,STATIC)
obstacle3 = Rectangle([350,500],700,50,STATIC)
obstacle4 = Rectangle([800-350,600],700,50,STATIC)
cross = Cross([400,300],90,10,KINEMATIC)
cross.angular_velocity = np.pi/4
GUIDER_PERIOD = 10
STEP_CNT = 1000
ANALYTICS = True

sim = Simulator(cfg, [rect], [obstacle,obstacle2,obstacle3,cross])
# _sim = Simulator(cfg, [rect], [obstacle,obstacle2,cross])

guider = vutils.make_guider(0,2000,100)

planning_fncs = {
    "guider": guider,
    "fail_condition": vutils.make_fail_condition(0),
    "reached_condition": vutils.make_reached_condition(0),
    "exporter": vutils.make_exporter(0)
}

planner = FetchAblePlanner(sim, planning_fncs, max_step_cnt=5000, only_simuls=False, sampling_period=100,
                           guider_period=GUIDER_PERIOD, track_analytics=ANALYTICS)

pos = [700,700]
# pos = [400,450]
def draw(surt):
    make_draw_circle(pos,5)(surt)

show_sim(sim,clb=draw)


# dbg = DebugViewer(_sim,realtime=False)
# dbg.draw_circle(pos,5,(255,0,0))



goal = vutils.Goal()
goal.pos = np.array(pos)
goal.rot = np.pi/2
goal.iter_cnt = 1000000

start = planner.form_start_node()

# near_neigh =[]

lower_b = np.array([0,0,0,1])
upper_b = np.array([800,800,2*np.pi,10000])

sampler = NDIMSampler(lower_b,upper_b,5)

storage = vutils.StorageWrapper(goal)
storage.save_to_storage(start)
TOTAL_STEP = STEP_CNT
SHITOKOLO = 0
t = time.time()
for i in range(STEP_CNT):
    if i%200 == 0:
        print("iter: ", i)
    t1 = time.time()
    x,y,rot,iter_cnt = sampler.sample()
    # print(x,y,rot,iter_cnt)
    g = vutils.Goal()
    g.pos = np.array([x,y])
    g.rot = rot

    g.iter_cnt = iter_cnt
    q_near = storage.get_nearest(g)
    # near_neigh.append(q_near)
    t2 = time.time()
    response = planner.check_path(q_near,g)
    t3 = time.time()
    for r in response.checkpoints:
        storage.save_to_storage(r)
    if not storage.want_next_iter:
        TOTAL_STEP = i
        print("Reached goal_points in iter: ",TOTAL_STEP)
        break
    t4 = time.time()
    SHITOKOLO += t4-t3 + t2 -t1
tt = time.time()-t
print("Done, time taken: ",tt)
print("Best dist: ", storage.best_dist)
path= storage.get_path()
if len(path) != 0:
    rp = ReplayablePath(sim,path,goal,guider,vutils.make_reached_condition(0),guider_period=GUIDER_PERIOD)
    rp.additional_data["time"] = tt
    rp.save("./data/good_velocity.rpath")

all_pnts = storage.get_all_points()
print("Number of points: ",len(all_pnts))

def clb(surf):
    make_draw_circle(goal.pos,10,color=(255,0,0))(surf)
    for i,n in enumerate(all_pnts):
        # print(n)
        make_draw_circle(n.exporter_data["pos"],5,color=(0,255,0))(surf)
        parent = n.replayer.parent
        if parent is not None:
            make_draw_line(n.exporter_data["pos"],parent.exporter_data["pos"],2)(surf)


    for i,nd in enumerate(path):
        p = nd.exporter_data["pos"]
        make_draw_circle(p,5)(surf)
        if i > 0:
            p2 = path[i-1].exporter_data["pos"]
            make_draw_line(p,p2,5)(surf)

for p in path:
    print(p)

show_sim(sim,clb=clb)
print("Shit okolo: ", SHITOKOLO)
print_analytics(planner.analytics,TOTAL_STEP)




# st_t = time.time()
# response = planner.check_path(start,goal_points)
# print("time taken: ",time.time()-st_t)