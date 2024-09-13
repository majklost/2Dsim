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


#obstacles
cfg = PMConfig()
rect = Rectangle([50,50],30,10,DYNAMIC)
obstacle = Rectangle([150,300],300,50,STATIC)
obstacle2 = Rectangle([650,300],300,50,STATIC)
cross = Cross([400,300],90,10,KINEMATIC)
cross.angular_velocity = np.pi/4


sim = Simulator(cfg, [rect], [obstacle,obstacle2,cross])

guider = vutils.make_guider(0,100)

planner = FetchAblePlanner(sim,guider,vutils.make_end_condition(0),vutils.make_exporter(0),only_simuls=False,sampling_period=10)

pos = [600,600]
# pos = [400,60]
# show_sim(sim)
# dbg = DebugViewer(sim,realtime=True)
# dbg.draw_circle(pos,5,(255,0,0))


goal = vutils.Goal()
goal.pos = np.array(pos)
goal.rot = np.pi/2
goal.iter_cnt = 50000

start = planner.form_start_node()



lower_b = np.array([0,0,0,1])
upper_b = np.array([800,800,2*np.pi,5000])

sampler = NDIMSampler(lower_b,upper_b,5)

storage = vutils.StorageWrapper(goal)
storage.save_to_storage(start)
t = time.time()
for i in range(10000):
    if i%100 == 0:
        print("iter: ", i)

    x,y,rot,iter_cnt = sampler.sample()
    # print(x,y,rot,iter_cnt)
    g = vutils.Goal()
    g.pos = np.array([x,y])
    g.rot = rot
    g.iter_cnt = iter_cnt
    q_near = storage.get_nearest(g)
    response = planner.check_path(q_near,g)
    for r in response.checkpoints:
        storage.save_to_storage(r)
    if not storage.want_next_iter:
        print("Reached goal in iter: ",i)
        break
tt = time.time()-t
print("Done, time taken: ",tt)
path= storage.get_path()
if len(path) != 0:
    rp = ReplayablePath(sim,path,goal,guider)
    rp.save("./data/velocity.rpath")

def clb(surf):
    make_draw_circle(goal.pos,10,color=(255,0,0))(surf)
    for i,nd in enumerate(path):
        p = nd.exporter_data["pos"]
        make_draw_circle(p,5)(surf)
        if i > 0:
            p2 = path[i-1].exporter_data["pos"]
            make_draw_line(p,p2,2)(surf)


for p in path:
    print(p)
show_sim(sim,clb=clb)





# st_t = time.time()
# response = planner.check_path(start,goal)
# print("time taken: ",time.time()-st_t)