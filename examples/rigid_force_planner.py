"""Choose start and end, planner will try to connect them by a straight line"""
import numpy as np

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.planners.fetchable_planner import FetchAblePlanner, FetchablePlannerRequest
from deform_plan.messages.nodes.sim_node import NodeReached, NodeGoal

cfg = PMConfig()
rect = Rectangle([50,50],100,10,DYNAMIC)
obstacle = Rectangle([400,200],100,100,KINEMATIC)
obstacle.velocity = np.array([0,-5])

sim = Simulator(cfg, [rect], [obstacle])

start = NodeReached(0,sim_export=sim.export())
planner = FetchAblePlanner(sim, 0, max_iter_cnt=1000, only_simuls=True)

dbg = DebugViewer(sim,realtime=True)
dbg.draw_circle([600,50],10,(255,0,0))

goal = NodeGoal(np.array([600,50,np.pi,5000,0]))
#
req = FetchablePlannerRequest(start=start, goal=goal)
response = planner.check_path(req)

# obstacle.velocity = np.array([0,-50])

def shit(_):
    obstacle.velocity = np.array([0,-50])

planner.after_load_clb = shit
# print(len(response.checkpoints))
# c2 = response.checkpoints[-1]
# print(c2.iter_cnt)
req2 = FetchablePlannerRequest(start=start, goal=goal)
res2 = planner.check_path(req2)






