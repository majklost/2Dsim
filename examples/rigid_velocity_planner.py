import numpy as np

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.planners.fetchable_planner import FetchablePlannerRequest
from deform_plan.planners.velocity_planner import VelocityPlanner
from deform_plan.messages.nodes.sim_node import NodeReached, NodeGoal

cfg = PMConfig()
rect = Rectangle([50,50],100,10,DYNAMIC)
obstacle = Rectangle([400,200],100,100,KINEMATIC)
obstacle.velocity = np.array([0,-50])

sim = Simulator(cfg, [rect], [obstacle])

start = NodeReached(0,sim_export=sim.export())
planner = VelocityPlanner(sim, 0, max_iter_cnt=1000, only_simuls=True)

dbg = DebugViewer(sim,realtime=True)
dbg.draw_circle([600,50],5,(255,0,0))

goal = NodeGoal(np.array([600,50,np.pi/2,500,250]))

req = FetchablePlannerRequest(start=start, goal=goal)
response = planner.check_path(req)