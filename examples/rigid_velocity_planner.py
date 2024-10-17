import numpy as np

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.planners.fetchable_planner import FetchAblePlanner
import deform_plan.rrt_utils.velocity_rrt as vutils



cfg = PMConfig()
rect = Rectangle([50,50],100,10,DYNAMIC)
obstacle = Rectangle([400,200],100,100,KINEMATIC)
obstacle.velocity = np.array([0,-50])

sim = Simulator(cfg, [rect], [obstacle])


planner = FetchAblePlanner(sim, vutils.make_guider(0), vutils.make_exporter(0), only_simuls=False)



dbg = DebugViewer(sim,realtime=True)
dbg.draw_circle([600,50],5,(255,0,0))

# goal_points = vutils.Goal(np.array([600,50,np.pi/2,500,250]))
goal = vutils.Goal()
goal.pos = np.array([600,50])
goal.rot = np.pi/2
goal.iter_cnt = 500

start = planner.form_start_node()

response = planner.check_path(start,goal)