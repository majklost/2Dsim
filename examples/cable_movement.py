"""Rework of cable playground from previous version of the project"""
import numpy as np


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.controllers.PM_cable_controller import PMCableController
from deform_plan.helpers.seed_manager import init_manager
# from week2.w2_utils import calc_distance_matrix,calc_stretch_index

cfg = PMConfig()
init_manager(10,15)



obstacle_g = RandomObstacleGroup(np.array([100,200]),200,200,4,3)
obstacle_g.color = (0,255,0,255)
cable = Cable([450,80],300,40,thickness=5,angle=np.pi)
sim = Simulator(cfg, [cable], [obstacle_g])

cable_controller = PMCableController(cable,moving_force=500)
# dist_matrix = calc_distance_matrix(sim,0)

dbg = DebugViewer(sim,realtime=True)
dbg.controller = cable_controller

# _sim.export()

for i in range(10000):
    if i %10 ==0:
        print(cable.outer_collision_idxs)
        # print(calc_stretch_index(cable.position, dist_matrix))
    if sim.step():
        break
