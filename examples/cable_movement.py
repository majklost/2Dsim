"""Rework of cable playground from previous version of the project"""
import numpy as np


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.controllers.PM_cable_controller import PMCableController

cfg = PMConfig()



obstacle_g = RandomObstacleGroup(np.array([100,200]),200,200,4,3,seed=20)
obstacle_g.color = (0,255,0,255)
cable = Cable([100,100],400,20,thickness=5)
sim = Simulator(cfg, [cable], [obstacle_g])

cable_controller = PMCableController(cable,moving_force=800)


dbg = DebugViewer(sim,realtime=True)
dbg.controller = cable_controller

# sim.export()

for i in range(10000):
    if sim.step():
        break
