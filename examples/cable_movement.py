"""Rework of cable playground from previous version of the project"""



from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.controllers.PM_cable_controller import PMCableController

cfg = PMConfig()

cable = Cable([100,100],400,50,thickness=5)
sim = Simulator(cfg, [cable], [])
sim.damping = .2

cable_controller = PMCableController(cable)


dbg = DebugViewer(sim,realtime=True)
dbg.controller = cable_controller

# sim.export()

for i in range(10000):
    if sim.step():
        break
