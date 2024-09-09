"""Movement of a rectangle with the arrow keys."""

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.controllers.PM_rectangle_controller import PMRectangleController

cfg = PMConfig()

rect = Rectangle([100,100],200,20,DYNAMIC)
sim = Simulator(cfg, [rect], [])
sim.damping = .1

rect_controller = PMRectangleController(rect, moving_force=20000)

dbg = DebugViewer(sim,realtime=True)
dbg.controller = rect_controller

for i in range(10000):
    if sim.step():
        break