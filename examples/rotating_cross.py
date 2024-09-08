# This file is used to test the importing of the simulator class
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer

cfg = PMConfig()

cross = Cross([400,400],50,10,KINEMATIC
              )
cross.angular_velocity = 2

s = Simulator(cfg, [cross], [])
viewer = DebugViewer(s,realtime=True)

for i in range(1000):
    if s.step():
        break


def import_export():
    cfg = PMConfig()
    crs = Cross([400,400],50,10,KINEMATIC)
    crs.angular_velocity = 2
    movables = [crs]
    s = Simulator(cfg, movables,[])
    s.step()
    s_export = s.export()
    angle_two_step = crs.orientation
    s.step()
    s.step()
    # assert crs.orientation == s._space.bodies[0].angle
    s.import_from(s_export)
    # assert
    # assert s_export.space.bodies[0].angle == angle_two_step
    assert crs.orientation == angle_two_step

import_export()