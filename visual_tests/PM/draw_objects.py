from deform_plan.assets.PM import *
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.utils.PM_debug_viewer import DebugViewer

cfg = PMConfig()

cross = Cross([400,400],50,10,KINEMATIC)
cross.angular_velocity = 2

rct = Rectangle([500,500],50,10,KINEMATIC)
rct.color = (0,0,255,0)
rct.angular_velocity = 2

rnd_blk = RandomBlock([600,600],50,KINEMATIC)
rnd_blk.color = (255,0,0,0)

s = Simulator(cfg, [cross,rct,rnd_blk], [])

dv = DebugViewer(s,realtime=True)

for i in range(1000):
    if s.step():
        break


