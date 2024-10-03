"""Rework of cable playground from previous version of the project"""
import pygame


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer

cfg = PMConfig()
cfg.gravity = 98.1

linear_spring_params = SpringParams(1000, 10)

cable = Cable([100,100],400,30, linear_params=linear_spring_params)
rect = Rectangle([150,50],20,20,DYNAMIC)
cable[-1].body.body_type = KINEMATIC
cable[0].body.body_type = STATIC
sim = Simulator(cfg, [cable], [rect])
sim.damping = 1

dbg = DebugViewer(sim,realtime=True)



for i in range(10000):
    t = pygame.time.get_ticks()
    if t % 10000 < 2500:
        cable[-1].velocity = 30,0
    elif t % 10000 < 5000:
        cable[-1].velocity = 0,30
    elif t % 10000 < 7500:
        cable[-1].velocity = -30,0
    else:
        cable[-1].velocity = 0,-30



    if sim.step():
        break
