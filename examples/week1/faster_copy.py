"""recreate the thing with ball bouncing on cable, compare manual safe with calling copy"""
import numpy as np
import pygame


from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer


def check_validity(sim1, sim2):
    for i in range(len(sim1._space.bodies)):
        assert np.allclose(sim1._space.bodies[i].position, sim2._space.bodies[i].position), (
            sim1._space.bodies[i].position, sim2._space.bodies[i].position)
        assert np.allclose(sim1._space.bodies[i].velocity, sim2._space.bodies[i].velocity), (
            sim1._space.bodies[i].velocity, sim2._space.bodies[i].velocity)
        assert np.allclose(sim1._space.bodies[i].angular_velocity, sim2._space.bodies[i].angular_velocity), (
            sim1._space.bodies[i].angular_velocity, sim2._space.bodies[i].angular_velocity)
        assert np.allclose(sim1._space.bodies[i].angle, sim2._space.bodies[i].angle), (
            sim1._space.bodies[i].angle, sim2._space.bodies[i].angle)


def get_simulator():
    cfg = PMConfig()
    cfg.gravity = 98.1

    linear_spring_params = SpringParams(800, 10)

    cable = Cable([100,100],400,30, linear_params=linear_spring_params)
    rect = Rectangle([150,50],20,20,DYNAMIC)
    cable[-1].body.body_type = KINEMATIC
    cable[0].body.body_type = STATIC
    sim = Simulator(cfg, [cable], [rect],unstable_sim=True)
    return sim



#step the ground truth
sim = get_simulator()
tester  = get_simulator()


cable =  sim.movable_objects[0]

#step it a bit
for i in range(10000):
    if i % 1000 < 250:
        cable[-1].velocity = 10,0
    elif i % 1000 < 500:
        cable[-1].velocity = 0,10
    elif i % 1000 < 750:
        cable[-1].velocity = -10,0
    else:
        cable[-1].velocity = 0,-10




    if sim.step():
        break

# cable[-1].velocity =0,0


#now transfer informaction from sim to tester
cable_tester = tester.movable_objects[0]
# for i in range(len(sim._space.bodies)):
#     tester._space.bodies[i].position = sim._space.bodies[i].position
#     tester._space.bodies[i].velocity = sim._space.bodies[i].velocity
#     tester._space.bodies[i].angle = sim._space.bodies[i].angle
#     tester._space.bodies[i].angular_velocity = sim._space.bodies[i].angular_velocity
#     tester._space.bodies[i].force = sim._space.bodies[i].force
    # print(sim._space.bodies[i],id(sim._space.bodies[i]))

# rect = sim.fixed_objects[0]
# rect_tester = tester.fixed_objects[0]
# rect_tester._body.position = rect._body.position
# rect_tester._body.velocity = rect._body.velocity
# rect_tester._body.angular_velocity = rect._body.angular_velocity
# rect_tester._body.angle = rect._body.angle


#now step both and compare results
def compare_sims(sim1:Simulator,sim2 :Simulator):
    for i in range(len(sim1._space.bodies)):
        assert np.allclose(sim1._space.bodies[i].position, sim2._space.bodies[i].position)
        assert np.allclose(sim1._space.bodies[i].velocity, sim2._space.bodies[i].velocity)
        assert np.allclose(sim1._space.bodies[i].angular_velocity, sim2._space.bodies[i].angular_velocity)
        assert np.allclose(sim1._space.bodies[i].angle, sim2._space.bodies[i].angle)
    # dbg = DebugViewer(sim,realtime=False)


    for x in range(10000):

        if x % 1000 < 250:
            cable[-1].velocity = 30, 0
            cable_tester[-1].velocity = 30, 0
        elif x % 1000 < 500:
            cable[-1].velocity = 0, 30
            cable_tester[-1].velocity = 0, 30
        elif x % 1000 < 750:
            cable[-1].velocity = -30, 0
            cable_tester[-1].velocity = -30, 0
        else:
            cable[-1].velocity = 0, -30
            cable_tester[-1].velocity = 0, -30
        if x%100==0:
            print(x)
        sim1.step()
        sim2.step()

    check_validity(sim1,sim2)




# dbg = DebugViewer(tester,realtime=False)
# dbg2 = DebugViewer(sim,realtime=False)
exported = sim.export()
tester.import_from(exported)
compare_sims(sim,tester)




