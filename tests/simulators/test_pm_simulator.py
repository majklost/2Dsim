import pytest

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *

class TestSimulator:
    def test_init(self):
        cfg = PMConfig()
        movables = [PMSingleBodyObject(KINEMATIC)]
        s = Simulator(cfg, movables, [])
        assert s.movable_objects == movables


    def test_import_export(self):
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


        # assert angle_one_step == angle_two_step


    def test_wrong_import(self):
        cfg = PMConfig()
        crs = Cross([400, 400], 50, 10, KINEMATIC)
        crs.angular_velocity = 2
        movables = [crs]
        s = Simulator(cfg, movables, [])
        s.step()
        s_export = s.export()
        angle_two_step = crs.orientation
        s.step()
        s.step()

        crs2 = Cross([400, 400], 50, 10, KINEMATIC)
        s2 = Simulator(cfg, [crs2], [])
        with pytest.raises(ValueError):
            s2.import_from(s_export)

    def test_apply_forces(self):
        pass