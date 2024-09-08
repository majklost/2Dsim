import numpy as np
import pytest

from deform_plan.assets.PM import *
class TestCross:
    def test_cross(self):
        crs = Cross([400,400],50,10,KINEMATIC)
        crs.angular_velocity = 2
        crs.orientation = 3.14
        b= crs.position == [400,400]
        c = np.array(crs.velocity) == np.array([0,0])
        assert b.all()
        assert crs.angular_velocity == 2
        assert crs.orientation == 3.14
        assert c.all()

