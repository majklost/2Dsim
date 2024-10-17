import cProfile

import numpy as np
import time
import pymunk

from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import show_sim,make_draw_line,make_draw_circle
from deform_plan.utils.analytics import print_analytics


from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
import deform_plan.rrt_utils.cable_rrt as vutils

from examples.helpers.cable_map import get_standard_simulator



CABLE_LENGTH = 400
SEGMENT_NUM = 70
MAX_FORCE =1200
cfg = PMConfig()
cable= Cable([100,50],400,SEGMENT_NUM,thickness=5)
obstacle_g = RandomObstacleGroup(np.array([100,200]),150,400,3,3,seed=20)
sim = get_standard_simulator(cable,obstacle_g)


def profile_space(s: pymunk.Space):
    for i in range(100):
        s.step(1/60)
    for i in range(100):
        sc = s.copy()


cProfile.run("profile_space(_sim._space)",sort="cumtime")