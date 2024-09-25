from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *


def get_standard_simulator(cable, obstacle_group=None):
    """
    Create a simulator with a cable
    :param cable: Cable object
    :param obstacle_group: ObstacleGroup object
    :return: Simulator object
    """
    cfg = PMConfig()


    top = Rectangle([400, 0], 800, 20, STATIC)
    bottom = Rectangle([400, 800], 800, 20, STATIC)
    left = Rectangle([0, 400], 20, 800, STATIC)
    right = Rectangle([800, 400], 20, 800, STATIC)

    fixed_objects = [top, bottom, left, right]
    if obstacle_group is not None:
        fixed_objects.append(obstacle_group)
    for f in fixed_objects:
        f.track_colisions = False

    sim = Simulator(cfg, [cable], fixed_objects)
    return sim

