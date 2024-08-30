import pygame
import pymunk
import numpy as np

# functions with general usage across the project
def render_goal(display, goal):
    pygame.draw.circle(display, (0, 255, 0), (goal[0], goal[1]), 10)


def get_points_from_space(space:pymunk.Space, atrrib):
    """
    Identifies bodies with the given attribute
    :return:
    """
    attrib_bodies = []
    for b in space.bodies:
        if hasattr(b, atrrib):
            attrib_bodies.append(b)
    return sorted(attrib_bodies, key=lambda x: getattr(x, atrrib))


