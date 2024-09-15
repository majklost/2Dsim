"""Make one frame from simulation - nice for seeing start position"""
import numpy as np
import pymunk
import pygame

from typing import Callable
from pymunk.pygame_util import DrawOptions
from ..simulators.PM.pm_simulator import Simulator

def show_sim(sim:Simulator, show_constraints=False, clb: Callable[[pygame.Surface],None]=None):
    pygame.init()
    w,h,fps = sim.get_debug_data()
    screen = pygame.display.set_mode((w,h))
    clock = pygame.time.Clock()
    screen.fill((255,255,255))
    draw_ops = DrawOptions(screen)
    if not show_constraints:
        draw_ops.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
    sim.draw_on(draw_ops)
    if clb is not None:
        clb(screen)
    pygame.display.update()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        clock.tick(fps)
        pygame.display.update()

def make_draw_line(p1:np.array,p2,thickness=2,color=pygame.Color("red")):
    def draw_line(surface):
        pygame.draw.line(surface,color,p1,p2,thickness)
    return draw_line

def make_draw_circle(pos,radius,color=pygame.Color("blue")):
    def draw_circle(surface):
        pygame.draw.circle(surface,color,pos,radius)
    return draw_circle

def make_draw_rect(pos:np.array,width,height,angle,color=pygame.Color("green")):
    def draw_rect(surface):
        rect = pygame.Rect(pos[0],pos[1],width,height)
        pygame.draw.rect(surface,color,rect)
    return draw_rect

