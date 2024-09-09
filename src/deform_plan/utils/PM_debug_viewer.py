import pymunk
import pygame
from pymunk.pygame_util import DrawOptions

from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.controllers.PM_cable_controller import PMCableController

class DebugViewer:
    def __init__(self, simulator: Simulator,
                 render_constraints=False,
                 realtime=False):
        self.simulator = simulator
        self.simulator.debugger = self
        w,h,self.FPS,_ = self.simulator.get_debug_data()
        self.display = pygame.display.set_mode((w, h))
        self.cur_scene = pygame.surface.Surface((w,h))
        self.draw_options = DrawOptions(self.cur_scene)
        self.realtime = realtime
        self.clock = pygame.time.Clock()
        self.want_running = True
        self.controller = None # type: PMCableController
        if not render_constraints:
            self.draw_options.flags = DrawOptions.DRAW_SHAPES



    def update_cur(self, space: pymunk.Space):
        # new rendered image, will be prepared to be rendered
        self.cur_scene.fill((255, 255, 255))
        space.debug_draw(self.draw_options)
        self.display.blit(self.cur_scene, (0, 0))
        pygame.display.update()
        if self.realtime:
            self.clock.tick(self.FPS)
        if self.controller is not None:
            self.controller.update()
        else:
            self._mark_end_custom()




    def want_end(self):
        return not self.want_running

    def _mark_end_custom(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.want_running = False

    def draw_line(self, start, end, color=(0, 0, 0)):
        pygame.draw.line(self.display, color, start, end)

    def draw_circle(self, pos, radius, color=(0, 0, 0)):
        pygame.draw.circle(self.display, color, pos, radius)
