import dill
import pygame
from pymunk.pygame_util import DrawOptions

from deform_plan.saveables.replayable_path import ReplayablePath


class PMReplayableCore:
    def __init__(self, replay_file):
        replay_data = dill.load(open(replay_file, 'rb'))  # type: ReplayablePath
        self.guider = replay_data.guider
        self.guider_period = replay_data.guider_period
        self.reached_condition = replay_data.reached_condition
        self.goal = replay_data.goal
        self.path = replay_data.path
        self.additional_data = replay_data.additional_data
        self.sim = replay_data.simulator
        self.drawings = []
        self.w, self.h, self.fps = self.sim.get_debug_data()
        self.cur_scene = pygame.surface.Surface((self.w, self.h))
        self.one_time_canvas = pygame.surface.Surface((self.w, self.h), pygame.SRCALPHA)
        self.one_time_canvas.fill((255, 255, 255, 0))
        self.drawing_fnc = self.additional_data.get("drawing_fnc", None)
        self.one_time_draw = self.additional_data.get("one_time_draw", None)
        if self.drawing_fnc is None:
            print("No drawing function provided")

    def draw_line(self, start, end, color=(0, 0, 0)):
        self.drawings.append( lambda :pygame.draw.line(self.cur_scene, color, start, end))
    def draw_circle(self, center, radius, color=(0, 0, 0)):
        self.drawings.append( lambda :pygame.draw.circle(self.cur_scene, color, center, radius))

    def _onestep(self, parent, n,i,draw_ops:DrawOptions):
        parent_guider_data = parent.guider_data
        if self.reached_condition(self.sim, parent, n.replayer.real_goal, parent_guider_data, i):
            return True
        if i % self.guider_period == 0:
            self.guider(self.sim, parent, n.replayer.real_goal, parent_guider_data, i)
        self.sim.step()
        self.cur_scene.fill((255, 255, 255))
        self.cur_scene.blit(self.one_time_canvas, (0, 0))
        self.sim.draw_on(draw_ops)
        if self.drawing_fnc is not None:
            self.drawing_fnc(self.sim, self.cur_scene, self.additional_data)
        for d in self.drawings:
            d()
        return False
