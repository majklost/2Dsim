import dill

from pymunk.pygame_util import DrawOptions
import pygame

from ..saveables.replayable_path import ReplayablePath
from .base_viewer import BaseViewer

class PMReplayableViewer(BaseViewer):
    def __init__(self, replay_file):
        replay_data = dill.load(open(replay_file, 'rb')) #type: ReplayablePath
        self.guider = replay_data.guider
        self.goal = replay_data.goal
        self.path = replay_data.path
        self.additional_data = replay_data.additional_data
        self.sim = replay_data.simulator
        self.drawings = []
        pygame.init()
        w,h,self.fps = self.sim.get_debug_data()
        self.display = pygame.display.set_mode((w, h))
        self.cur_scene = pygame.surface.Surface((w,h))

    def draw_line(self, start, end, color=(0, 0, 0)):
        pygame.draw.line(self.display, color, start, end)


    def show(self,realtime=True):
        self.sim.import_from(self.path[0].sim_export)
        clock = pygame.time.Clock()
        draw_ops = DrawOptions(self.cur_scene)
        # for n in range(len(self.path)):
        #     iter_cnt = self.path[n].replayer.segment_iter_cnt
        #     for i in range(iter_cnt):
        #         self.guider(self.sim,self.path[n-1],)
        #         self.sim.step()
        for n in self.path[1:]:
            parent = n.replayer.parent
            parent_guider_data = parent.guider_data
            print("Iterating now: ", n.replayer.segment_iter_cnt)


            iters = 0
            while iters < n.replayer.segment_iter_cnt:
                iters += 1
                self.guider(self.sim, parent, n.replayer.real_goal, parent_guider_data, iters)
                self.sim.step()
                self.cur_scene.fill((255, 255, 255))
                self.sim.draw_on(draw_ops)
                for d in self.drawings:
                    d.draw(self.display)
                self.display.blit(self.cur_scene, (0, 0))
                if realtime:
                    clock.tick(self.fps)
                pygame.display.update()
                if self.want_end():
                    return


    @staticmethod
    def want_end():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        return False








