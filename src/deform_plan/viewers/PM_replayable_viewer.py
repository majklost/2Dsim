import dill

from pymunk.pygame_util import DrawOptions
import pygame

from ..saveables.replayable_path import ReplayablePath
from .base_viewer import BaseViewer
from .PM_replayable_core import PMReplayableCore

class PMReplayableViewer(BaseViewer,PMReplayableCore):
    def __init__(self, replay_file,allow_outer_render=True):
        PMReplayableCore.__init__(self, replay_file)

        self.display = pygame.display.set_mode((self.w, self.h))

    def show(self,realtime=True,constraints=False):
        # for k,v in self.additional_data.items():
        #     print(f"{k} : {v}")
        self.sim.import_from(self.path[0].sim_export)
        if self.one_time_draw is not None:
            self.one_time_draw(self.sim, self.one_time_canvas, self.additional_data)

        clock = pygame.time.Clock()
        draw_ops = DrawOptions(self.cur_scene)
        if not constraints:
            draw_ops.flags = DrawOptions.DRAW_SHAPES

        for n in self.path[1:]:
            parent = n.replayer.parent
            print("Iterating now: ", n.replayer.segment_iter_cnt)
            for i in range(n.replayer.segment_iter_cnt):
                if self._onestep(parent, n, i,draw_ops):
                    break
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









