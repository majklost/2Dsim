import pygame
import pymunk
from  pymunk import pygame_util



class PygameRenderer:
    """
    Tool for debugging spaces
    Renders the space in debug mode
    """
    def __init__(self, width, height,FPS,eventQueue_clb=None, update_cur_clb=None):
        self.display = pygame.display.set_mode((width,height))
        self.FPS = FPS
        self.draw_constraints = False
        self.cur_scene = pygame.surface.Surface((width,height))
        self.draw_options = pymunk.pygame_util.DrawOptions(self.cur_scene)
        self.want_running = True
        self.update_cur_clb = update_cur_clb
        pygame.init()
        self.clock = pygame.time.Clock()
        if eventQueue_clb is None:
            self.eventQueue_clb = self._process_eventQueue
        else:
            self.eventQueue_clb = eventQueue_clb
        if not self.draw_constraints:
            self.draw_options.flags = pymunk.pygame_util.DrawOptions.DRAW_SHAPES


    def update_cur(self,space: pymunk.Space):
        self.want_running = True

        self.eventQueue_clb()
        # new rendered image, will be prepared to be rendered
        self.cur_scene.fill((255,255,255))
        space.debug_draw(self.draw_options)
        self.display.blit(self.cur_scene, (0, 0))
        if self.update_cur_clb is not None:
            self.update_cur_clb(self.display,space)
        pygame.display.update()
        # self.clock.tick(self.FPS)

    def _process_eventQueue(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.want_running = False
                print("want quit")


