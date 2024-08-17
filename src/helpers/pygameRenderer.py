import pygame
import pymunk
from  pymunk import pygame_util



class pygameRenderer:
    """
    Renders the space in debug mode
    """
    def __init__(self, width, height,FPS):
        self.display = pygame.display.set_mode((width,height))

        self.FPS = FPS
        self.draw_constraints = False
        self.cur_scene = pygame.surface.Surface((width,height))
        self.draw_options = pymunk.pygame_util.DrawOptions(self.cur_scene)
        self.want_running = True
        pygame.init()
        self.clock = pygame.time.Clock()
        if not self.draw_constraints:
            self.draw_options.flags = pymunk.pygame_util.DrawOptions.DRAW_SHAPES


    def update_cur(self,space: pymunk.Space):
        self.want_running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.want_running = False
                print("want quit")
        # new rendered image, will be prepared to be rendered
        self.cur_scene.fill((255,255,255))
        space.debug_draw(self.draw_options)
        self.display.blit(self.cur_scene, (0, 0))
        pygame.display.update()
        self.clock.tick(self.FPS)


