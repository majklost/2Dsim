import pygame
import pymunk
from pymunk import pygame_util
#boilerplate for simple pygame/pymunk tests
class TestTemplate:
    def __init__(self, width, height,FPS):
        self.clock = pygame.time.Clock()
        self.space = pymunk.Space()
        pygame.init()
        self.display = pygame.display.set_mode((width, height))
        self.FPS = FPS


    def setup(self):
        print("No setup defined")
        pass

    def pre_render(self):
        # print("No prerender defined")
        pass

    def post_render(self):
        # print("No postrender defined")
        pass

    def run(self):
        self.setup()
        draw_options = pymunk.pygame_util.DrawOptions(self.display)
        running = True
        while running:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.display.fill((255, 255, 255))
            #things before rendering physics
            self.pre_render()
            self.space.debug_draw(draw_options)
            self.space.step(1 / self.FPS)
            #things after rendering
            self.post_render()
            pygame.display.update()
            self.clock.tick(self.FPS)

        pygame.quit()












