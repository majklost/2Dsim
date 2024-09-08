import pygame
import pymunk
from pymunk import pygame_util


# boilerplate for simple pygame/pymunk z_old_tests
class TestTemplate:
    def __init__(self, width, height, FPS):
        self.clock = pygame.time.Clock()
        self.space = pymunk.Space()
        pygame.init()
        self.display = pygame.display.set_mode((width, height))
        self.FPS = FPS
        self.draw_constraints = True
        self.keys = pygame.key.get_pressed()
        self.keydowns = []
        self.click = None

    def setup(self):
        """
        This method should be overridden in the child class
        It is run once before the main loop
        :return:
        """
        print("No setup defined")
        pass

    def pre_render(self):
        """
        This method should be overridden in the child class
        It is run before rendering the physics
        :return:
        """
        # print("No prerender defined")
        pass

    def post_render(self):
        """
        This method should be overridden in the child class
        It is run after rendering the physics
        :return:
        """
        # print("No postrender defined")
        pass

    def run(self):
        """
        Main loop
        :return:
        """
        self.setup()
        draw_options = pymunk.pygame_util.DrawOptions(self.display)
        if not self.draw_constraints:
            draw_options.flags = pymunk.pygame_util.DrawOptions.DRAW_SHAPES
        pymunk.pygame_util.DrawOptions.DRAW_CONSTRAINTS = 0
        # draw_options.constraint_color = (0, 0, 0, 100)
        running = True
        pygame.time.get_ticks()
        while running:
            self.keydowns = []
            self.click = None
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    self.keydowns.append(event)
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.click = event



            self.keys = pygame.key.get_pressed()
            self.display.fill((255, 255, 255))
            # things before rendering physics
            self.pre_render()
            self.space.debug_draw(draw_options)
            self.space.step(1 / self.FPS)
            # things after rendering
            self.post_render()
            pygame.display.update()
            self.clock.tick(self.FPS)

        pygame.quit()

