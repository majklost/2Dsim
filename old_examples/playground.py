import pygame
import pymunk
import pymunk.pygame_util
import math



class Rotator():
    def __init__(self, x=400, y=400, r=50, btype=pymunk.Body.DYNAMIC):
        self.body = pymunk.Body(1, 100, body_type=btype)
        self.body.angular_velocity = 5
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, r)

    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape)


class Block():
    def __init__(self, x=450, y=400, w=700, h=100):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.STATIC)
        self.body.position = x, y
        self.shape = pymunk.Poly.create_box(self.body, (w, h))

    def add(self, space):
        space.add(self.body, self.shape)


class Cross():
    def __init__(self):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.KINEMATIC)
        self.body.position = 50, 400
        self.body.angular_velocity = 0
        self.body.angle = math.pi / 5
        self.shape1 = pymunk.Segment(self.body, (-50, 0), (50, 0), 10)
        self.shape2 = pymunk.Segment(self.body, (0, -50), (0, 50), 10)

    def add(self, space):
        space.add(self.body, self.shape1, self.shape2)


def game():
    pygame.init()
    # basics
    display = pygame.display.set_mode((800, 800))
    clock = pygame.time.Clock()
    FPS = 80
    running = True

    # physics
    space = pymunk.Space()
    space.gravity = 0.0, 98.1
    draw_options = pymunk.pygame_util.DrawOptions(display)

    # objects
    rotator = Rotator(y=0)
    rotator.add(space)

    # cross = Cross()
    # cross.add(space)

    block = Block()
    block.add(space)

    block2 = Block(350, 600)
    block2.add(space)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                quit()

        display.fill((255, 255, 255))
        space.debug_draw(draw_options)
        space.step(1 / FPS)

        pygame.display.update()
        clock.tick(FPS)


if __name__ == '__main__':
    game()
