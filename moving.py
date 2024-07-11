# Move one object around playground
# There are two other objects (static, kinematic)
# used for checking collisions - get to know with presolve, post solve, and separate

import pygame
import pymunk
from pymunk import pygame_util

VEL = 600
PLAYER_COLLISION_TYPE = 1
STATIC_COLLISION_TYPE = 2


class Player():
    def __init__(self):
        self.body = pymunk.Body(1, 100)
        self.body.body_type = pymunk.Body.KINEMATIC
        self.body.position = 0, 0
        self.shape = pymunk.Poly.create_box(self.body, (50, 50))
        self.acc = (0, 0)
        self.shape.collision_type = PLAYER_COLLISION_TYPE

    def add(self, space):
        space.add(self.body, self.shape)

    def move(self, keys):
        cur_vel = (0, 0)
        if keys[pygame.K_LEFT]:
            cur_vel = (-VEL, 0)
        if keys[pygame.K_RIGHT]:
            cur_vel = (VEL, 0)
        if keys[pygame.K_UP]:
            cur_vel = (0, -VEL)
        if keys[pygame.K_DOWN]:
            cur_vel = (0, VEL)

        # self.body.velocity += self.acc
        self.body.velocity = cur_vel


class StaticO():
    def __init__(self):
        self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.body.position = 400, 400
        self.shape = pymunk.Poly.create_box(self.body, (100, 50))
        self.shape.color = pygame.Color("red")
        self.shape.collision_type = STATIC_COLLISION_TYPE

    def add(self, space):
        space.add(self.body, self.shape)


class KinematicO():
    def __init__(self):
        self.body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.body.position = 400, 200
        self.body.velocity = 10, 0
        self.shape = pymunk.Poly.create_box(self.body, (100, 50))
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = STATIC_COLLISION_TYPE

    def add(self, space):
        space.add(self.body, self.shape)


def test_col_fnc(arbiter, space, data):
    print("pre solve")
    return False


def game():
    pygame.init()
    # basics
    display = pygame.display.set_mode((800, 800))
    clock = pygame.time.Clock()
    FPS = 80
    running = True

    # physics
    space = pymunk.Space()
    space.iterations = 60
    draw_options = pymunk.pygame_util.DrawOptions(display)
    handler = space.add_collision_handler(PLAYER_COLLISION_TYPE, STATIC_COLLISION_TYPE)
    handler.begin = test_col_fnc  #calls when enters object
    # handler.pre_solve = test_col_fnc #calls during collision
    # handler.separate = test_col_fnc #calls when leaves object
    # handler.post_solve = test_col_fnc #calls after collision - only for solving collision
    # objects - player, staticO, kinematicO
    player = Player()
    player.add(space)
    static = StaticO()
    static.add(space)
    kinematic = KinematicO()
    kinematic.add(space)

    #loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                quit()

        keys = pygame.key.get_pressed()
        player.move(keys)

        display.fill((255, 255, 255))
        space.debug_draw(draw_options)
        space.step(1 / FPS)

        pygame.display.update()
        clock.tick(FPS)


if __name__ == '__main__':
    game()
