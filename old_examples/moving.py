# Move one object around playground
# There are two other objects (static, kinematic)
# used for checking collisions - get to know with presolve, post solve, and separate

import pygame
import pymunk
from pymunk import pygame_util
from pymunk import vec2d

VEL = 200.0
PLAYER_COLLISION_TYPE = 1
STATIC_COLLISION_TYPE = 2


class Player():
    def __init__(self):
        self.body = pymunk.Body(1, 10)
        self.body.body_type = pymunk.Body.DYNAMIC
        self.body.position = 0, 0
        self.body.colliding = False
        self.shape = pymunk.Poly.create_box(self.body, (50, 50))
        self.acc = (0, 0)
        self.shape.collision_type = PLAYER_COLLISION_TYPE

    def add(self, space):
        space.add(self.body, self.shape)

    def move(self, keys):
        if self.body.colliding:
            normal = self.body.to_push
            self.body.velocity = normal *10
            return
        # print("prev", self.body.velocity)
        cur_vel = [0, 0]
        if keys[pygame.K_LEFT]:
            cur_vel[0] = -VEL
        if keys[pygame.K_RIGHT]:
            cur_vel[0] = VEL
        if keys[pygame.K_UP]:
            cur_vel[1] = -VEL
        if keys[pygame.K_DOWN]:
            cur_vel[1] = VEL

        # print(type(VEL))
        c=vec2d.Vec2d(cur_vel[0], cur_vel[1])
        goodC = c.rotated(-self.body.angle)

        # self.body.apply_force_at_local_point(goodC, (0, 0))
        # self.body.velocity += self.acc
        self.body.velocity = cur_vel
        # self.body.position += cur_vel[0] * 1/80, cur_vel[1] * 1/80


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
    kinematic,obstacle = arbiter.shapes


    print("Collision detected")
    # kinematic.body.velocity = [0,0]
    kinematic.body.colliding = True
    kinematic.body.to_push = -arbiter.contact_point_set.normal

    return False

def sep(arbiter, space, data):
    kinematic, obstacle = arbiter.shapes
    print("Collision ended")
    return True



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
    handler.pre_solve = test_col_fnc  #calls when enters object
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
            if event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_TAB:
                    print("TAB")


        keys = pygame.key.get_pressed()
        player.move(keys)

        display.fill((255, 255, 255))
        space.debug_draw(draw_options)
        player.body.colliding = False
        space.step(1 / FPS)

        pygame.display.update()
        clock.tick(FPS)



if __name__ == '__main__':
    game()
