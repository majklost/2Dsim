import pymunk
import pygame
from typing import List

from pymunk import vec2d

# movement and switching objects to control

MOVING_VEL = 3000

class KeyControls:
    def __init__(self, space: pymunk.Space, objects: List[pymunk.Body]):
        self.space = space
        self.objects = objects
        self.current = 0
        self.objects[self.current].color = pygame.Color("red")



    def solve_keys(self, keys, keydowns):
        for k in keydowns:
            if k.key == pygame.K_TAB and k.mod & pygame.KMOD_SHIFT:
                self.current = (self.current - 1) % len(self.objects)
                print("change back")
            elif k.key == pygame.K_TAB:
                self.current = (self.current + 1) % len(self.objects)
                print("change forward")

        cur_vel = [0, 0]
        if keys[pygame.K_LEFT]:
            cur_vel[0] = -MOVING_VEL
        if keys[pygame.K_RIGHT]:
            cur_vel[0] = MOVING_VEL
        if keys[pygame.K_UP]:
            cur_vel[1] = -MOVING_VEL
        if keys[pygame.K_DOWN]:
            cur_vel[1] = MOVING_VEL
        c=vec2d.Vec2d(cur_vel[0], cur_vel[1])
        goodC = c.rotated(-self.objects[self.current].angle)
        self.objects[self.current].apply_force_at_local_point(goodC*20, (0, 0))
        # self.objects[self.current].velocity = cur_vel








