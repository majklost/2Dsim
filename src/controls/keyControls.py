import pymunk
import pygame
from typing import List


from pymunk import vec2d
from pymunk.pygame_util import from_pygame


# movement and switching objects to control
# selecting object can be done by 2 things:
# 1. tab move forward, shift+tab move backward
# 2. click on the object


# MOVING_FORCE = 3000

class KeyControls:
    def __init__(self, space: pymunk.Space, objects: List[pymunk.Body], moving_force,screen):
        self.space = space
        self.objects = objects
        self.current = 0
        self.objects[self.current].color = pygame.Color("yellow")
        self.moving_force = moving_force
        self.screen = screen

    def change_color(self,color):
        o = self.objects[self.current]
        for s in o.shapes:
            s.color = color

    def solve_keys(self, keys, keydowns,click):
        self.change_color(pygame.Color("blue"))
        for k in keydowns:
            if k.key == pygame.K_TAB and k.mod & pygame.KMOD_SHIFT:
                self.current = (self.current - 1) % len(self.objects)
                print("change back")
            elif k.key == pygame.K_TAB:
                self.current = (self.current + 1) % len(self.objects)
                print("change forward")
        if click is not None:
            p = from_pygame(click.pos, self.screen)
            for i, obj in enumerate(self.objects):
                for s in obj.shapes:
                    dist = s.point_query(p).distance
                    if dist < 0:
                        self.current = i
                        print("change to", i)
                        break


        cur_force = [0, 0]
        if keys[pygame.K_LEFT]:
            cur_force[0] = -self.moving_force
        if keys[pygame.K_RIGHT]:
            cur_force[0] = self.moving_force
        if keys[pygame.K_UP]:
            cur_force[1] = -self.moving_force
        if keys[pygame.K_DOWN]:
            cur_force[1] = self.moving_force
        c=vec2d.Vec2d(cur_force[0], cur_force[1])
        goodC = c.rotated(-self.objects[self.current].angle)
        self.objects[self.current].apply_force_at_local_point(goodC, (0, 0))
        # self.objects[self.current].velocity = cur_force
        self.change_color(pygame.Color("yellow"))








