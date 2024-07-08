import sys

import pygame
from pygame.locals import *

pygame.init()
vec = pygame.math.Vector2

HEIGHT = 800
WIDTH = 600
ACC = 0.5
FRIC = -0.12
FPS = 60

FramePerSec = pygame.time.Clock()
displaysurface = pygame.display.set_mode((WIDTH, HEIGHT),RESIZABLE)
pygame.display.set_caption("Game")


class Player(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.surf = pygame.Surface((60, 60))
        self.surf.fill((128, 255, 40))
        self.rect = self.surf.get_rect()

        self.pos = vec((10, 385))
        self.vel = vec(0, 0)
        self.acc = vec(0, 0)

    def move(self):
        self.acc = vec(0, 0)

        pressed_keys = pygame.key.get_pressed()

        if pressed_keys[K_LEFT]:
            self.acc.x = -ACC
        if pressed_keys[K_RIGHT]:
            self.acc.x = ACC
        if pressed_keys[K_UP]:
            self.acc.y = -ACC
        if pressed_keys[K_DOWN]:
            self.acc.y = ACC

        self.acc.x += self.vel.x * FRIC
        self.acc.y += self.vel.y * FRIC
        self.vel += self.acc
        self.pos += self.vel + 0.5 * self.acc
        if self.pos.x > WIDTH:
            self.pos.x = WIDTH
        if self.pos.x < 0:
            self.pos.x = 0
        if self.pos.y > HEIGHT:
            self.pos.y = HEIGHT
        if self.pos.y < self.rect.height:
            self.pos.y = self.rect.height

        self.rect.midbottom = self.pos


class platform(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.surf = pygame.Surface((WIDTH, 20))
        self.surf.fill((255, 0, 0))
        self.rect = self.surf.get_rect(center=(WIDTH / 2, HEIGHT - 10))


PT1 = platform()
P1 = Player()

all_sprites = pygame.sprite.Group()
# all_sprites.add(PT1)
# all_sprites.add(P1)
PT1.add(all_sprites)
P1.add(all_sprites)

print(pygame.display.get_driver())
while True:
    pressed_keys = pygame.key.get_pressed()
    if pressed_keys[K_q] and pressed_keys[K_LCTRL]:
        pygame.quit()
        sys.exit()

    #read input
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    #prepare buffer
    displaysurface.fill((0, 0, 0))
    #apply inputs
    P1.move()
    #draw to buffer
    for entity in all_sprites:
        displaysurface.blit(entity.surf, entity.rect)
    #display buffer
    pygame.display.update()
    #calc how many milliseconds passed + delay the game to run at FPS defined
    FramePerSec.tick(FPS)
