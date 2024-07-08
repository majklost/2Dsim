import pygame
import pymunk

pygame.init()

display = pygame.display.set_mode((800, 800))
clock = pygame.time.Clock()
space = pymunk.Space()
FPS = 80

body = pymunk.Body()
body.position = 400, 400
shape = pymunk.Circle(body, 10)
space.add(body, shape)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    space.step(1/FPS)
    display.fill((255, 255, 255))
    pygame.draw.circle(display, (0, 0, 0), (int(body.position.x), int(body.position.y)), 10)
    pygame.display.update()
    clock.tick(FPS)