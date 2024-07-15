import pygame
def render_goal(display, goal):
    pygame.draw.circle(display, (0, 255, 0), (goal.x,goal.y), 10)