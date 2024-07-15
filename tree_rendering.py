import pygame
from RRTNode import RRTNode
from typing import List
class TreeRenderer:
    def __init__(self,vertices:List[RRTNode],tick_time=100):
        self.vertices = vertices
        self.render_cnt = 0
        self.lines = []
        self.last_added_time = 0
        self.tick_time = tick_time

    def render(self,display,cur_time):
        self.showMore(cur_time)
        for line in self.lines:
            pygame.draw.line(display, (0, 0, 0), line[0], line[1], 1)

    #add 100 more lines to render
    def showMore(self, cur_time):
        if cur_time - self.last_added_time < self.tick_time:
            return

        for i in range(100):
            if self.render_cnt < len(self.vertices):
                parent = self.vertices[self.render_cnt].parent
                if parent is not None:
                    self.lines.append((self.vertices[self.render_cnt].get_pos(), parent.get_pos()))
                self.render_cnt += 1
            else:
                break
        self.last_added_time = cur_time

    def render_path(self,display):
        goal = self.vertices[-1]
        while goal.parent is not None:
            # pygame.draw.circle(display, (255, 0, 0), goal.get_pos(), 10)
            pygame.draw.line(display, (255, 0, 0), goal.get_pos(), goal.parent.get_pos(), 5)
            goal = goal.parent
