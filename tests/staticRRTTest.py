# Try RRT on some easy environment
import pygame
import pymunk
import pymunk.pygame_util
from staticLocalPlanner import LocalPlanner
from RRT import RRT
from RRTNode import RRTNode
from tree_rendering import TreeRenderer


class Agent:
    def __init__(self, x, y):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.KINEMATIC)
        self.body.position = x, y

        self.shape = pymunk.Poly.create_box(self.body, (50, 50))
        self.shape.color = pygame.Color("blue")

    def add(self, space):
        space.add(self.body, self.shape)


class Block:
    def __init__(self, x=450, y=400, w=700, h=100):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.STATIC)
        self.body.position = x, y
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("red")

    def add(self, space):
        space.add(self.body, self.shape)


def render_goal(display, goal):
    pygame.draw.circle(display, (0, 255, 0), (goal.x,goal.y), 10)


def game():
    GOAL = RRTNode(400, 50,0)
    pygame.init()
    # basics
    display = pygame.display.set_mode((800, 800))
    clock = pygame.time.Clock()
    FPS = 80
    running = True

    # physics
    space = pymunk.Space()
    draw_options = pymunk.pygame_util.DrawOptions(display)

    # objects
    agent = Agent(50, 750)
    agent.add(space)


    block = Block()
    block.add(space)

    block2 = Block(350, 600)
    block2.add(space)

    # totalBlock = Block(400,200,800,20)
    # totalBlock.add(space)


    lp = LocalPlanner(space, agent.shape)
    rrt = RRT(800,800,0, lp)
    start = LocalPlanner.node_from_shape(agent.shape)
    path = rrt.find_path(start, GOAL)
    verts = rrt.get_verts()
    print(len(verts))

    tree_renderer = TreeRenderer(verts)



    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                quit()

        display.fill((255, 255, 255))
        tree_renderer.render(display,pygame.time.get_ticks())
        tree_renderer.render_path(display)
        space.debug_draw(draw_options)
        render_goal(display, GOAL)
        space.step(1 / FPS)


        pygame.display.update()
        clock.tick(FPS)


if __name__ == '__main__':
    game()
