# simple scene for checking if local planner can connect two points by line
# and if it can find that it reached obstacle
# also tries to render the paths and nodes
import pygame
import pymunk
import pymunk.pygame_util


from staticLocalPlanner import LocalPlanner


class Agent:
    def __init__(self, x, y):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.KINEMATIC)
        self.body.position = x, y

        self.shape = pymunk.Poly.create_box(self.body, (50, 50))
        self.shape.color = pygame.Color("blue")

    def add(self, space):
        space.add(self.body, self.shape)


class Obstacle:
    def __init__(self, x, y, w, h):
        self.body = pymunk.Body(1, 100, body_type=pymunk.Body.STATIC)
        self.body.position = x, y
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.color = pygame.Color("red")



    def add(self, space):
        space.add(self.body, self.shape)


def render_goal(display, goal):
    pygame.draw.circle(display, (0, 255, 0), goal, 10)

def game():
    GOAL1 = (50, 700) #the blocked one
    GOAL2 = (700, 50) # free one
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
    agent = Agent(50, 90)
    agent.add(space)

    obstacle = Obstacle(200, 130, 400, 20)
    # obstacle.body.angle = 0.5
    obstacle.add(space)

    # local planner
    # lp = LocalPlanner(space, agent.shape, Agent(GOAL2[0], GOAL2[1]).shape)
    lp = LocalPlanner(space, agent.shape, Agent(GOAL1[0], GOAL1[1]).shape)
    lp.check_path()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        display.fill((255, 255, 255))
        space.debug_draw(draw_options)
        render_goal(display, GOAL1)
        render_goal(display, GOAL2)
        pygame.display.update()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    game()
