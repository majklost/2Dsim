from z_old_tests.TestTemplate import TestTemplate
import pymunk.constraints
import pymunk
from _old_src.helpers.objectLibrary import AbstractObj
import pygame

class SpringedBall(AbstractObj):
    def __init__(self, x, y, r, mass=1, moment=100):
        super().__init__(x, y, pymunk.Body.DYNAMIC, mass, moment)
        self.shape = pymunk.Circle(self.body, r)
        self.shape.color = pygame.Color("yellow")
        self.shape.collision_type = 1

    def add(self, space: pymunk.Space):
        super().add(space)
        # space.add(pymunk.constraints.DampedRSpring(self.body, space.static_body, (0, 0), self.body.position-(50,0), 100, 200, 0.5))
        space.add(pymunk.constraints.DampedRotarySpring(self.body,space.static_body, 0, 1000, 0))
class SpringedCover:
    def __init__(self,x,y,r, numBalls=20, ballSep=5):
        """
        x,y is the most left point of the cover
        :param x:
        :param y:
        :param r:
        """
        self.balls = []
        self.ball_shapes = []
        self.springs = []
        for i in range(numBalls):
            ball = pymunk.Body(1, 100, body_type=pymunk.Body.DYNAMIC)
            ball.position = x + i*ballSep, y

            ball_shape = pymunk.Circle(ball, r)
            ball_shape.friction = 5
            ball_shape.color = pygame.Color("green")
            self.balls.append(ball)
            self.ball_shapes.append(ball_shape)
        for i in range(numBalls-1):
            spring = pymunk.constraints.DampedSpring(self.balls[i], self.balls[i+1], (0,0), (0, 0), 1, 200, 5)
            self.springs.append(spring)

    def add(self, space: pymunk.Space):
        space.add(*self.balls)
        space.add(*self.springs)
        space.add(*self.ball_shapes)

class MultiLayerCover:
    def __init__(self,x,y,r,numBalls=200, ballSep=1, numOfLayers=2):
        self.layers = []
        for i in range(numOfLayers):
            c = SpringedCover(x-i*ballSep,y-i*ballSep,r,numBalls,ballSep)
            self.layers.append(c)

        self.multiLayerSprings = []
        for li in range(numOfLayers-1):
            for i in range(numBalls):
                spring = pymunk.constraints.DampedSpring(self.layers[li].balls[i],self.layers[li+1].balls[i], (0,0), (0,0),1,200,5)
                self.multiLayerSprings.append(spring)

    def add(self, space: pymunk.Space):
        for l in self.layers:
            l.add(space)
        space.add(*self.multiLayerSprings)





class FallingBall(AbstractObj):
    def __init__(self, x, y, r, mass=10, moment=100):
        super().__init__(x, y, pymunk.Body.DYNAMIC, mass, moment)
        self.shape = pymunk.Circle(self.body, r)
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = 1
        self.shape.friction = 50
    def add(self, space: pymunk.Space):
        super().add(space)

class Scene(TestTemplate):
    def __init__(self, width, height, FPS):
        super().__init__(width, height, FPS)


    def setup(self):
        # self.space.gravity = 0,  98.1
        # self.agent = MultiLayerCover(100, 100, 2,numOfLayers=3,numBalls=20)
        # self.agent.add(self.space)
        # # self.agent = SpringedBall(120, 100, 20)
        # # self.agent.add(self.space)
        # self.obstacle = FallingBall(150, 400, 50)
        # self.obstacle.set_body_type(pymunk.Body.STATIC)
        # self.obstacle.add(self.space)
        self.agent = SpringedBall(100, 100, 20)
        self.agent.add(self.space)
        self.agent.body.angle = 0.5

    # def post_render(self):
        # print(self.clock.get_fps())

if __name__ == "__main__":
    scene = Scene(800, 600, 80)
    scene.run()




