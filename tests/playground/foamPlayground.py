from tests.TestTemplate import TestTemplate
import pymunk
import pymunk.constraints
from src.helpers.foams import Foam


class FoamPlayground(TestTemplate):

    def __init__(self):
        super().__init__(800, 800,80)
    def setup(self):
        self.draw_constraints = False
        self.space.gravity = 0, 98.1
        self.space.damping = .3
        #foam
        foamParamsStruct = Foam.SpringParams(2000, 2000)
        foamParamsBend = Foam.SpringParams(300, 50)
        self.foam = Foam(100, 200, 400, 400, .06, foamParamsStruct, foamParamsBend)
        self.foam.add(self.space)
        self.foam.segments[0][0].body_type = pymunk.Body.STATIC
        self.foam.segments[0][-1].body_type = pymunk.Body.STATIC

        #ball
        ball = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
        ball.position = 600, 380
        ball.velocity = -500,0
        ball_shape = pymunk.Circle(ball, 50)
        ball_shape.density = .2
        ball_shape.color = (255, 0, 0, 255)
        self.space.add(ball, ball_shape)

    def pre_render(self):
        pass

    def post_render(self):
        pass


if __name__ == "__main__":
    scene = FoamPlayground()
    scene.run()
