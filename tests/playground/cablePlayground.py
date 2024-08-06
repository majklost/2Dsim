import pygame.time

from tests.TestTemplate import TestTemplate
import pymunk
import pymunk.constraints
from src.helpers.cables import Cable,MultibodyCable,HardJointCable


class CablePlayground(TestTemplate):
    def setup(self):
        self.draw_constraints = False
        bendingParams = Cable.SpringParams(20, 100)
        structuralParams = Cable.SpringParams(400, 100)

        #cable
        self.cable = Cable(100, 100, 400, 50, structuralParams, bendingParams)
        # self.cable.add(self.space)
        self.space.gravity = 0, 98.1
        self.moving_points= []
        # connect sides of the cable to the world
        self.cable.masses[0].body_type = pymunk.Body.STATIC
        self.cable.masses[-1].body_type = pymunk.Body.KINEMATIC


        # multibody cable
        multibodParams = Cable.SpringParams(2000, 1000)
        self.multibodyCable = MultibodyCable(20, 300, 400, 24, multibodParams, multibodParams)
        self.multibodyCable.add(self.space)
        self.multibodyCable.segments[0].body_type = pymunk.Body.STATIC
        self.multibodyCable.segments[-1].body_type = pymunk.Body.KINEMATIC

        #hardJoint cable
        hardJointParams = Cable.SpringParams(200, 100)
        self.hardJointCable = HardJointCable(20, 300, 400, 24, hardJointParams)
        # self.hardJointCable.add(self.space)
        self.hardJointCable.segments[0].body_type = pymunk.Body.STATIC
        self.hardJointCable.segments[-1].body_type = pymunk.Body.KINEMATIC


        ball = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
        ball.position = 150, 180
        ball.velocity = 0, 20


        self.moving_points.append(self.hardJointCable.segments[-1])
        self.moving_points.append(self.multibodyCable.segments[-1])

        # blockage = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        # blockage.position = 250, 250
        # blockage_shape = pymunk.Poly.create_box(blockage, (100, 100))
        # blockage_shape.color = (0, 0, 0, 255)
        # self.space.add(blockage, blockage_shape)
        #
        # motor = pymunk.constraints.SimpleMotor(ball, , )
        ball_shape = pymunk.Circle(ball, 20)
        ball_shape.density = .002
        ball_shape.color = (255, 0, 0, 255)
        ball_shape.friction = 0.5
        ball_shape.collision_type = 1
        self.space.add(ball, ball_shape)
        # for p in self.moving_points:
        #     p.velocity = 30,0

    def pre_render(self):
        t = pygame.time.get_ticks()
        if t > 40000:
            for p in self.moving_points:
                p.velocity = 0, 0
            return
        # force_mags = [s.impulse*self.FPS for s in self.multibodyCable.linearSprings]
        # print("push force: ", force_mags[-1])

        for p in self.moving_points:
            if t % 10000 < 2500:
                p.velocity = 50, 0
            elif t % 10000 < 5000:
                p.velocity = 0,50
            elif t % 10000 < 7500:
                p.velocity = -50, 0
            else:
                p.velocity = 0, -50




if __name__ == '__main__':
    c = CablePlayground(800, 600, 60)
    c.run()
