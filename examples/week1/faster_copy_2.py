import pymunk
import numpy as np
s1 = pymunk.Space()
s2 = pymunk.Space()
s1.gravity = 0,98.1
s2.gravity = 0,98.1

b1= pymunk.Body(body_type=pymunk.Body.DYNAMIC)
b1.position = 200,20
b1.velocity = 10,12

b2 = pymunk.Body(body_type=pymunk.Body.DYNAMIC)


shape1 = pymunk.Circle(b1,5)
shape1.density = .1
s1.add(b1,shape1)


def check_space(s1:pymunk.Space,s2: pymunk.Space):
    for bi in range(len(s1.bodies)):
        assert np.allclose(s1.bodies[bi].position, s2.bodies[bi].position), (s1.bodies[bi].position, s2.bodies[bi].position)
        assert np.allclose(s1.bodies[bi].velocity, s2.bodies[bi].velocity), (s1.bodies[bi].velocity, s2.bodies[bi].velocity)
        assert np.allclose(s1.bodies[bi].angular_velocity, s2.bodies[bi].angular_velocity), (s1.bodies[bi].angular_velocity, s2.bodies[bi].angular_velocity)
        assert np.allclose(s1.bodies[bi].angle, s2.bodies[bi].angle), (s1.bodies[bi].angle, s2.bodies[bi].angle)


for i in range(1000):
    s1.step(1/80)
s2 = s1.copy()

for i in range(1000):
    s1.step(1/80)
    s2.step(1/80)
check_space(s1,s2)