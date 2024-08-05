import pymunk
import pygame
import pymunk.constraints


# prepared classes for the objects in the simulation
# IMPORTANT
# for agent and obstacle, each agent shape has collision type 1, each obstacle shape has collision type 2
# this is important for working with simulated dynamic planner

class AbstractObj:
    def __init__(self, x, y, body_type, mass=0, moment=0):
        self.body = pymunk.Body(mass, moment, body_type=body_type)
        self.body.position = x, y
        self.shapes = []

    def add(self, space: pymunk.Space):
        space.add(self.body)
        for shape in self.shapes:
            space.add(shape)

    @property
    def shape(self):
        return self.shapes[0]

    @shape.setter
    def shape(self, value):
        if self.shapes:
            self.shapes[0] = value
        else:
            self.shapes.append(value)

    def set_body_type(self, body_type):
        self.body.body_type = body_type


class Agent(AbstractObj):
    def __init__(self, x, y, w=10, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.mass = 1
        self.shape.color = pygame.Color("blue")
        self.shape.collision_type = 1


class Obstacle(AbstractObj):
    def __init__(self, x=450, y=400, w=700, h=100, angle=0):
        super().__init__(x, y, pymunk.Body.STATIC)
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (w, h))
        self.shape.mass = 1
        self.shape.color = pygame.Color("red")
        self.shape.collision_type = 2


# body is formed from two shapes
class Cross(AbstractObj):
    def __init__(self, x, y, w, av):
        super().__init__(x, y, pymunk.Body.KINEMATIC)
        self.body.angular_velocity = av
        self.shape1 = pymunk.Segment(self.body, (-w, 0), (w, 0), 10)
        self.shape1.mass = 1
        self.shape2 = pymunk.Segment(self.body, (0, -w), (0, w), 10)
        self.shape2.mass = 1
        self.shape1.collision_type = self.shape2.collision_type = 2

    def add(self, space: pymunk.Space):
        space.add(self.body, self.shape1, self.shape2)


class Cable:
    """
    A cable implemented from http://dx.doi.org/10.1051/matecconf/20153110002
    """


    def __init__(self, x, y, length, numMasses, structuralParams: 'SpringParams', bendingParams: 'SpringParams'):
        self.masses = []
        self.mass_shapes = []
        self.bending_springs = []
        self.structural_springs = []
        self.length = length
        self.structuralParams = structuralParams # type: Cable.SpringParams
        self.bendingParams = bendingParams
        sep = self.length / numMasses
        for m in range(numMasses):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + m * sep, y
            mass_shape = pymunk.Circle(mass, sep/2)
            # mass_shape = pymunk.Poly.create_box(mass, (sep, 5))
            mass_shape.density = .05/numMasses
            mass_shape.elasticity = 0.5
            mass_shape.friction = 0.5
            mass_shape.collision_type = 2
            self.masses.append(mass)
            self.mass_shapes.append(mass_shape)
        for i in range(numMasses - 1):
            spring = pymunk.constraints.DampedSpring(self.masses[i], self.masses[i + 1], (0, 0), (0, 0),sep, self.structuralParams.stiffness, self.structuralParams.damping)
            self.structural_springs.append(spring)
        for i in range(numMasses - 2):
            spring = pymunk.constraints.DampedSpring(self.masses[i], self.masses[i + 2], (0, 0), (0, 0), 2*sep, self.bendingParams.stiffness, self.bendingParams.damping)
            # spring.color = (255, 0, 0, 0)
            self.bending_springs.append(spring)

    def add(self, space: pymunk.Space):
        space.add(*self.masses)
        space.add(*self.structural_springs)
        space.add(*self.bending_springs)
        space.add(*self.mass_shapes)

    class SpringParams:
        def __init__(self, stiffness, damping):
            self.stiffness = stiffness
            self.damping = damping


class MultibodyCable:
    """
    A review of techniques for modeling flexible cables
    """
    def __init__(self,x,y,length,segmentNum,linearParams,angularParams, thickness=2):
        self.segments = []
        self.segments_shapes = []
        self.linearSprings = []
        self.angularSprings = []
        self.segmentNum = segmentNum
        self.linearParams = linearParams # type: Cable.SpringParams
        self.angularParams = angularParams # type: Cable.SpringParams


        segment_length = length/segmentNum
        for i in range(segmentNum):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + i * segment_length, y
            mass_shape = pymunk.Poly.create_box(mass, (segment_length, thickness))
            mass_shape.density = .005
            mass_shape.friction = 0.5
            mass_shape.collision_type = 2
            self.segments.append(mass)
            self.segments_shapes.append(mass_shape)
        for i in range(segmentNum-1):
            spring = pymunk.constraints.DampedSpring(self.segments[i], self.segments[i + 1], (0.3*segment_length, 0), (-0.3*segment_length, 0),0.1*segment_length, self.linearParams.stiffness/segment_length, self.linearParams.damping)

            # spring.max_force= 10
            # if i == 0:
                # spring.stiffness *= 10
            self.linearSprings.append(spring)
        for i in range(segmentNum - 1):
            spring = pymunk.constraints.DampedRotarySpring(self.segments[i], self.segments[i + 1],  0, 70, 10)
            self.angularSprings.append(spring)

    def add(self, space: pymunk.Space):
        space.add(*self.segments)
        space.add(*self.segments_shapes)
        space.add(*self.linearSprings)
        space.add(*self.angularSprings)
        # space.add(*self.centerSprings)
class HardJointCable:
    def __init__(self,x,y,length,segmentNum,linearParams,thickness=2):
        self.segments = []
        self.segments_shapes = []
        self.joints = []
        self.segmentNum = segmentNum
        self.linearSprings = []
        self.linearParams = linearParams
        segment_length = length/segmentNum
        for i in range(segmentNum):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + i * segment_length, y
            mass_shape = pymunk.Poly.create_box(mass, (segment_length, thickness))
            mass_shape.density = .005
            mass_shape.friction = 0.5
            mass_shape.collision_type = 2
            self.segments.append(mass)
            self.segments_shapes.append(mass_shape)
        for i in range(segmentNum-1):
            joint = pymunk.PivotJoint(self.segments[i], self.segments[i + 1], (x+i*segment_length+segment_length/2, y))
            self.joints.append(joint)
    def add(self, space: pymunk.Space):
        space.add(*self.segments)
        space.add(*self.segments_shapes)
        space.add(*self.joints)