import pymunk


class AbstractCable:
    class SpringParams:
        def __init__(self, stiffness, damping):
            self.stiffness = stiffness
            self.damping = damping



class Cable(AbstractCable):
    """
    A cable implemented from http://dx.doi.org/10.1051/matecconf/20153110002
    Not so good performance, diverges quickly
    but easier manipulation (points are just masses)
    """

    def __init__(self, x, y, length, numMasses, structuralParams: 'AbstractCable.SpringParams', bendingParams: 'AbstractCable.SpringParams'):
        self.masses = []
        self.mass_shapes = []
        self.bending_springs = []
        self.structural_springs = []
        self.length = length
        self.structuralParams = structuralParams  # type: AbstractCable.SpringParams
        self.bendingParams = bendingParams
        sep = self.length / numMasses
        for m in range(numMasses):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + m * sep, y
            mass_shape = pymunk.Circle(mass, sep / 2)
            # mass_shape = pymunk.Poly.create_box(mass, (sep, 5))
            mass_shape.density = .05 / numMasses
            mass_shape.elasticity = 0.5
            mass_shape.friction = 0.5
            mass_shape.collision_type = 1
            self.masses.append(mass)
            self.mass_shapes.append(mass_shape)
        for i in range(numMasses - 1):
            spring = pymunk.constraints.DampedSpring(self.masses[i], self.masses[i + 1], (0, 0), (0, 0), sep,
                                                     self.structuralParams.stiffness, self.structuralParams.damping)
            self.structural_springs.append(spring)
        for i in range(numMasses - 2):
            spring = pymunk.constraints.DampedSpring(self.masses[i], self.masses[i + 2], (0, 0), (0, 0), 2 * sep,
                                                     self.bendingParams.stiffness, self.bendingParams.damping)
            # spring.color = (255, 0, 0, 0)
            self.bending_springs.append(spring)

    def add(self, space: pymunk.Space):
        space.add(*self.masses)
        space.add(*self.structural_springs)
        space.add(*self.bending_springs)
        space.add(*self.mass_shapes)


class MultibodyCable(AbstractCable):
    """
    A review of techniques for modeling flexible cables

    best performance from all tested cables
    """

    def __init__(self, x, y, length, segmentNum, linearParams, thickness=2):
        self.segments = []
        self.segments_shapes = []
        self.linearSprings = []
        self.angularSprings = []
        self.segmentNum = segmentNum
        self.linearParams = linearParams  # type: AbstractCable.SpringParams
        # self.angularParams = angularParams  # type: # AbstractCable.SpringParams

        segment_length = length / segmentNum
        for i in range(segmentNum):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + i * segment_length, y
            mass_shape = pymunk.Poly.create_box(mass, (segment_length, thickness))
            mass_shape.density = .005
            mass_shape.friction = 0.5
            mass_shape.collision_type = 1
            mass_shape.color = (0,0,255,0)
            self.segments.append(mass)
            self.segments_shapes.append(mass_shape)
        for i in range(segmentNum - 1):
            spring = pymunk.constraints.DampedSpring(self.segments[i], self.segments[i + 1], (0.3 * segment_length, 0),
                                                     (-0.3 * segment_length, 0), 0.4 * segment_length,
                                                     self.linearParams.stiffness / segment_length,
                                                     self.linearParams.damping)

            # spring.max_force= 10
            # if i == 0:
            # spring.stiffness *= 10
            self.linearSprings.append(spring)
        for i in range(segmentNum - 1):
            spring = pymunk.constraints.DampedRotarySpring(self.segments[i], self.segments[i + 1], 0, 70, 10)
            self.angularSprings.append(spring)

    def add(self, space: pymunk.Space):
        space.add(*self.segments)
        space.add(*self.segments_shapes)
        space.add(*self.linearSprings)
        space.add(*self.angularSprings)
        # space.add(*self.centerSprings)

    standardParams = Cable.SpringParams(5000, 10)




class HardJointCable(AbstractCable):
    def __init__(self, x, y, length, segmentNum, linearParams, thickness=2):
        self.segments = []
        self.segments_shapes = []
        self.joints = []
        self.segmentNum = segmentNum
        self.linearSprings = []
        self.linearParams = linearParams
        segment_length = length / segmentNum
        for i in range(segmentNum):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + i * segment_length, y
            mass_shape = pymunk.Poly.create_box(mass, (segment_length, thickness))
            mass_shape.density = .005
            mass_shape.friction = 0.5
            mass_shape.collision_type = 1
            self.segments.append(mass)
            self.segments_shapes.append(mass_shape)
        for i in range(segmentNum - 1):
            joint = pymunk.PivotJoint(self.segments[i], self.segments[i + 1],
                                      (x + i * segment_length + segment_length / 2, y))
            self.joints.append(joint)

    def add(self, space: pymunk.Space):
        space.add(*self.segments)
        space.add(*self.segments_shapes)
        space.add(*self.joints)
