import pymunk

DENSIY = 0.05
SEP_TO_SEGMENT_RATIO = 2.3
# SEP_TO_SEGMENT_RATIO = 2.5
DIST_DEVIATION_TOLERANCE = 0.01
DIST_DEVIATION_PENALTY = 70


class Foam:
    """
    https://graphics.stanford.edu/~mdfisher/cloth.html
    https://www.scss.tcd.ie/michael.manzke/CS7057/cs7057-1516-14-MassSpringSystems-mm.pdf
    https://graphics.stanford.edu/courses/cs468-02-winter/Papers/Rigidcloth.pdf
    """

    # STANDARD_StructuralSpringParams = Foam.SpringParams(2000, 2000)
    # STANDARD_ShearSpringParams = SpringParams(300, 50)

    def __init__(self, x, y, w, h, masspoints_per_length, structSpringParams, shearSpringParams):
        """

        :param x: x coord of position of left top corner
        :param y: y coord of position of left top corner
        :param w: width in pixels
        :param h: height in pixels
        :param masspoints_per_length:  how many masspoint on one pixel of length
        :param structSpringParams: stiffness and damping of structural springs
        :param shearSpringParams:  stiffness and damping of shear springs
        """
        self.SHOW_CORRECTION = True
        self.segments = []
        self.segments_shapes = []
        self.structuralSprings = []  #horizontal and vertical
        self.shearSprings = []  #diagonal one

        self.row_mass_num = int(w * masspoints_per_length)  #number of masses in row
        self.col_mass_num = int(h * masspoints_per_length)  #number of masses in column
        self.structuralSpringsParams = structSpringParams
        self.shearSpringsParams = shearSpringParams
        self.sep = w / self.row_mass_num
        self.segment_radius = self.sep / SEP_TO_SEGMENT_RATIO

        self._create_grid(x, y)

    def _create_grid(self, x, y):
        for i in range(self.col_mass_num):
            row, row_shape = self._fillLine(x, y + i * self.sep, self.row_mass_num, self.sep)
            self.segments.append(row)
            self.segments_shapes.append(row_shape)
        # add vertical springs
        for i in range(self.row_mass_num):
            for j in range(self.col_mass_num - 1):
                spring = pymunk.constraints.DampedSpring(self.segments[j][i], self.segments[j + 1][i], (0, 0), (0, 0),
                                                         self.sep,
                                                         self.structuralSpringsParams.stiffness,
                                                         self.structuralSpringsParams.damping)
                spring.force_func = self._spring_force_fnc
                self.structuralSprings.append(spring)

    def _create_cross_springs(self):
        #add  \ diagonal springs
        for i in range(self.row_mass_num - 1):
            for j in range(self.col_mass_num - 1):
                spring = pymunk.constraints.DampedSpring(self.segments[j][i], self.segments[j + 1][i + 1], (0, 0),
                                                         (0, 0),
                                                         self.sep * (2) ** 0.5,
                                                         self.shearSpringsParams.stiffness,
                                                         self.shearSpringsParams.damping)
                # spring.force_func = self._spring_force_fnc
                self.shearSprings.append(spring)
        # add / diagonal springs
        for i in range(self.row_mass_num - 1):
            for j in range(self.col_mass_num - 1):
                spring = pymunk.constraints.DampedSpring(self.segments[j + 1][i], self.segments[j][i + 1], (0, 0),
                                                         (0, 0),
                                                         self.sep * (2) ** 0.5,
                                                         self.shearSpringsParams.stiffness,
                                                         self.shearSpringsParams.damping)
                # spring.force_func = self._spring_force_fnc
                self.shearSprings.append(spring)

    def _spring_force_fnc(self, spring: pymunk.DampedSpring, dist):

        if dist > spring.rest_length * (1 + DIST_DEVIATION_TOLERANCE):
            if self.SHOW_CORRECTION:
                for s in spring.a.shapes.union(spring.b.shapes):
                    s.color = (255, 0, 0, 255)
            return -DIST_DEVIATION_PENALTY * spring.stiffness * (dist - spring.rest_length)
        if self.SHOW_CORRECTION:
            for s in spring.a.shapes.union(spring.b.shapes):
                s.color = (0, 0, 255, 255)
        return -spring.stiffness * (dist - spring.rest_length)

    def _fillLine(self, x, y, num, sep):
        """
        fill horizontal line with masses and connect them with springs
        :param x:
        :param y:
        :param num:
        :return:
        """
        seg_line = []
        seg_shape_line = []
        for i in range(num):
            mass = pymunk.Body(0, 0, body_type=pymunk.Body.DYNAMIC)
            mass.position = x + i * sep, y
            mass_shape = pymunk.Circle(mass, self.segment_radius)
            mass_shape.density = DENSIY
            # mass_shape.friction = 0.5
            mass_shape.collision_type = 1
            seg_line.append(mass)
            seg_shape_line.append(mass_shape)
        for i in range(num - 1):
            spring = pymunk.constraints.DampedSpring(seg_line[i], seg_line[i + 1], (0, 0), (0, 0), self.sep,
                                                     self.structuralSpringsParams.stiffness,
                                                     self.structuralSpringsParams.damping)
            spring.force_func = self._spring_force_fnc
            self.structuralSprings.append(spring)
        return seg_line, seg_shape_line

    def add(self, space: pymunk.Space):
        space.add(*[s for seg in self.segments for s in seg])
        space.add(*[s for seg in self.segments_shapes for s in seg])
        space.add(*self.structuralSprings)
        space.add(*self.shearSprings)
        # space.add(*self.centerSprings)

    class SpringParams:

        def __init__(self, stiffness, damping):
            self.stiffness = stiffness
            self.damping = damping


