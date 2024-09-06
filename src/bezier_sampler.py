import numpy as np

from typing import List
# TODO - rewrite to sampling bezier curve and then sampling points on it


def create_bezier(A,B,C,D):
    """
    create bezier curve with control points A,B,C,D
    """
    def bezier(t):
        return (1-t)**3*A + 3*(1-t)**2*t*B + 3*(1-t)*t**2*C + t**3*D
    return bezier
ADD_SPACING = 0


class Sampler:
    """
    algorithm to create a path that is non intersecting
    1) [0,0] and [20,0] are endpoints
    2) create rectangle that one side is collinear with the line between the points, same length
    and second side is perpendicular to the line twice the length
    |-|
    A-D
    |-|
    3) sample 2 points B,C on the rectangle sides
    4) form a bezier curve with control points A, B, C, D
    5) bezier curve is parametrize by t in [0,1]
    6) create n+1 points on curve by equidistant sampling of t
    7) MAYBE - TODO - create mapping estimate by https://gamedev.stackexchange.com/questions/5373/moving-ships-between-two-planets-along-a-bezier-missing-some-equations-for-acce/5427#5427
    8) get centers of the n segments between points,these are midpoints of cable segments, get also slopes in these points
    9) scale all segments to fit the desired cable length - this also fixes non uniform lengths of segments
    10) return the points

    """
    def __init__(self, cable_length,cable_segments_num,controllable_indexes: List[int], seed=None):
        if seed is not None:
            np.random.seed(seed)
        self.cable_length = cable_length
        self.cable_segments_num = cable_segments_num
        self.controllable_indexes = controllable_indexes
        self.sc=None
        self.segment_length = cable_length / cable_segments_num + ADD_SPACING

        self.last_sampled =[]
        self.last_angles = []

    def sample(self,sc: 'BezierSampler.SamplingConstraints'):
        curve_points = self._get_curve_points(sc)
        directions = self.calc_directions(curve_points)

        dir_lengths = [np.linalg.norm(d) for d in directions]
        coefs = [self.segment_length / dl for dl in dir_lengths]
        new_dirs = [d * c for d, c in zip(directions, coefs)]
        self.last_angles =self.dirs_to_angles(new_dirs)
        self.last_sampled = self.create_midpoints(self.dirs_to_points(curve_points[0],new_dirs))

        return self.arbitrary_points(self.last_sampled,self.controllable_indexes)

    def extract_all_points(self):
        return self.last_sampled


    def _get_curve_points(self, sc: 'BezierSampler.SamplingConstraints'):
        x_controls = np.random.uniform(0, 20, 2)
        y_controls = np.random.uniform(-20, 20, 2)
        A = np.array([0, 0])
        B = np.array([x_controls[0], y_controls[0]])
        C = np.array([x_controls[1], y_controls[1]])
        D = np.array([20, 0])
        bezier = create_bezier(A, B, C, D)

        aff_x = np.random.uniform(sc.xMin, sc.xMax)
        aff_y = np.random.uniform(sc.yMin, sc.yMax)
        affine = np.array([aff_x, aff_y])

        rot_angle = np.random.uniform(sc.angleMin, sc.angleMax) # rotation of whole object
        rot_matrix = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], [np.sin(rot_angle), np.cos(rot_angle)]])


        points = [rot_matrix@bezier(t) + affine for t in np.linspace(0, 1, self.cable_segments_num+1)]
        return points

    @staticmethod
    def arbitrary_points(points,indexes):
        return [points[i] for i in indexes]

    @staticmethod
    def calc_directions(points):
        directions = [points[i + 1] - points[i] for i in range(len(points) - 1)]
        return directions

    @staticmethod
    def dirs_to_points(start_point,directions):
        points = [start_point]
        for d in directions:
            points.append(points[-1] + d)
        return points


    @staticmethod
    def create_midpoints(points):
        midpoints = [(points[i] + points[i + 1]) / 2 for i in range(len(points) - 1)]
        return midpoints

    @staticmethod
    def dirs_to_angles(directions):
        """
        directions to angle with x axis
        :param directions:
        :return:
        """
        return [np.arctan2(d[1], d[0]) for d in directions]


    class SamplingConstraints:
        def __init__(self, xMin, xMax, yMin, yMax, angleMin=0, angleMax=2 * np.pi):
            """
            curve is moved into sampled point - sampled in xMin,xMax,yMin,yMax and rotated by rotation angleMin,angleMax
            :param xMin: min x coordinate to sample first segment
            :param xMax:
            :param yMin:
            :param yMax:
            :param angleMin: max of rotation of whole curce
            :param angleMax:
            """
            self.xMin = xMin
            self.xMax = xMax
            self.yMin = yMin
            self.yMax = yMax
            self.angleMin = angleMin
            self.angleMax = angleMax


if __name__ == "__main__":
    from helpers.PygameRenderer import PygameRenderer
    import pymunk
    from src.helpers.cables import MultibodyCable
    import pygame


    CABLE_LENGTH = 200
    CABLE_SEG_NUM = 30

    space = pymunk.Space()

    cable = MultibodyCable(0, 20, CABLE_LENGTH, CABLE_SEG_NUM, MultibodyCable.standardParams, thickness=5)
    cable.add(space)



    cs = BezierSampler(CABLE_LENGTH,CABLE_SEG_NUM,[0,1,2])
    sc = BezierSampler.SamplingConstraints(100,600,100,600,0,2*np.pi)

    for i in range(10):
        pr = PygameRenderer(800,800,80)
        print("Start sampling")
        points = cs.sample(sc)
        print("Sample done")
        angles = cs.last_angles
        for i,b in enumerate(cable.segments):
            b.position = cs.last_sampled[i].tolist()
            b.angle = angles[i]

        space.step(1/80) #just to show what was generated
        while True:
            if not pr.want_running:
                break

            pr.update_cur(space)




