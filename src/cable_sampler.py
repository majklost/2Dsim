import random
from typing import List
import numpy as np



"""
Sample a configuration that is feasible for cable
"""

class CableSampler:

    def __init__(self, cable_length, cable_segments_num, controllable_indexes : List[int], seed=None):
        if seed:
            self.random = random.Random(seed)
        else:
            self.random = random.Random()
        self.cable_length = cable_length
        self.cable_segments_num = cable_segments_num
        self.cable_thickness = None #TODO maybe calculate angle from this
        self.controllable_indexes = controllable_indexes
        self.segment_length = cable_length / cable_segments_num
        self.last_sampled = None #indexes of whole cable
        self.last_angles = []

    def sample(self, sc:'CableSampler.SamplingConstraints') -> np.array:
        self.last_angles = []
        self.last_sampled = self._sample_first(sc)
        return self._only_indexes(self.last_sampled, self.controllable_indexes)
    @staticmethod
    def _only_indexes(points, indexes):
        return [points[i] for i in indexes]

    def _sample_first(self, sc:'CableSampler.SamplingConstraints') -> np.array:
        """
        Sample a feasible configuration, so it does not self-intersect
        :return:
        """


        # Sample first segment (index 0)
        x = self.random.uniform(sc.xMin, sc.xMax)
        y = self.random.uniform(sc.yMin, sc.yMax)

        # Sample the rest of the segments
        return self._sample_next(sc.angleMin, sc.angleMax, [np.array([x,y])], self.cable_segments_num-1, sc)

    def _sample_next(self,angleMin,angleMax,points: List[np.array], remaining_segments_num:int,sc:'CableSampler.SamplingConstraints') -> np.array:
        """
        Sample a feasible configuration, so it does not self-intersect
        :return:
        """
        if remaining_segments_num == 0:
            return np.array(points)


        # Sample next segment
        x,y = [sc.xMin-1,sc.yMin-1]
        angle = self.random.uniform(angleMin, angleMax)
        self.last_angles.append(angle)
        x = points[-1][0] + self.segment_length * np.cos(angle)
        y = points[-1][1] + self.segment_length * np.sin(angle)

        points.append(np.array([x,y]))
        # Check if the cable self-collides
        if self._check_self_col_last_point(points):
            print("Fail")
            points.pop()
            return self._sample_next(angleMin,angleMax,points, remaining_segments_num, sc)
        else:
            return self._sample_next(angleMin,angleMax,points, remaining_segments_num-1, sc)



    def _check_self_col_last_point(self, points: List[np.array]) -> bool:
        """
        Check if after adding last point, the cable self-collides
        :param points:
        :return:
        """
        if len(points) <3:
            return False
        if (points[0] == points[-1]).all():
            return True

        for i in range(len(points)-3):
            if self._segments_intersect(points[i], points[i+1], points[-2], points[-1]):
                    return True
        return False

    @staticmethod
    def _segments_intersect(Ap,Bp,Cp,Dp):
        """
        Check if segments ApBp and CpDp intersect
        taken from https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
        :param Ap:
        :param Bp:
        :param Cp:
        :param Dp:
        :return: bool
        """
        def ccw(A,B,C):
            return (C[1]-A[1]* (B[0]-A[0])) > (B[1]-A[1]) * (C[0]-A[0])
        return ccw(Ap,Cp,Dp) != ccw(Bp,Cp,Dp) and ccw(Ap,Bp,Cp) != ccw(Ap,Bp,Dp)



    class SamplingConstraints:
        def __init__(self, xMin, xMax, yMin, yMax, angleMin, angleMax):
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

    CABLE_LENGTH = 200
    CABLE_SEG_NUM = 30

    space = pymunk.Space()

    cable = MultibodyCable(0, 20, CABLE_LENGTH, CABLE_SEG_NUM, MultibodyCable.standardParams, thickness=5)
    cable.add(space)

    pr = PygameRenderer(800,800,80)


    cs = CableSampler(CABLE_LENGTH+20,CABLE_SEG_NUM,[0,10,29])
    sc = CableSampler.SamplingConstraints(100,600,100,600,0,np.pi)
    print("Start sampling")
    points = cs.sample(sc)
    print("Sample done")


    angles = cs.last_angles
    angles_sums = [sum(angles[:i]) for i in range(1,len(angles)+1)]
    for i,b in enumerate(cable.segments):
        b.position = cs.last_sampled[i].tolist()
        if i > 0:
            b.angle = angles_sums[i-1]

    space.step(1/80)
    while True:
        if not pr.want_running:
            break
        pr.update_cur(space)
