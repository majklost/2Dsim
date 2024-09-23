"""check if KD-Trees can be used for cable planning"""
import numpy as np
import time
import profile

from deform_plan.storages.kd_tree import KDTree
from deform_plan.storages.brute_force import BruteForce
from deform_plan.samplers.bezier_sampler import BezierSampler
SEGMENT_NUM = 40

def t3000_nearest_neighbour_weird():
    bsampler = BezierSampler(400,SEGMENT_NUM,np.array([300,300,0]),np.array([500,500,2*np.pi]),seed=26)
    # kd = KDTree(2)
    bf = BruteForce()

    kd_insert_time = 0
    bf_insert_time = 0
    kd_nn_time = 0
    bf_nn_time = 0

    dist_fnc = lambda a,b: np.sum(np.linalg.norm(a-b,axis=1))
    for i in range(300):
        points = np.array(bsampler.sample())
        # t1 = time.time()
        # kd.insert(points)
        # t2 = time.time()
        bf.insert(points)
        # t3 = time.time()
        # kd_insert_time += t2-t1
        # bf_insert_time += t3-t2

        test_points = np.array(bsampler.sample())
        # t1 = time.time()
        # res1 = kd.nearest_neighbour(test_points,dist_fnc)
        # t2 = time.time()
        res2 = bf.nearest_neighbour(test_points,dist_fnc)
        # t3 = time.time()
        # kd_nn_time += t2-t1
        # bf_nn_time += t3-t2
        # if dist_fnc(res1,test_points) == dist_fnc(res2,test_points):
        #     continue
        # assert res1 == res2
    # print()
    # print(f"KD insert time: {kd_insert_time}")
    # print(f"BF insert time: {bf_insert_time}")
    # print(f"KD nn time: {kd_nn_time}")
    # print(f"BF nn time: {bf_nn_time}")

if __name__ == '__main__':
    # test_3000_nearest_neighbour_weird()
    profile.run("t3000_nearest_neighbour_weird()",sort="cumulative")