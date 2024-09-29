import time
import random

from deform_plan.storages.GNAT import GNAT
from deform_plan.storages.brute_force import BruteForce
from deform_plan.storages.kd_tree import KDTree

dist_fnc = lambda x, y: sum((x[i] - y[i]) ** 2 for i in range(2)) ** 0.5
gnat = GNAT(dist_fnc)
bf = BruteForce(dist_fnc)
kd = KDTree(dist_fnc,2)

def perf_storage():
    gnattime=0
    bftime=0
    kdtime=0

    for i in range(15000):
        x = random.randint(0, 1000)
        y = random.randint(0, 1000)
        xtest = random.randint(0, 1000)
        ytest = random.randint(0, 1000)
        #bf
        # t1 = time.time()
        # bf.insert([x, y])
        # t2 = time.time()
        # bftime += t2-t1
        #kd
        t1 = time.time()
        kd.insert([x, y])
        t2 = time.time()
        kdtime += t2-t1
        #gnat
        t1 = time.time()
        gnat.insert([x, y])
        t2 = time.time()
        gnattime += t2-t1

        #nearest neighbour
        #bf
        # t1 = time.time()
        # bfres = bf.nearest_neighbour([xtest, ytest])
        # t2 = time.time()
        # bftime += t2-t1
        #kd
        t1 = time.time()
        kdres = kd.nearest_neighbour([xtest, ytest])
        t2 = time.time()
        kdtime += t2-t1
        #gnat
        t1 = time.time()
        gnatres = gnat.nearest_neighbour([xtest, ytest])
        t2 = time.time()
        gnattime += t2-t1
        if dist_fnc(gnatres, [xtest, ytest])== dist_fnc(kdres, [xtest, ytest]):
            continue
        assert gnatres == kdres
    print(f"GNAT: {gnattime}")
    # print(f"BF: {bftime}")
    print(f"KD: {kdtime}")


perf_storage()

