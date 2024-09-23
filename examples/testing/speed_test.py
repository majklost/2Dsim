"""Test if simulation is slower when cable is there"""
import time
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.assets.PM import *


def run():

    def cable_maker(seg_num):
        return Cable([100,100],400,seg_num,DYNAMIC)

    cables = [cable_maker(i) for i in range(10,100,10)]
    sims = [Simulator(PMConfig(),[cable],[],threaded=False) for cable in cables]

    def benchamrk(sim):
        start = time.time()
        for i in range(10000):
            sim.step()
        end = time.time()
        return end-start


    times = [benchamrk(sim) for sim in sims]
    with open("speed_test.txt","w") as f:
        for i in range(len(cables)):
            f.write(f"{len(cables[i].bodies)} {times[i]}\n")


def visu():
    import matplotlib.pyplot as plt
    with open("speed_test.txt","r") as f:
        lines = f.readlines()
        data = [line.split() for line in lines]
        data = [(int(d[0]),float(d[1])) for d in data]
        x,y = zip(*data)
        plt.plot(x,y)
        plt.plot(x,y,"ro")
        plt.plot((x[0],x[-1]),(y[0],y[-1]),"r--")
        plt.show()


run()
visu()