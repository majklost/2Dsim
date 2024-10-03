
import time

import numpy as np

from deform_plan.assets.PM.objects.boundings import Boundings
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import make_draw_line, make_draw_circle, show_sim
from deform_plan.utils.analytics import print_analytics

from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from examples.week1.w1_utils import *

def draw(sim,surf,additional_data:dict):
    import pygame
    import numpy as np
    def random_color():
        return np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255)


    nodes = additional_data["nodes"]
    if not  "colors" in additional_data:
        additional_data["colors"] = [random_color() for _ in range(len(nodes))]
    for x,node in enumerate(nodes):
        for i in range(len(node)-1):
            pygame.draw.line(surf,additional_data["colors"][x],node[i],node[i+1],2)
        for i in range(len(node)):
            pygame.draw.circle(surf,(255,0,0),node[i],4)
    for gpoint in additional_data["goal"]:
        pygame.draw.circle(surf,(0,0,255),gpoint,5)





if __name__ == "__main__":
    CABLE_LENGTH = 400
    SEGMENT_NUM = 70
    MAX_FORCE_PER_SEGMENT = .1
    cfg = PMConfig()
    cable = Cable([100, 70], 400, SEGMENT_NUM, thickness=5)
    obstacle_g = RandomObstacleGroup(np.array([50, 300]), 200, 200, 4, 4, radius=100,seed=20)
    bounding = Boundings(cfg.width, cfg.height)
    sim = Simulator(cfg, [cable], [bounding,obstacle_g], threaded=False)

    GUIDER_PERIOD = 5

    SHITOKOLO = 0
    STEPS = 15000
    lb = np.array([0, 0, 0])
    ub = np.array([800, 800, 2 * np.pi])
    sampler = BezierSampler(CABLE_LENGTH, SEGMENT_NUM, lb, ub, seed=16)
    goal_points = sampler.sample(x=280, y=630, angle=0)
    control_idxs = [i for i in range(SEGMENT_NUM)]
    # control_idxs = [0, SEGMENT_NUM - 1]
    # control_idxs = [0]
    guider = make_guider(0, control_idxs, MAX_FORCE_PER_SEGMENT)
    ender = make_fail_condition(0)
    control_fnc = {
        "guider": guider,
        "fail_condition": ender,
        "reached_condition": make_reached_condition(20, SEGMENT_NUM, control_idxs),
        "exporter": make_exporter(0)
    }


    planner = FetchAblePlanner(sim,
                               control_fnc,
                               max_iter_cnt=500,
                               only_simuls=False,
                               sampling_period=10,
                               guider_period=GUIDER_PERIOD,
                                 track_analytics=True
                               )
    GOAL = Point.from_points(goal_points, control_idxs)
    start = planner.form_start_node()

    storage = StorageWrapper(GOAL,20,control_idxs)
    storage.save_to_storage(start)


    #debugging
    def debug_draw(surf):

        for i,g in enumerate(goal_points):
            if i in control_idxs:
                make_draw_circle(g, 5, (255, 0, 0))(surf)
                # print(f"Control {i} point in goal: ", g)
            else:
                make_draw_circle(g, 5, (0, 0, 255))(surf)
    show_sim(sim, clb=debug_draw)
    # dbg = DebugViewer(sim, realtime=True)
    # dbg.draw_clb = debug_draw
    #end of debugging
    st = time.time()
    for i in range(STEPS):
        t1 = time.time()
        if i % 20 == 0:
            print("iter: ", i)
        q_rand = Point.from_points(sampler.sample(),control_idxs)
        if storage.try_goal:
            print("Trying goal")
            q_rand = GOAL
            storage.try_goal = False
        q_near = storage.get_nearest(q_rand)
        t2 = time.time()
        response = planner.check_path(q_near.node, q_rand)
        t3 = time.time()
        for res in response.checkpoints:
            storage.save_to_storage(res)
        if not storage.want_next_iter:
            print("Goal reached in iter: ", i)
            break
        t4 = time.time()
        SHITOKOLO += t4-t3 +t2-t1
    endt = time.time()
    print("Time: ", endt-st)
    print_analytics(planner.analytics, STEPS)
    print("SHITOKOLO: ", SHITOKOLO)
    print("Best dist: ", storage.best_dist)
    path = storage.get_path()
    all_nodes = storage.get_all_points()
    # print("all_nodes: ", all_nodes)
    all_main_points = np.array([p.main_points for p in all_nodes])
    rp = ReplayablePath(sim,path,GOAL,guider,control_fnc["reached_condition"],
                        GUIDER_PERIOD,
                        {"nodes":
        all_main_points,
                         "drawing_fnc": draw,
                         "goal" : goal_points,
                         })
    try:
        rp.save("./data/cable_rrt")
    except TypeError:
        print("Cannot save path when debugger attached")


