
import time

import numpy as np

from deform_plan.assets.PM.objects.boundings import Boundings
from deform_plan.assets.PM.objects.one_end_cable import OneEndCable
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import make_draw_line, make_draw_circle, show_sim
from deform_plan.utils.analytics import print_analytics

from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from examples.week2.w2_utils import *
from examples.week2.config import CONFIG

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
    CABLE_LENGTH = CONFIG["CABLE_LENGTH"]
    SEGMENT_NUM = CONFIG["SEGMENT_NUM"]
    MAX_FORCE_PER_SEGMENT = CONFIG["MAX_FORCE_PER_SEGMENT"]
    cfg = CONFIG["CFG"]
    cable = Cable([100, 70], 400, SEGMENT_NUM, thickness=5)
    obstacle_g = RandomObstacleGroup(np.array([50, 300]), 200, 200, 4, 2, radius=100,seed=20)
    bounding = Boundings(cfg.width, cfg.height)
    sim = Simulator(cfg, [cable], [bounding,obstacle_g], threaded=False,unstable_sim=True)

    GUIDER_PERIOD = CONFIG["GUIDER_PERIOD"]

    OUTSIDE_CHPATH = 0
    ITERATIONS = CONFIG["ITERATIONS"]
    lb = np.array([0, 0, 0])
    ub = np.array([800, 800, 2 * np.pi])
    sampler = BezierSampler(CABLE_LENGTH, SEGMENT_NUM, lb, ub, seed=16)
    goal_points = sampler.sample(x=280, y=720, angle=0)
    control_idxs = CONFIG["CONTROL_IDXS"]
    # control_idxs = [0, SEGMENT_NUM - 1]
    # control_idxs = [0]

    dist_matrix = calc_distance_matrix(sim, 0)
    guider = make_guider(0, control_idxs, MAX_FORCE_PER_SEGMENT,dist_matrix)
    ender = make_fail_condition(0)
    control_fnc = {
        "guider": guider,
        "fail_condition": ender,
        "reached_condition": make_reached_condition(CONFIG["REACHED_THRESHOLD"], SEGMENT_NUM, control_idxs),
        "exporter": make_exporter(0)
    }


    planner = FetchAblePlanner(sim,
                               control_fnc,
                               max_iter_cnt=CONFIG["MAX_STEPS"],
                               only_simuls=CONFIG["ONLY_SIMULS"],
                               sampling_period=CONFIG["SAMPLING_PREIOD"],
                               guider_period=CONFIG["GUIDER_PERIOD"],
                                 track_analytics=CONFIG["TRACK_ANALYTICS"]
                               )
    GOAL = Point.from_points(goal_points, control_idxs)
    start = planner.form_start_node()

    storage = StorageWrapper(GOAL,CONFIG["REACHED_THRESHOLD"],control_idxs)
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
    for i in range(ITERATIONS):
        t1 = time.time()
        if i % 100 == 0:
            print("iter: ", i)

        throw = np.random.random()

        if throw < storage.goal_bias:
            print("Throwing goal")
            q_rand = GOAL
        else:
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
        OUTSIDE_CHPATH += t4 - t3 + t2 - t1
    endt = time.time()
    print("Time: ", endt-st)
    planner.analytics["TOTAL"] = endt-st
    planner.analytics["OUTSIDE_CHPATH"] = OUTSIDE_CHPATH
    print_analytics(planner.analytics, ITERATIONS)
    print("OUTSIDE_CHPATH: ", OUTSIDE_CHPATH)
    print("Best dist: ", storage.best_dist)
    path = storage.get_path()
    all_nodes = storage.get_all_points()
    # print("all_nodes: ", all_nodes)
    all_main_points = np.array([p.main_points for p in all_nodes])
    rp = ReplayablePath(sim,path,GOAL,guider,control_fnc["reached_condition"],
                        GUIDER_PERIOD,
                        {"nodes":
        all_main_points,
                         "one_time_draw": draw,
                         "goal" : goal_points,
                         "analytics": planner.analytics,

                         })
    try:
        rp.save("./data/cable_rrt")
    except TypeError:
        print("Cannot save path when debugger attached")


