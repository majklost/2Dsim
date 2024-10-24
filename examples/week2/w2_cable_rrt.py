
import time
from copy import deepcopy

import numpy as np

from deform_plan.assets.PM.objects.boundings import Boundings
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.assets.PM import *
from deform_plan.utils.PM_debug_viewer import DebugViewer
from deform_plan.utils.PM_space_visu import make_draw_circle, show_sim
from deform_plan.utils.analytics import print_analytics

from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from examples.week2.w2_utils import *
from examples.week2.path_sampling import PathSamplerRect
from deform_plan.rrt_utils.config import CONFIG

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
    for gpoint in additional_data["_goal_points"]:
        pygame.draw.circle(surf,(0,0,255),gpoint,5)
    if "heuristic_paths" in additional_data:
        for path in additional_data["heuristic_paths"]:
            for i in range(len(path)-1):
                pygame.draw.line(surf,(255,0,0),path[i],path[i+1],5)
            for i in range(len(path)):
                pygame.draw.circle(surf,(255,0,0),path[i],10)

def cable_to_point_dist(cable_points:np.array,point):
    return np.linalg.norm(np.mean(cable_points,axis=0)-point)



if __name__ == "__main__":
    for k,v in CONFIG.items():
        print(f"{k}: {v}")
    POS = (100,70)

    CABLE_LENGTH = CONFIG["CABLE_LENGTH"]
    SEGMENT_NUM = CONFIG["SEGMENT_NUM"]
    MAX_FORCE_PER_SEGMENT = CONFIG["MAX_FORCE_PER_SEGMENT"]
    cfg = CONFIG["CFG"]
    cable = OneEndCable(POS, CABLE_LENGTH, SEGMENT_NUM, thickness=5)
    obstacle_g = RandomObstacleGroup(np.array([50, 300]), 200, 200, 4, 2, radius=100,seed=20)
    bounding = Boundings(cfg.width, cfg.height)
    sim = Simulator(cfg, [cable], [bounding,obstacle_g], threaded=CONFIG['THREADED_SIM'],unstable_sim=CONFIG['UNSTABLE_SIM'])

    GUIDER_PERIOD = CONFIG["GUIDER_PERIOD"]

    OUTSIDE_CHPATH = 0
    ITERATIONS = CONFIG["ITERATIONS"]
    lb = np.array([0, 0, 0])
    ub = np.array([800, 800, 2 * np.pi])
    sampler = BezierSampler(CABLE_LENGTH, SEGMENT_NUM, lb, ub, seed=10)
    GOAL_POS = (280,570)

    goal_points = sampler.sample(x=GOAL_POS[0], y=GOAL_POS[1], angle=0)
    control_idxs = CONFIG["CONTROL_IDXS"]
    # control_idxs = [0, SEGMENT_NUM - 1]
    # control_idxs = [0]

    paths = []
    if CONFIG["SUBSAMPLER_RUNS"] > 0:
        rng = np.random.RandomState(CONFIG["SUBSAMPLER"]["SEED"])
        for i in range(CONFIG["SUBSAMPLER_RUNS"]):
            path_sampler = PathSamplerRect(deepcopy([bounding,obstacle_g]),CONFIG,{"start":POS,"_goal_points":GOAL_POS},show_sim_bool=True,seed=rng.randint(0,1000))
            path = path_sampler.run()
            if not path:
                continue
            paths.append(path)

        path_idxs = [0 for _ in range(len(paths))]
    dist_matrix = calc_distance_matrix(sim, 0)
    guider = make_guider(0, control_idxs, MAX_FORCE_PER_SEGMENT,dist_matrix)
    ender = make_fail_condition(0)
    control_fnc = {
        "guider": guider,
        "fail_condition": ender,
        "reached_condition": make_reached_condition(CONFIG["REACHED_THRESHOLD"], SEGMENT_NUM, control_idxs),
        "exporter": make_exporter(0)
    }


    planner = FetchAblePlanner(sim, control_fnc, max_step_cnt=CONFIG["MAX_STEPS"], only_simuls=CONFIG["ONLY_SIMULS"],
                               sampling_period=CONFIG["SAMPLING_PERIOD"], guider_period=CONFIG["GUIDER_PERIOD"],
                               track_analytics=CONFIG["TRACK_ANALYTICS"])
    GOAL = Point.from_points(goal_points, control_idxs)
    start = planner.form_start_node()

    if CONFIG["USE_TRRT"]:
        print("Using TRRT")
        storage = StorageWrapperTRRT(GOAL,CONFIG["REACHED_THRESHOLD"],control_idxs,dist_matrix)
    else:
        storage = StorageWrapper(GOAL,CONFIG["REACHED_THRESHOLD"],control_idxs)
    storage.save_to_storage(start)


    #debugging
    def debug_draw(surf):

        for i,g in enumerate(goal_points):
            if i in control_idxs:
                make_draw_circle(g, 5, (255, 0, 0))(surf)
                # print(f"Control {i} point in _goal_points: ", g)
            else:
                make_draw_circle(g, 5, (0, 0, 255))(surf)
    show_sim(sim, clb=debug_draw)
    # dbg = DebugViewer(_sim, realtime=True)
    # dbg.draw_clb = debug_draw
    #end of debugging
    st = time.time()
    real_iters = 0
    for i in range(ITERATIONS):
        real_iters += 1
        t1 = time.time()
        if i % 100 == 0:
            print("iter: ", i)


        if CONFIG['USE_GOAL_BIAS'] and np.random.random() < storage.goal_bias:
            print("Throwing _goal_points")
            q_rand = GOAL
        else:
            if CONFIG['SUBSAMPLER_RUNS'] > 0:
                angle = None
                path_num = len(paths)+1
                if CONFIG["SUBSAMPLE_PATH_ONLY"]:
                    path_num -=1 # Remove one for random point
                if path_num >0:
                    path_idx = np.random.randint(0, path_num)
                else:
                    path_idx = 0
                if path_idx >= len(paths):
                    q_rand = Point.from_points(sampler.sample(), control_idxs)
                else:
                    chosen_path = paths[path_idx]
                    point_on_path_idx = path_idxs[path_idx]
                    if point_on_path_idx != len(chosen_path)-1:
                        next_point = chosen_path[point_on_path_idx+1]
                        direction = next_point - chosen_path[point_on_path_idx]
                        angle = np.arctan2(direction[1],direction[0])

                    wanted = chosen_path[point_on_path_idx]
                    sample = sampler.sample(x=wanted[0], y=wanted[1], angle=angle)
                    q_rand = Point.from_points(sample, control_idxs)
            else:
                q_rand = Point.from_points(sampler.sample(),control_idxs)
        if storage.try_goal:
            print("Trying _goal_points")
            q_rand = GOAL
            storage.try_goal = False
        q_near = storage.get_nearest(q_rand)
        t2 = time.time()
        response = planner.check_path(q_near.node, q_rand)
        t3 = time.time()
        for res_idx in range(len(response.checkpoints)):
            res = response.checkpoints[res_idx] # type: SimNode
            if CONFIG['SUBSAMPLER_RUNS'] > 0:
                res_points = res.exporter_data["points"]
                for x in range(len(paths)):
                    path = paths[x]
                    cur_node = path[path_idxs[x]]
                    dist = cable_to_point_dist(res_points,cur_node)
                    # print("Dist: ", dist)
                    if dist < 50:
                        print("Path: ", x, " reached ", path_idxs[x], "/", len(path))
                        path_idxs[x] += 1
                        if path_idxs[x] >= len(path):
                            path_idxs[x] = len(path)-1
            if res_idx == 0:
                res.previous_node = res.replayer.parent
            else:
                res.previous_node = response.checkpoints[res_idx-1]
            if not storage.save_to_storage(res):
                break
        if not storage.want_next_iter:
            print("Goal reached in iter: ", i)
            break
        t4 = time.time()
        OUTSIDE_CHPATH += t4 - t3 + t2 - t1
    endt = time.time()
    print("Time: ", endt-st)
    if CONFIG["TRACK_ANALYTICS"]:
        planner.analytics["TOTAL"] = endt-st
        planner.analytics["OUTSIDE_CHPATH"] = OUTSIDE_CHPATH
        if hasattr(storage,"overall_rejections"):
            planner.analytics["REJECTED_CNT"] = storage.overall_rejections
        print_analytics(planner.analytics, real_iters)
        print("OUTSIDE_CHPATH: ", OUTSIDE_CHPATH)
    print("Best dist: ", storage.best_dist)
    path = storage.get_path()
    all_nodes = storage.get_all_points()
    print("all_nodes: ", len(all_nodes))
    all_main_points = np.array([p.main_points for p in all_nodes])
    rp = ReplayablePath(sim,path,GOAL,guider,control_fnc["reached_condition"],
                        GUIDER_PERIOD,
                        {"nodes":
        all_main_points,
                         "one_time_draw": draw,
                         "_goal_points" : goal_points,
                         "analytics": planner.analytics,
                         "steps": real_iters,
                         "config": CONFIG,
                         "heuristic_paths": paths
                         })
    try:
        rp.save("./data/cable_rrt")
    except TypeError:
        print("Cannot save path when debugger attached")


