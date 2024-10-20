#Similar functionaliyy to w2_cable_rrt.py, but with refactored code
import time
import numpy as np

from deform_plan.helpers.config_manager import ConfigManager, TRRT, SUBSAMPLER, GOAL_BIAS, CREASED
from deform_plan.helpers.seed_manager import init_manager
from deform_plan.assets.PM import *
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.utils.PM_space_visu import show_sim, make_draw_line, make_draw_circle
from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.samplers.goal_bias_sampler import GoalBiasSampler
from deform_plan.storage_wrappers.cable_wrapper import StorageWrapper
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.rrt_utils.planning_factory import Point, get_main_idxs
import deform_plan.rrt_utils.planning_factory as fct
from deform_plan.storages.GNAT import GNAT
from examples.week3.w3_utils import make_creased_cost, make_cost_fnc, get_planning_fncs_standard, \
    get_planning_fncs_cost
from w3_utils import distance,distance_inner,prepare_standard_wrapper,prepare_standard_sampler,prepare_goal_bias_sampler,prepare_trrt_wrapper,prepare_guiding_paths
from deform_plan.utils.PM_debug_viewer import DebugViewer



if __name__ == "__main__":
    def draw(sim, surf, additional_data: dict):
        import pygame
        import numpy as np
        def random_color():
            return np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255)

        nodes = additional_data["nodes"]
        if not "colors" in additional_data:
            additional_data["colors"] = [random_color() for _ in range(len(nodes))]
        for x, node in enumerate(nodes):
            for i in range(len(node) - 1):
                pygame.draw.line(surf, additional_data["colors"][x], node[i], node[i + 1], 2)
            for i in range(len(node)):
                pygame.draw.circle(surf, (255, 0, 0), node[i], 4)
        for gpoint in additional_data["goal_points"]:
            pygame.draw.circle(surf, (0, 0, 255), gpoint, 5)
        if "heuristic_paths" in additional_data:
            for path in additional_data["heuristic_paths"]:
                for i in range(len(path) - 1):
                    pygame.draw.line(surf, (0, 0, 255), path[i], path[i + 1], 5)
                for i in range(len(path)):
                    pygame.draw.circle(surf, (0, 0, 255), path[i], 10)



    def debug_draw(surf):
        for i, g in enumerate(goal_points):
            if i in cfg.CONTROL_IDXS:
                make_draw_circle(g, 5, (255, 0, 0))(surf)
                # print(f"Control {i} point in goal_points: ", g)
            else:
                make_draw_circle(g, 5, (0, 0, 255))(surf)
    cfg = ConfigManager().clone()
    cfg.update({
        "seed_env": 10,
        "seed_plan": 45,
        "cable_thickness": 5,
        "ITERATIONS": 5000
    })
    cfg.update(GOAL_BIAS)
    # cfg.update(CREASED)
    # cfg.update({
    #     "MAX_CREASED_COST": 0.6
    # })
    # cfg.update({
    #     "USE_GOAL_BIAS": False,
    #     "ITERATIONS": 1000,
    # })
    # cfg.update(TRRT)
    cfg.update(SUBSAMPLER)

    init_manager(cfg.seed_env, cfg.seed_plan)
    POS = (100,70)
    GOAL_POS = (400,840)
    cable = Cable(POS, cfg.CABLE_LENGTH, cfg.SEGMENT_NUM, thickness=cfg.cable_thickness)
    obstacle_g = RandomObstacleGroup(np.array([50, 350]), 200, 200, 4, 2, radius=100)
    bounding = Boundings(cfg.CFG['width'], cfg.CFG['height'])
    sim = Simulator(cfg.CFG, [cable], [bounding,obstacle_g], threaded=cfg.THREADED_SIM, unstable_sim=cfg.UNSTABLE_SIM)
    sampler = prepare_standard_sampler(cfg)
    goal_points = sampler.sample(x=GOAL_POS[0],y=GOAL_POS[1],angle=0,fixed_shape=True)
    storage = GNAT(distancefnc=distance)
    goal = Point(None,arbitrary_points=goal_points,config=cfg)
    storage_utils ={
        "dist": distance,
        "cls_maker": Point,
        "cls_kwargs": {"config": cfg},
        "storage": storage,
    }

    # control_fnc = {
    #     "guider": fct.make_guider_standard(0,cfg.CONTROL_IDXS,cfg.MAX_FORCE_PER_SEGMENT),
    #     "fail_condition": fct.make_fail_condition_standard(0),
    #     "reached_condition": fct.make_reached_condition_standard(0,distance_inner,cfg.CONTROL_IDXS,cfg.REACHED_THRESHOLD,cfg.MAIN_PTS_NUM,cfg.SEGMENT_NUM),
    #     "exporter": fct.make_exporter_standard(0),
    # }
    control_fnc = None
    if cfg.USE_MAX_CREASED:
        print("Creased cost")
        all_pts_init = sim.movable_objects[0].position
        main_idxs = get_main_idxs(len(all_pts_init),cfg.MAIN_PTS_NUM)
        main_pts_init = all_pts_init[main_idxs]
        cost_fnc = make_creased_cost(main_pts_init,main_idxs)
        control_fnc = get_planning_fncs_cost(cfg,0,cost_fnc,cfg.MAX_CREASED_COST)
    else:
        control_fnc = get_planning_fncs_standard(cfg,0)

    planner = FetchAblePlanner(sim, control_fnc,
                               max_step_cnt=cfg.MAX_STEPS,
                               guider_period=cfg.GUIDER_PERIOD,
                               sampling_period=cfg.SAMPLING_PERIOD,
                               only_simuls=cfg.ONLY_SIMULS,
                               track_analytics=cfg.TRACK_ANALYTICS,
                               )
    start = planner.form_start_node()

    main_idxs = get_main_idxs(cfg.SEGMENT_NUM,cfg.MAIN_PTS_NUM)
    init_main_pts = start.exporter_data["points"][main_idxs]


    if cfg.USE_TRRT:
        cost_fnc = make_cost_fnc(init_main_pts,main_idxs)
        storage_wrapper = prepare_trrt_wrapper(cfg,storage_utils,cost_fnc,goal)
    else:
        storage_wrapper = prepare_standard_wrapper(cfg,storage_utils,goal)
    storage_wrapper.save_to_storage(start)

    if cfg.USE_GOAL_BIAS:
        sampler = prepare_goal_bias_sampler(cfg,goal_points,sampler)
    else:
        sampler = prepare_standard_sampler(cfg)

    if cfg.USE_SUBSAMPLER:
        sampler,storage_wrapper = prepare_guiding_paths(cfg,sim.fixed_objects,POS,GOAL_POS,sampler,storage_wrapper,True)

    if cfg.TRACK_ANALYTICS:
        t1 = time.time()

    show_sim(sim,clb=debug_draw)
    # viewer = DebugViewer(sim,realtime=False)
    # g= goal
    # def debug_viewer_clb(surf):
    #     for pn in g.all_points:
    #         make_draw_circle(pn, 5, (0, 255, 0))(surf)
    #     make_draw_circle(sampler.last_chpoint(),10,(255,0,0))(surf)
    # viewer.draw_clb = debug_viewer_clb

    for i in range(cfg.ITERATIONS):
        if not storage_wrapper.want_next_iter:
            break
        qrand = sampler.sample()
        g = Point(None,arbitrary_points=qrand,config=cfg)
        q_near = storage_wrapper.get_nearest(g)
        response = planner.check_path(q_near.node, g)
        for r in response.checkpoints:
            storage_wrapper.save_to_storage(r)
        if cfg.VERBOSE and i % 100 == 0:
            print("iter: ", i)
    if cfg.TRACK_ANALYTICS:
        t2 = time.time()
        print("Time: ", t2 - t1)
        planner_analytics = planner.analytics
        for k,v in planner_analytics.items():
            print(k,": ",v)
    all_pts = [pt.main_points for pt in storage.get_all_nodes()]
    path = storage_wrapper.get_path()
    additional_data ={
        "one_time_draw": draw,
        "goal_points": goal_points,
        "steps": cfg.MAX_STEPS,
        "config": cfg,
        "nodes": all_pts,
        "analytics": planner.analytics,
    }
    if cfg.USE_SUBSAMPLER:
        additional_data.update({"heuristic_paths": sampler.get_paths()})


    rp = ReplayablePath(
        sim, path, goal, control_fnc["guider"], control_fnc["reached_condition"], cfg.GUIDER_PERIOD,additional_data
    )
    try:
        rp.save("./data/cable_rrt")
    except TypeError:
        print("Could not save path when debugger attached")


    cfg.check_unused()









