#let grow tree in obstacles, watch what nodes will be poicked
import numpy as np

from deform_plan.helpers.config_manager import ConfigManager,TRRT,SUBSAMPLER,GOAL_BIAS
from deform_plan.helpers.seed_manager import init_manager
from deform_plan.assets.PM import *
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.simulators.PM.pm_simulator import Simulator
from deform_plan.utils.PM_space_visu import show_sim, make_draw_line, make_draw_circle
from deform_plan.samplers.bezier_sampler import BezierSampler
from deform_plan.samplers.goal_bias_sampler import GoalBiasSampler
from deform_plan.storage_wrappers.cable_wrapper import StorageWrapper
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.rrt_utils.planning_factory import Point
import deform_plan.rrt_utils.planning_factory as fct
from deform_plan.storages.GNAT import GNAT
from w3_utils import distance,distance_inner


if __name__ == "__main__":
    cfg = ConfigManager().clone()
    cfg.update({
        "seed_env": 14,
        "seed_plan": 13,
        # "seed_plan": None,
        "cable_thickness": 5,
        "CABLE_LENGTH": 200,
        "SEGMENT_NUM": 30,
        "CONTROL_IDXS": [i for i in range(30)],
        "TRACK_ANALYTICS": False,
        "ITERATIONS": 100,
    })
    init_manager(cfg.seed_env,cfg.seed_plan)
    def clb(surf):
        make_draw_circle(GOAL,15,(0,0,0))(surf)
    POS = (100,200)
    GOAL = (600,400)
    cable = Cable(POS, cfg.CABLE_LENGTH, cfg.SEGMENT_NUM, thickness=cfg.cable_thickness)
    rect_up = Rectangle(np.array([300, 150]), 1000, 20,STATIC)
    boundaries = Boundings(cfg.CFG["width"], cfg.CFG["height"])
    rect_down = Rectangle(np.array([200, 270]), 600, 20, STATIC)
    rect_down.orientation = 0
    rect_vert = Rectangle(np.array([480, 570]), 600, 20, STATIC)
    rect_vert.orientation = np.pi/2
    sim = Simulator(cfg.CFG, [cable], [rect_up,rect_down,rect_vert,boundaries],threaded=False,unstable_sim=cfg.UNSTABLE_SIM)
    # show_sim(sim,clb=clb)
    lb = np.array([0, 0, 0])
    ub = np.array([800, 800, 2 * np.pi])
    sampler = BezierSampler(cfg.CABLE_LENGTH, cfg.SEGMENT_NUM, lb, ub)
    storage_utils = {
        "dist": distance,
        "cls_maker":Point,
        "cls_kwargs": {"config":cfg},
        "storage": GNAT(distance),
    }

    control_fnc = {
        "guider": fct.make_guider_standard(0, cfg.CONTROL_IDXS, cfg.MAX_FORCE_PER_SEGMENT),
        "fail_condition": fct.make_fail_condition_standard(0),
        "reached_condition": fct.make_reached_condition_standard(0,
                                                                 dist_fnc=distance_inner,
                                                                 control_idxs=cfg.CONTROL_IDXS,
                                                                 threshold=cfg.REACHED_THRESHOLD,
                                                                 main_pts_num=cfg.MAIN_PTS_NUM,
                                                                 all_pts_num=cfg.SEGMENT_NUM,
                                                                 ),
        "exporter": fct.make_exporter_standard(0)
    }
    planner = FetchAblePlanner(sim, control_fnc, max_step_cnt=cfg.MAX_STEPS,guider_period=cfg.GUIDER_PERIOD,sampling_period=cfg.SAMPLING_PERIOD,only_simuls=cfg.ONLY_SIMULS,track_analytics=cfg.TRACK_ANALYTICS)
    start = planner.form_start_node()
    dummy_goal = Point(None,arbitrary_points=sampler.sample(x=GOAL[0],y=GOAL[1],angle=0),config=cfg)
    storage_wrapper = StorageWrapper(storage_utils,dummy_goal,goal_threshold=cfg.REACHED_THRESHOLD,controlled_idxs=cfg.CONTROL_IDXS,verbose=True)
    storage_wrapper.save_to_storage(start)
    # Let tree grow
    for i in range(cfg.ITERATIONS):
        if not storage_wrapper.want_next_iter:
            break
        qrand = sampler.sample()
        g = Point(None,arbitrary_points=qrand,config=cfg)
        q_near = storage_wrapper.get_nearest(g)
        response = planner.check_path(q_near.node, g)
        for r in response.checkpoints:
            storage_wrapper.save_to_storage(r)

    all_pts = [pt.main_points for pt in storage_wrapper.storage.get_all_nodes()]
    def clb2(surf):
        def random_color():
            return np.random.randint(0, 255),np.random.randint(0, 255),np.random.randint(0, 255)
        for pts in all_pts:
            color = random_color()
            for i in range(len(pts)-1):
                make_draw_line(pts[i],pts[i+1],2,color=color)(surf)
            for i in range(len(pts)):
                make_draw_circle(pts[i],4,(255,0,0))(surf)
        make_draw_circle(GOAL,15,(0,0,0))(surf)
    print("Before trying")
    show_sim(sim,clb=clb2)
    #experiment starts


    for i in range(5):
        qrand = sampler.sample(x=GOAL[0],y=GOAL[1])
        g = Point(None, arbitrary_points=qrand, config=cfg)
        q_near = storage_wrapper.get_nearest(g)
        def clb3(surf):
            clb2(surf)
            for i in range(len(qrand)-1):
                make_draw_line(qrand[i],qrand[i+1],9,color=(0,0,255))(surf)
            for i in range(len(q_near.main_points)-1):
                make_draw_line(q_near.main_points[i],q_near.main_points[i+1],9,color=(0,255,0))(surf)
        show_sim(sim,clb=clb3)
