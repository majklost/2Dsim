import time
from datetime import datetime
import os

from deform_plan.helpers.config_manager import ConfigManager
from deform_plan.helpers.seed_manager import init_manager
from deform_plan.planners.fetchable_planner import FetchAblePlanner
from deform_plan.rrt_utils.planning_factory import get_main_idxs, Point
from deform_plan.rrt_utils.prepared_templates import prepare_standard_sampler, make_creased_cost, \
    get_planning_fncs_cost, get_planning_fncs_standard, make_cost_fnc, prepare_trrt_wrapper, distance, \
    prepare_standard_wrapper, prepare_guiding_paths, prepare_goal_bias_sampler
from deform_plan.saveables.replayable_path import ReplayablePath
from deform_plan.storages.GNAT import GNAT
from deform_plan.utils.PM_space_visu import show_sim, make_draw_circle
from deform_plan.utils.analytics import Analytics, save_analytics
from parser_helper import parser
from maps import get_map


args = parser.parse_args()
NAME = args.name
OUTPUT_DIR = args.output
REACHED = False


if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

VISUAL = False  # Showing the visualisation

STORAGE_WRAPPER_TIME = 0
SAMPLER_TIME = 0

cfg = ConfigManager({})
cfg.load_from_file(args.config)
ITERNUM = cfg.ITERATIONS

cfg.update({"seed_plan": int(args.seed)})

init_manager(cfg.seed_env, cfg.seed_plan)
cur_map = get_map(args.map, cfg)
movable_idx = cur_map.get_movable_idx()
START_POS = cur_map.get_start()  # (x,y)
GOAL_POS = cur_map.get_goal()  # (x,y)
start_points = cur_map.get_start_points()  # list of points
goal_points = cur_map.get_goal_points()  # list of points
sim = cur_map.get_sim()
sampler = prepare_standard_sampler(cfg)
storage = GNAT(distancefnc=distance)
storage_utils = {
    "dist": distance,
    "cls_maker": Point,
    "cls_kwargs": {"config": cfg},
    "storage": storage,
}

control_fnc = None
t1 = time.time()
# Now cases
if cfg.USE_MAX_CREASED:
    print("Using max creased")
    main_idxs = get_main_idxs(len(start_points), cfg.MAIN_PTS_NUM)
    main_pts_init = start_points[main_idxs]
    cost_fnc = make_creased_cost(main_pts_init, main_idxs)
    control_fnc = get_planning_fncs_cost(
        cfg, movable_idx, cost_fnc, cfg.MAX_CREASED_COST)
else:
    control_fnc = get_planning_fncs_standard(cfg, movable_idx)

if cfg.USE_TRRT:
    print("Using TRRT")
    main_idxs = get_main_idxs(len(start_points), cfg.MAIN_PTS_NUM)
    main_pts_init = start_points[main_idxs]
    cost_fnc = make_cost_fnc(main_pts_init, main_idxs)
    storage_wrapper = prepare_trrt_wrapper(cfg, storage_utils, cost_fnc, Point(
        None, arbitrary_points=goal_points, config=cfg))
else:
    storage_wrapper = prepare_standard_wrapper(
        cfg, storage_utils, Point(None, arbitrary_points=goal_points, config=cfg))

if cfg.USE_GOAL_BIAS:
    print("Using goal bias")
    sampler = prepare_goal_bias_sampler(cfg, goal_points, sampler)

if cfg.USE_SUBSAMPLER:
    print("Using subsampler")
    sampler, storage_wrapper = prepare_guiding_paths(
        cfg, sim.fixed_objects, START_POS, GOAL_POS, sampler, storage_wrapper, VISUAL)

if VISUAL:
    def debug_draw(surf):
        for i, g in enumerate(goal_points):
            if i in cfg.CONTROL_IDXS:
                make_draw_circle(g, 5, (255, 0, 0))(surf)
                # print(f"Control {i} point in _goal_points: ", g)
            else:
                make_draw_circle(g, 5, (0, 0, 255))(surf)
    show_sim(sim, clb=debug_draw)

planner = FetchAblePlanner(sim, control_fnc,
                           max_step_cnt=cfg.MAX_STEPS,
                           guider_period=cfg.GUIDER_PERIOD,
                           sampling_period=cfg.SAMPLING_PERIOD,
                           only_simuls=cfg.ONLY_SIMULS,
                           track_analytics=True)
start = planner.form_start_node()
storage_wrapper.save_to_storage(start)


print("Starting iterations")
for i in range(cfg.ITERATIONS):
    if not storage_wrapper.want_next_iter:
        print("Finished in ", i)
        REACHED = True
        ITERNUM = i
        break
    if cur_map.finished():
        print("Finished in ", i)
        REACHED = True
        ITERNUM = i
        break
    t_sampler = time.time()
    qrand = sampler.sample()
    g = Point(None, arbitrary_points=qrand, config=cfg)
    t_s = time.time()
    SAMPLER_TIME += t_s - t_sampler
    q_near = storage_wrapper.get_nearest(g)
    t_e = time.time()
    STORAGE_WRAPPER_TIME += t_e - t_s
    response = planner.check_path(q_near.node, g)
    t_s = time.time()
    for r in response.checkpoints:
        res = storage_wrapper.save_to_storage(r)
        if not res:
            break
    t_e = time.time()
    STORAGE_WRAPPER_TIME += t_e - t_s

    if i % 100 == 0:
        print("iter: ", i)

t2 = time.time()

path = storage_wrapper.get_path()
all_pts = [pt.main_points for pt in storage_wrapper.storage.get_all_nodes()]
additional_data = {
    "goal_points": goal_points,
    "start_points": start_points,
    "nodes": all_pts,
    "config": cfg.get(),
    "reached": REACHED,
    "iterations": ITERNUM
}
if cfg.USE_SUBSAMPLER:
    additional_data["heur_paths"] = sampler.get_paths()

ANALYTICS: Analytics = {
    "tot_time": t2-t1,
    "storage_wrapper_time": STORAGE_WRAPPER_TIME,
    "sample_time": SAMPLER_TIME,
    "planner_data": planner.analytics,
    "storage_data": storage_wrapper.analytics(),
    "sampler_data": sampler.analytics(),
    "additional_data": {"REACHED": REACHED, "ITERATIONS": ITERNUM},
    "creation_date": datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
}


rp = ReplayablePath(sim,
                    path,
                    Point(None, arbitrary_points=goal_points, config=cfg),
                    guider=control_fnc["guider"],
                    reached_condition=control_fnc["reached_condition"],
                    guider_period=cfg.GUIDER_PERIOD,
                    additional_data=additional_data)
cfg.check_unused()
print("Saving to ", f"{OUTPUT_DIR}/{NAME}.rpath")
rp.save(f"{OUTPUT_DIR}/{NAME}.rpath")
save_analytics(f"{OUTPUT_DIR}/{NAME}.analytics", ANALYTICS)
print("Done")
