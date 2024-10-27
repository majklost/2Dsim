import numpy as np
from copy import deepcopy



from ..helpers.config_manager import ConfigManager
from ..samplers.bezier_sampler import BezierSampler
from ..samplers.goal_bias_sampler import GoalBiasSampler
from ..storage_wrappers.cable_wrapper import StorageWrapper
from ..storage_wrappers.TRRT import TRRT
from ..samplers.heuristic_sampler import HeuristicSampler,HeuristicWrapper
from ..helpers.rect_heuristic import RectHeuristic
from ..rrt_utils import planning_factory as fct



def distance(p1: 'fct.Point', p2:'fct.Point'):
    # d1  = distance_inner(p1.main_points, p2.main_points)
    # d2 = distance_inner(p1.main_points,np.flip(p2.main_points,0))
    # return min(d1,d2)
    return distance_inner(p1.main_points,p2.main_points)

def distance_inner(p1: np.array, p2:np.array):
    d1 = np.linalg.norm(p1 - p2, axis=1).sum()/len(p1)
    # d2 = np.linalg.norm(np.flip(p1,0)-p2,axis=1).sum()/len(p1)
    return d1

def make_cost_fnc(main_points_init,main_points_idxs):
    creased_standard = make_creased_cost(main_points_init,main_points_idxs)
    def cost_fnc(point: fct.Point):
        all_pts = point.all_points
        return creased_standard(all_pts)
    return cost_fnc

def calc_dist_matrix(main_points:np.array):
    distance_matrix = np.zeros((len(main_points),len(main_points)))
    for i in range(len(main_points)):
        for j in range(i+1,len(main_points)):
            distance_matrix[i,j] = np.linalg.norm(main_points[i]-main_points[j])
            distance_matrix[j,i] = distance_matrix[i,j]
    return distance_matrix



def make_creased_cost(main_points_init,main_points_idxs):
    """calculate cost of creased cable, given points, where to measure"""
    matrix = calc_dist_matrix(main_points_init)
    def creased_cost(all_pts):
        stretch_index = 0
        num_tests = 0
        for i in range(len(main_points_idxs)):
            for j in range(i + 1, len(main_points_idxs)):
                stretch_index += np.linalg.norm(all_pts[main_points_idxs[i]] - all_pts[main_points_idxs[j]]) / matrix[
                    i, j]
                num_tests += 1
        return 1- stretch_index/num_tests
    return creased_cost

def get_planning_fncs_standard(cfg:ConfigManager,movable_idx:int):
    control_fnc = {
        "guider": fct.make_guider_standard(movable_idx,cfg.CONTROL_IDXS,cfg.MAX_FORCE_PER_SEGMENT),
        "fail_condition": fct.make_fail_condition_standard(movable_idx,cfg.CONTROL_IDXS),
        "reached_condition": fct.make_reached_condition_standard(movable_idx,distance_inner,cfg.CONTROL_IDXS,cfg.REACHED_THRESHOLD,cfg.MAIN_PTS_NUM,cfg.SEGMENT_NUM),
        "exporter": fct.make_exporter_standard(movable_idx),
    }
    return control_fnc
def get_planning_fncs_cost(cfg:ConfigManager, movable_idx:int,cost_fnc,max_cost):
    control_fnc = get_planning_fncs_standard(cfg,movable_idx)
    control_fnc["guider"] = fct.make_guider_cutoff(movable_idx,cfg.CONTROL_IDXS,cfg.MAX_FORCE_PER_SEGMENT,max_cost,cost_fnc)
    return control_fnc



def prepare_standard_sampler(cfg:ConfigManager):
    lb = np.array([0, 0, 0])
    ub = np.array([cfg.CFG['width'], cfg.CFG['height'], 2 * np.pi])
    sub_sampler = BezierSampler(cfg.CABLE_LENGTH, cfg.SEGMENT_NUM, lb, ub)
    return sub_sampler

def prepare_goal_bias_sampler(cfg:ConfigManager,goal_points, subsampler):
    # subsampler = prepare_standard_sampler(cfg)
    sampler = GoalBiasSampler(subsampler, goal_points, cfg.GOAL_BIAS, verbose=True)
    return sampler


def prepare_standard_wrapper(cfg: ConfigManager, storage_utils,goal):
    storage_wrapper = StorageWrapper(storage_utils, goal, goal_threshold=cfg.REACHED_THRESHOLD,
                                     controlled_idxs=cfg.CONTROL_IDXS, verbose=True)
    return storage_wrapper

def prepare_trrt_wrapper(cfg, storage_utils, cost_fnc,goal):
    """

    :param cfg: config
    :param storage_utils:
    :param cost_fnc: (node) -> R+
    :param goal: expected goal packed in cls_maker
    :return:
    """
    trrt_utils = {}
    trrt_utils.update(cfg.TRRT)
    trrt_utils.update({
        "cost_fnc": cost_fnc
    })

    storage_wrapper = TRRT(storage_utils,trrt_utils,goal,cfg.REACHED_THRESHOLD,controlled_idxs=cfg.CONTROL_IDXS, verbose=True)
    return storage_wrapper

def prepare_guiding_paths(cfg,
                          fixed_objects,
                          start,
                          goal,
                          child_sampler,
                          child_wrapper,
                          show_sim_bool=True):
    """
    Return both _sampler and wrapper
    :param cfg: config of simulation with expected parameters for SUBSAMPLER
    :param fixed_objects: fixed objects of simulation
    :param start: 2D point
    :param goal: 2D point
    :param child_sampler: subsampler that samples when random chosen
    :param child_wrapper: wrapper
    :param show_sim_bool:  show which paths were found
    :return: _sampler, wrapper
    """
    sampler_data = setup_heuristic_config(cfg,start,goal)
    paths =[]
    for i in range(cfg.SUBSAMPLER_RUNS):
        rh = RectHeuristic(fixed_objects,sampler_data,cfg.CFG,show_sim_bool,show_sim_bool)
        path = rh.run()
        if path:
            paths.append(path)
    # for p in paths[0]:
    #     print("POINT: ", p)
    paths = []
    print("Found ", len(paths), " paths")
    heuristic_sampler = HeuristicSampler(child_sampler,child_wrapper,paths,cfg.PATH_PROB,cfg.STD_DEV)
    heuristic_wrapper = heuristic_sampler.create_wrapper()
    return heuristic_sampler,heuristic_wrapper


def setup_heuristic_config(cfg, start,goal):
    """

    :param cfg: config with subsampler addon
    :param start: 2D point
    :param goal:  2D point
    :return:
    """
    sampler_data = deepcopy(cfg.SUBSAMPLER)
    sampler_data["start"] = start
    sampler_data["_goal_points"] = goal
    return sampler_data


