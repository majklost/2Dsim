import pathlib

from deform_plan.helpers.config_manager import ConfigManager, SUBSAMPLER, GOAL_BIAS, TRRT

conf_dir = pathlib.Path(__file__).parent / "configs"



seg_num = 50
conf = {
    'GUIDER_PERIOD': 10,
    'SEGMENT_NUM': seg_num,
    'MAX_FORCE_PER_SEGMENT': .1,
    'ITERATIONS': 15000,
    'REACHED_THRESHOLD': 30,
    'CONTROL_IDXS': [i for i in range(seg_num)],
    'SAMPLING_PERIOD': 50,
    'MAX_STEPS': 5000,
    'ONLY_SIMULS': True,
    'MAIN_PTS_NUM': 4,
    'THREADED_SIM': False,
    'UNSTABLE_SIM': True,
    "USE_MAX_CREASED": False,
    "USE_SUBSAMPLER": False,
    "USE_GOAL_BIAS": False,
    "USE_TRRT": False,
    "seed_env": 10,
}


#prepare config to file
def prepare_vanilla(fpath):


    cm = ConfigManager(conf)
    cm.save_to_file(fpath)

def prepare_goal_bias(fpath):
    cm = ConfigManager(conf)
    cm.update(GOAL_BIAS)
    cm.update({
        "USE_GOAL_BIAS": True,
        "GOAL_BIAS": 0.05,
    })
    cm.save_to_file(fpath)

def prepare_subsampler(fpath):
    cm = ConfigManager(conf)
    cm.update(SUBSAMPLER)
    cm.save_to_file(fpath)


def prepare_trrt(fpath):
    cm = ConfigManager(conf)
    cm.update(TRRT)
    cm.save_to_file(fpath)


def prepare_vanilla_two_controlable(fpath):
    cm = ConfigManager(conf)
    cm.update({
        'CONTROL_IDXS': [0, seg_num-1],
    })
    cm.save_to_file(fpath)


def prepare_vanilla_one_controlable(fpath):
    cm = ConfigManager(conf)
    cm.update({
        'CONTROL_IDXS': [0],
    })
    cm.save_to_file(fpath)


def prepare_subsampler_one_controlable(fpath):
    cm = ConfigManager(conf)
    cm.update(SUBSAMPLER)
    cm.update({
        'CONTROL_IDXS': [0],
    })
    cm.save_to_file(fpath)


def prepare_subsampler_two_controlable(fpath):
    cm = ConfigManager(conf)
    cm.update(SUBSAMPLER)
    cm.update({
        'CONTROL_IDXS': [0, seg_num-1],
    })
    cm.save_to_file(fpath)


if __name__ == "__main__":
    prepare_vanilla(pathlib.Path(conf_dir) / "vanilla_config.json")
    prepare_goal_bias(pathlib.Path(conf_dir) / "goal_bias_config.json")
    prepare_subsampler(pathlib.Path(conf_dir) / "subsampler_config.json")
    prepare_trrt(pathlib.Path(conf_dir) / "trrt_config.json")
    prepare_vanilla_two_controlable(pathlib.Path(
        conf_dir) / "vanilla_two_controlable_config.json")
    prepare_vanilla_one_controlable(pathlib.Path(
        conf_dir) / "vanilla_one_controlable_config.json")
    prepare_subsampler_one_controlable(pathlib.Path(
        conf_dir) / "subsampler_one_controlable_config.json")
    prepare_subsampler_two_controlable(pathlib.Path(
        conf_dir) / "subsampler_two_controlable_config.json")
