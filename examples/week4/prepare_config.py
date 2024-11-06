from deform_plan.helpers.config_manager import ConfigManager, SUBSAMPLER, GOAL_BIAS
from deform_plan.helpers.config_manager import ConfigManager,SUBSAMPLER,GOAL_BIAS, TRRT


seg_num = 50
conf = {
    'GUIDER_PERIOD': 10,
    'SEGMENT_NUM': seg_num,
    'MAX_FORCE_PER_SEGMENT': .1,
    'ITERATIONS': 25000,
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


# prepare config to file
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

if __name__ == "__main__":
    prepare_vanilla("vanilla_config.json")
    prepare_goal_bias("goal_bias_config.json")
    prepare_subsampler("subsampler_config.json")
    prepare_trrt("trrt_config.json")

