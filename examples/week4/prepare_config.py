from deform_plan.helpers.config_manager import ConfigManager

#prepare config to file
def prepare_vanilla(fpath):
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
    cm = ConfigManager(conf)
    cm.save_to_file(fpath)


if __name__ == "__main__":
    prepare_vanilla("vanilla_config.json")

