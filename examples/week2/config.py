from deform_plan.assets.PM.configs.pymunk_env_cfg import PMConfig

CONFIG = {
    'CABLE_LENGTH': 400,
    'SEGMENT_NUM': 70,
    'MAX_FORCE_PER_SEGMENT': .1,
    'GUIDER_PERIOD': 5,
    'ITERATIONS': 5000,
    'CFG': PMConfig(),
    'REACHED_THRESHOLD': 20,
    'CONTROL_IDXS': [i for i in range(70)],
    'SAMPLING_PREIOD': 30,
    'MAX_STEPS': 500,
    'ONLY_SIMULS': True,
    'TRACK_ANALYTICS': True,
    'MAIN_PTS_NUM': 4,
    'MIN_STRETCH_IDX': 0.7

}

def config_to_str():
    s = ""
    for key, value in CONFIG.items():
        s += f'{key}: {str(value)}\n'