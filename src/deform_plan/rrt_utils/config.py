from deform_plan.assets.PM.configs.pymunk_env_cfg import PMConfig

CONFIG = {
    'CABLE_LENGTH': 400,
    'SEGMENT_NUM': 70,
    'MAX_FORCE_PER_SEGMENT': .1, # Max force per segment
    'GUIDER_PERIOD': 5, # How often in steps can guider change direction of cable
    'ITERATIONS': 5000, # Number of iterations for RRT
    'CFG': PMConfig(),
    'REACHED_THRESHOLD': 20, # How close to goal should cable be
    'CONTROL_IDXS': [i for i in range(70)], # Indexes of controlled points
    'SAMPLING_PERIOD': 30, #period of steps in pymunk between samples
    'MAX_STEPS': 500, # Max steps of pymunk simulation
    'ONLY_SIMULS': True, # If True, no fetching is done
    'TRACK_ANALYTICS': True, # If True, analytics are tracked
    'MAIN_PTS_NUM': 4, # Number of main points that describes cable
    'MAX_CREASED_COST': 0.9, # If set to 1, accept all cables, if 0, only straight cables
    'USE_MAX_CREASED': True, # If False, stretch index is not used
    'THREADED_SIM' : False, # When True, simulation is threaded (internal in pymunk)
    'UNSTABLE_SIM': True, # when True, simulation is not copied, only objects
    'USE_TRRT': True,
    'TRRT':{
        'T': 1e5, # Temperature
        'nfail_max': 10, # Number of fails before temperature is updated
        'alpha': 2, # Temperature update factor
    },
    'USE_GOAL_BIAS': True,
    'GOAL_BIAS': 0.01, # Probability of selecting goal instead of random point

}

def config_to_str(cfg):
    s = ""
    for key, value in cfg.items():
        s += f'{key}: {str(value)}\n'