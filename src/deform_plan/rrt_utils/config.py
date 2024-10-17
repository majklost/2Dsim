from deform_plan.assets.PM.configs.pymunk_env_cfg import PMConfig



def config_to_str(cfg):
    s = ""
    for key, value in cfg.items():
        s += f'{key}: {str(value)}\n'