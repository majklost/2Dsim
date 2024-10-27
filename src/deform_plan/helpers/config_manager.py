import copy
import warnings
from typing import Dict, Any, TypedDict


CONFIG = {
    'CABLE_LENGTH': 400,
    'SEGMENT_NUM': 60,
    'MAX_FORCE_PER_SEGMENT': .1,  # Max force per segment
    'GUIDER_PERIOD': 10,  # How often in steps can guider change direction of cable
    'ITERATIONS': 5000,  # Number of iterations for RRT
    'CFG': {
        'width': 800,
        'height': 1000,
        'FPS': 60,
        'gravity': 0,
        'damping': .15,
        'collision_slope': 0.01,
    },
    'REACHED_THRESHOLD': 30,  # How close to _goal_points should cable be
    'CONTROL_IDXS': [i for i in range(60)],  # Indexes of controlled points
    'SAMPLING_PERIOD': 20,  # period of steps in pymunk between samples
    'MAX_STEPS': 1000,  # Max steps of pymunk simulation
    'ONLY_SIMULS': True,  # If True, no fetching is done
    'TRACK_ANALYTICS': True,  # If True, analytics are tracked
    'MAIN_PTS_NUM': 4,  # Number of main points that describes cable
    # When True, simulation is threaded (internal in pymunk)
    'THREADED_SIM': False,
    'UNSTABLE_SIM': True,  # when True, simulation is not copied, only objects
    "VERBOSE": True,  # If True, print debug information
    "USE_MAX_CREASED": False,
    "USE_SUBSAMPLER": False,
    "USE_GOAL_BIAS": False,
    "USE_TRRT": False
}

CREASED = {
    'MAX_CREASED_COST': 0.5,  # If set to 1, accept all cables, if 0, only straight cables
    'USE_MAX_CREASED': True,  # If False, stretch index is not used
}

SUBSAMPLER = {'SUBSAMPLER':
              {  # Configuration for sampling guiding paths
                  'ITERATIONS': 1000,  # iterations of subsampler
                  'THRESHOLD': 30,  # How close to _goal_points should cable be
                  'VELOCITY': 800,  # Velocity of
                  'POST_PROC_ITER': 200,  # Number of iterations for postprocessing
                  'W': 100,
                  'H': 30,
              },
              'PATH_PROB': .3,  # probability that will create sample on the path
              'STD_DEV': 30,
              'SUBSAMPLER_RUNS': 1,  # Number of runs of subsampler
              "USE_SUBSAMPLER": True
              }

GOAL_BIAS = {
    'USE_GOAL_BIAS': True,
    'GOAL_BIAS': 0.01,  # Probability of selecting _goal_points instead of random point
}

TRRT = {
    'USE_TRRT': True,
    'TRRT': {
        'T': 1e-2,  # Temperature
        'n_fail_max': 1,  # Number of fails before temperature is updated
        'alpha': 5,  # Temperature update factor
        'K': .8  # average cost of sample
    },
}


class ConfigManager:
    def __init__(self, base_config: Dict[str, Any] = CONFIG):
        # Store the default configuration
        self._base_config = copy.deepcopy(base_config)
        self._current_config = copy.deepcopy(base_config)
        self._accessed_keys = set()  # Keep track of accessed keys

    def __deepcopy__(self, memodict={}):
        raise NotImplementedError(
            "Deepcopy is not supported for ConfigManager")

    def update(self, new_config: Dict[str, Any]):
        """Update current configuration with new values"""
        self._deep_update(self._current_config, new_config)
        # print("Updating config")
        # print("Config updated")

    def reset(self):
        """Reset current configuration to the base configuration"""
        self._current_config = copy.deepcopy(self._base_config)
        self._accessed_keys.clear()  # Reset accessed keys

    def get(self) -> Dict[str, Any]:
        """Get the current configuration as a dictionary"""
        return self._current_config

    def clone(self) -> 'ConfigManager':
        """Return a new instance of ConfigManager with the same config"""
        return ConfigManager(self._current_config)

    def _deep_update(self, original: Dict[str, Any], updates: Dict[str, Any]):
        """Helper function to recursively update dictionaries"""
        for key, value in updates.items():
            if isinstance(value, dict) and key in original:
                self._deep_update(original[key], value)
            else:
                original[key] = value

    def save_to_file(self, filepath: str):
        """Save the current configuration to a file (as JSON)"""
        import json
        with open(filepath, 'w') as f:
            json.dump(self._current_config, f, indent=4)

    def load_from_file(self, filepath: str):
        """Load configuration from a file and update the current config"""
        import json
        with open(filepath, 'r') as f:
            new_config = json.load(f)
            self.update(new_config)

    def __setattr__(self, key, value):
        if key in self.__dict__:
            return
        super().__setattr__(key, value)

    # Dot notation access for configuration keys
    def __getattr__(self, key: str) -> Any:
        # if key in self._current_config:
        #     self._accessed_keys.add(key)
        #     return self._current_config[key]
        # raise AttributeError(f"'ConfigManager' object has no attribute '{key}'")
        # Access internal attributes safely
        if '_current_config' in self.__dict__ and key in self._current_config:
            self._accessed_keys.add(key)
            return self._current_config[key]
        raise AttributeError(
            f"'ConfigManager' object has no attribute '{key}'")

    def __setattr__(self, key: str, value: Any):
        # if key in ('_base_config', '_current_config', '_accessed_keys'):  # Internal attributes
        #     super().__setattr__(key, value)
        # else:
        #     self._current_config[key] = value
        # Handle internal attributes safely
        if key in ('_base_config', '_current_config', '_accessed_keys'):
            self.__dict__[key] = value
        else:
            self._current_config[key] = value

    def __getitem__(self, key: str) -> Any:
        """Allow dict-style access"""
        self._accessed_keys.add(key)
        return self._current_config[key]

    def __setitem__(self, key: str, value: Any):
        """Allow dict-style setting"""
        self._current_config[key] = value

    # Check unused configuration values
    def check_unused(self):
        """Warn if any config values were not accessed"""
        unused_keys = set(self._current_config.keys()) - self._accessed_keys
        if unused_keys:
            unused = "\n".join(
                f"  - {key}: {self._current_config[key]}" for key in unused_keys)
            warnings.warn(
                f"The following configuration values were not used: \n{unused}")
