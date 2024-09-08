from dataclasses import dataclass
from enum import Enum
import pymunk

@dataclass
class PMConfig:
    gravity: float = 0
    damping: float = .1
    width: int = 800
    height: int = 800
    FPS: int = 60

DYNAMIC = pymunk.Body.DYNAMIC
KINEMATIC = pymunk.Body.KINEMATIC
STATIC = pymunk.Body.STATIC
