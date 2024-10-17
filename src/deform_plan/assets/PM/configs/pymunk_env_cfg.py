from dataclasses import dataclass
import pymunk

@dataclass
class PMConfig:
    gravity: float = 0
    damping: float = .1
    width: int = 800
    height: int = 800
    FPS: int = 60
    collision_slope: float = 0.01
    def __str__(self):
        return f"PMConfig: gravity:{self.gravity}, damping:{self.damping}, width:{self.width}, height:{self.height}, FPS:{self.FPS}, collision_slope:{self.collision_slope}"

DYNAMIC = pymunk.Body.DYNAMIC
KINEMATIC = pymunk.Body.KINEMATIC
STATIC = pymunk.Body.STATIC
