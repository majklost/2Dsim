#similar to cable test but now with foam instead of cable
from tests.TestTemplate import TestTemplate
from src.helpers.objectLibrary import Obstacle, RandomBlock
from src.helpers.foams import Foam
from src.controls.keyControls import KeyControls
import random

#test movement of foam between static obstacles

#for obstacles
BLOCK_RADIUS = 75
BLOCK_ROWS = 3
BLOCK_COLUMNS = 3
SEED_SEQ_INIT = 20
FOAM_WIDTH=200
FOAM_HEIGHT=250
MASS_PER_LENGTH = .06
FOAM_STRUCTURAL_PARAMS = Foam.SpringParams(2000, 2000)
FOAM_BENDING_PARAMS = Foam.SpringParams(300, 50)
OBSTACLES = True
MOVING_FORCE = 20000

class FoamTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .3

    def setup(self):
        self.draw_constraints = False
        random.seed(SEED_SEQ_INIT)
        blocks = []
        #add obstacles
        for i in range(BLOCK_COLUMNS):
            for j in range(BLOCK_ROWS):
                block = RandomBlock(100 + 300 * i, 350 + 200 * j, BLOCK_RADIUS,random.randint(0,1000))
                if OBSTACLES:
                    block.add(self.space)
                blocks.append(block)

        end_platform = Obstacle(400, 800, 800, 100)
        end_platform.add(self.space)

        #foam
        self.foam = Foam(100, 10, FOAM_WIDTH, FOAM_HEIGHT, MASS_PER_LENGTH, FOAM_STRUCTURAL_PARAMS, FOAM_BENDING_PARAMS)
        self.foam.add(self.space)
        self.kk = KeyControls(self.space,sum(self.foam.segments,[]),MOVING_FORCE,self.display)
        self.prev_vel = self.foam.segments[self.kk.current][0].velocity

    def pre_render(self):
        # self.foam.segments_shapes[self.kk.current][0].color = (0, 0, 255, 255)
        if self.click:
            print("click here")
        self.kk.solve_keys(self.keys,self.keydowns,self.click)
        # self.foam.segments_shapes[self.kk.current][0].color = (255, 0, 0, 255)


if __name__ == "__main__":
    scene = FoamTest()
    scene.run()