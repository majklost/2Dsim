#similar to cable test but now with foam instead of cable
from z_old_tests.TestTemplate import TestTemplate
from _old_src.helpers.objectLibrary import Obstacle, RandomBlock
from _old_src.helpers.foams import Foam
from _old_src.controls.keyControls import KeyControls
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
# MASS_PER_LENGTH = .07
FOAM_STRUCTURAL_PARAMS = Foam.SpringParams(2000, 500)
FOAM_BENDING_PARAMS = Foam.SpringParams(300, 50)
OBSTACLES = True
MOVING_FORCE = 40000

class FoamTest(TestTemplate):
    def __init__(self):
        super().__init__(800, 800, 80)
        self.space.damping = .3
        self.suspicious = 0
        self.prev_vel = 0

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

    def pre_render(self):

        self.kk.solve_keys(self.keys,self.keydowns,self.click)
        cur = self.kk.objects[self.kk.current]

        if cur.force.length > 1000:
            seg__vel_sum = sum([s.velocity.length for s in self.kk.objects],0)
            if seg__vel_sum <1000 and seg__vel_sum < self.prev_vel:
                print("blocked")
            self.prev_vel = seg__vel_sum




if __name__ == "__main__":
    scene = FoamTest()
    scene.run()