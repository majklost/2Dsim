from tests.TestTemplate import TestTemplate
import pymunk
import pymunk.constraints

class FoamPlayground(TestTemplate):

    def setup(self):
        self.draw_constraints = False
