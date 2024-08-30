import pymunk
#creates a region (initially a rectangle)
#it can track how many masspoints are in the region

class GoalSpecifier:
    def __init__(self, x, y, w, h,space:pymunk.Space, required_in_count=1):
        region_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        self.region = pymunk.Poly.create_box(region_body, (w, h))
        self.region.body.position = x, y
        self.region.sensor = True
        self.region.color = (0, 255, 0, 20)
        self.region.collision_type = 3
        self.space = space
        self.space.add(region_body,self.region)
        self.in_count = 0
        self._add_handler()
        self.required_in_count = required_in_count
        print("Goal Specified, required in count: ", required_in_count)



    def _add_handler(self):
        self.space.add_collision_handler(1, 3).begin = self._on_enter
        self.space.add_collision_handler(1, 3).separate = self._on_exit

    def _on_enter(self, arbiter, space, data):
        self.in_count += 1
        print("Entered Goal, now in goal: ", self.in_count)
        return False

    def _on_exit(self, arbiter, space, data):
        self.in_count -= 1
        print("Exited Goal, now in goal: ", self.in_count)
        return False

    def is_complete(self):
        b= self.in_count >= self.required_in_count
        if b:
            print("Goal Complete")
        return b