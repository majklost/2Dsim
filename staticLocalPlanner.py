#this should yield new nodes to RRT
# we give initial position and goal position to it, planner tries to morph between these two
# if it encounters an obstacle, it stops
# during the process, it adds new nodes to the tree
import pymunk  #uses pymunk for checking validity of path


class LocalPlanner:
    def __init__(self, space: pymunk.Space, agent: pymunk.Shape, goal: pymunk.Shape):
        self.space = space
        #set filtering so that agent does not collide with itself
        agent.filter = pymunk.ShapeFilter(group=1)
        # agent.body.data = "test"
        self.agent = agent.copy()
        self.agent.sensor = True
        self.goal = goal
        # self.gap = gap
        print("Local planner initialized")

    def _query(self):
        print("Querying local planner")
        print("-" * 20)
        print(self.space.shape_query(self.agent))
        print("-" * 20)
        self.check_path()

    def check_path(self):
        # TODO rewrite to trying this directly in simulation if slow or reduce number of checks
        checkpoints = []
        direction = self.goal.body.position - self.agent.body.position
        angle_morph = self.goal.body.angle - self.agent.body.angle
        num_steps = max(int(direction.get_length_sqrd()/1000),100)
        angle_step = angle_morph / num_steps
        print(f"Num steps: {num_steps}")
        for i in range(num_steps):


            self.agent.body.position += direction / num_steps
            self.agent.body.angle += angle_step
            if self.space.shape_query(self.agent):
                print("Path blocked")
                print(checkpoints)
                return checkpoints
            if i % 20 == 0 and i != 0:
                checkpoints.append((self.agent.body.position,self.agent.body.angle))
        print("Path clear")
        print(checkpoints)
        return checkpoints

    def list_bodies(self):
        for b in self.space.bodies:
            print(b)
