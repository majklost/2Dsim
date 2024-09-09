

class BasePlannerRequest:
    def __init__(self,start,goal):
        self.start = start
        self.goal = goal



class BasePlannerResponse:
    def __init__(self):
        self.checkpoints = []