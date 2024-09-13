class G:
    start: int
    prd: str

    def __str__(self):
        return f"Start: {self.start}, Prd: {self.prd}"



g1 = G()
g1.start = 1
g1.prd = "Name"

g2 = G()
g2.start = 2
g2.prd = "Name2"


print(g1,g1.start,g1.prd)

from deform_plan.messages.planner_messages import PlannerResponse