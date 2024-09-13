import dill

from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer

rv = PMReplayableViewer("./data/velocity.rpath")


def draw_path(self):
    for i in range(len(self.path) - 1):
        self.draw_line(self.path[i].exporter_data["pos"], self.path[i + 1].exporter_data["pos"], (255, 0, 0))
    for p in self.path:
        self.draw_circle(p.exporter_data["pos"], 5, (0, 255, 0))
draw_path(rv)

rv.show(True)
