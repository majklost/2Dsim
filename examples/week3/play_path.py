from deform_plan.viewers.PM_replayable_exporter import PMReplayableExporter
from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer


def replay_week_cable():
    rv = PMReplayableViewer("./data/cable_rrt",True)
    g = rv.goal
    for p in g.main_points:
        rv.draw_circle(p, 5, (0, 0, 255))
    rv.show(False)


def save_week_cable():
    rv = PMReplayableExporter("./data/cable_rrt")
    # rv.render("./data/")
    rv.save_all_zip("./saves/")

if __name__ == '__main__':
    replay_week_cable()
    # save_week_cable()