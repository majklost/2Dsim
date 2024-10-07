from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer


def replay_week1_cable():
    rv = PMReplayableViewer("./data/cable_rrt",True)
    g = rv.goal
    for p in g.main_points:
        rv.draw_circle(p, 5, (0, 0, 255))
    rv.show(True)



if __name__ == '__main__':
    replay_week1_cable()
