

from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer


def replay_velocity():
    rv = PMReplayableViewer("data/good_velocity.rpath")




    def draw_path(rv_obj):




        for i in range(len(rv_obj.path) - 1):
            rv_obj.draw_line(rv_obj.path[i].exporter_data["pos"], rv_obj.path[i + 1].exporter_data["pos"], (255, 0, 0))
        for p in rv_obj.path:
            print(p)
            rv_obj.draw_circle(p.exporter_data["pos"], 5, (0, 255, 0))
    draw_path(rv)

    rv.show(True)

def replay_cable():
    rv = PMReplayableViewer("data/cable.rpath")
    if rv.additional_data is not None and "time" in rv.additional_data:
        print("Time: ", rv.additional_data["time"])
    def draw_path(rv_obj):

        pts = rv.goal.points
        print("Goal: ", pts)
        for p in pts:

            rv_obj.draw_circle(p, 5, (0, 0, 255))
        # for i in range(len(rv_obj.path) - 1):
        #     rv_obj.draw_line(rv_obj.path[i].exporter_data["pos"], rv_obj.path[i + 1].exporter_data["pos"], (255, 0, 0))
        # for p in rv_obj.path:
        #     print(p)
        #     for s in p.exporter_data["points"]:
        #         rv_obj.draw_circle(s, 5, (0, 255, 0))
            # rv_obj.draw_circle(p.exporter_data["pos"], 5, (0, 255, 0))
    draw_path(rv)

    rv.show(False)



if __name__ == '__main__':
    replay_cable()
    # replay_velocity()