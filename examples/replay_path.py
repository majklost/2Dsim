from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer

if __name__ == '__main__':
    fpath= input("give path: ")
    v = PMReplayableViewer(fpath)
    v.show()
