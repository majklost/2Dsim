import argparse
import pathlib

from deform_plan.viewers.PM_replayable_exporter import PMReplayableExporter
from play_path import one_time_draw, draw



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Play the path'
    )
    parser.add_argument(
        '-p','--path',
        type=str,
        help='Path to the path file'
    )
    args = parser.parse_args()
    rv = PMReplayableExporter(args.path)
    rv.one_time_draw = one_time_draw
    rv.drawing_fnc = draw
    folder = pathlib.Path(args.path).parent
    rv.render(folder)