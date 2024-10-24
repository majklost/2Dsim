import argparse
import pygame

from deform_plan.viewers.PM_replayable_viewer import PMReplayableViewer


def one_time_draw(sim, canvas, additional_data):
    for i, g in enumerate(additional_data["goal_points"]):
        if i in additional_data["config"]["CONTROL_IDXS"]:
            pygame.draw.circle(canvas, (255, 0, 0), g, 5)
        else:
            pygame.draw.circle(canvas, (0, 0, 255), g, 5)
    for n in additional_data["nodes"]:
        for p in n:
            pygame.draw.circle(canvas, (0, 0, 255), p, 5)
        for i in range(len(n) - 1):
            pygame.draw.line(canvas, (0, 0, 0), n[i], n[i + 1], 2)

cnt = 0

def draw(sim,canvas,additional_data):
    global cnt
    cnt += 1
    font = pygame.font.Font(pygame.font.get_default_font(), 36)
    text = font.render(f"Frame: {cnt}", True, (0, 0, 0))
    canvas.blit(text, (10, 10))

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
    rv = PMReplayableViewer(args.path, True)
    rv.one_time_draw = one_time_draw
    rv.drawing_fnc = draw
    rv.show(False)
