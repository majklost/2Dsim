import pymunk

from _old_src.helpers.PygameRenderer import PygameRenderer
from _old_src.helpers.objectLibrary import Cross
FPS = 80

renderer = PygameRenderer(800, 800, FPS)
space = pymunk.Space()
cr = Cross(400,400,20,5)
cr.add(space)
for i in range(2000):
    space.step(1/FPS)
    renderer.update_cur(space)
    if not renderer.want_running:
        break
c2 = Cross(400,600,20,2)
c2.add(space)
for i in range(2000):
    space.step(1/FPS)
    renderer.update_cur(space)
    if not renderer.want_running:
        break
