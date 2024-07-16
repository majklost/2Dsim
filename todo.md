## What to do
- [x] write local planner for static scene
  - [x] find how to check collision between two kinematics objects
- [x] prepare some template pipeline for quick experiments not to reinvent wheel
- [x] write RRT for static scene
- [x] refactoring
- [ ] write local planner for moving obstacles
- [ ] write RRT for moving obstacles
- [ ] try DynamicRRT
- [ ] separation
  - [ ] model (each map for testing, motion planning algorithms) 
  - [ ] view (rendering - pygame - with/without RRT) 
  - [ ] controller (pymunk - collision detection)
- [ ] change debug draw to standard pygame draw in some organized fashion
- [ ] make RRT more general (support arbitrary DOFs)
## Thoughts
- Scope of Pygame must end in view (only callbacks from UI can be passed)
- Scope of Pymunk should end in local planning RRT must be independent of it
- Create copy of pyMunk space vs. create dummy player that tries to make a path vs. shape_query (Now using shape query)?