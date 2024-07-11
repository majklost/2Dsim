## What to do
- [ ] write local planner for static scene
  - [x] find how to check collision between two kinematics objects
- [ ] prepare some template pipeline for quick experiments not to reinvent wheel
- [ ] write RRT for static scene
- [ ] refactoring
- [ ] write local planner for moving obstacles
- [ ] write RRT for moving obstacles
- [ ] try DynamicRRT
- [ ] separation
  - [ ] model (each map for testing, motion planning algorithms) 
  - [ ] view (rendering - pygame - with/without RRT) 
  - [ ] controller (pymunk - collision detection)
- [ ] change debug draw to standard pygame draw in some organized fashion

## Thoughts
- Scope of Pygame must end in view (only callbacks from UI can be passed)
- Scope of Pymunk should end in local planning RRT must be independent of it
- Create copy of pyMunk space vs. create dummy player that tries to make a path vs. shape_query (Now using shape query)?