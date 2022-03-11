# Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

## Introduction

Below I address the required rubric points and explain how they were handled in code.

## Explanation of the Starter Code

### 1. Explanation of the functionality provided in `motion_planning.py` and `planning_utils.py`

The provided scripts contain a basic planning implementation that:

a. Provides new functions in planning_utils module:
- `create_grid()`: loads obstacle shapes from `colliders.csv` and fills grid with 1's based on the shape and given path altitude, and safety margin.
- `a_star()`: implements the A-star path search algorithm over the obstacle grid based the the provided action costs and heuristic.
- `heuristic()`: defines admissable heuristic function a parameter to the a_star search algorithm.

b. Introduces `PLANNING` state to the mission, and

c. While in the planning state performs:
- Load colliders map csv file which models obstacles as rectangular prisms with center and half sizes.
- Creates grid representation of obstructions(using 5m Altitude, and 5m safety distance).
- Sets start point to grid centre and sets relative goal at 10m north, 10m east.
- Uses a-star algo to find a minimum cost path through grid from start to goal.
- Converts grid path to waypoints and adds to the mission for drone to follow.
- Transition to `TAKEOFF` state.

## Implementing My Path Planning Algorithm

### 1. Setting the global home position
Here the code is modified to read the first line of the colliders csv file, extract `lat0` and `lon0` as floating point values and use the `self.set_home_position()` method to set global home. This was accomplished in the code as follows:

- Added method `load_home_position()` to `MotionPlanning` class.
- It uses regular expression matching to parse first line and extract `lat0`, `lon0` and return as ECEF coordinates.

### 2. Setting the current local position
Here the code was modified to determine flyer's local position relative to global home. This was accomplished as follows:

- Used the helper function `global_to_local()` from the `udacidrone.frame_utils` package to convert the drones global position in ECEF to relative local position in NED coordinates.
- This code was added to the method `plan_path_transition()` in MotionPlanning class.

### 3. Set grid start position from local position
Flexibility was added to the start location. It was implemented as follows:

- Added helper function `local_to_grid()` to planning_utils module, which offsets the local position in NED coordinates relative to the centre of the configuration space grid.
- Used this to set grid_start variable in `plan_path_transition()`.

### 4. Set grid goal position from geodetic coords
Flexibility  was added to the desired goal location. The requirement was that you should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid. This was implemented as outlined below:

- Added two new command line arguments: `--goal-lat, --goal-lon`.`
- These are converted using `global_to_local()` then `local_to_grid()` functions and used to set `grid_goal` in the `plan_path_transition()` method.

### 5. Modify A* to include diagonal motion (or replace A* altogether)
Here the requirement was to modify the code in `planning_utils()` to update the A* implementation to include diagonal motions on the grid that have a cost of `sqrt(2)`. Below I explain the code used to accomplish this step.

- Changes made in `planning_utils` package.
- Added 4 extra enum values to the `Action` enum for the diagonal movements, and set cost to `sqrt(2)`.
- Updated `valid_actions()` function to filter the 8 possible action values down to the set of  unobstructed actions that stay within the configuration space boundaries.

### 6. Cull waypoints
To address this rubric the requirement was to choose and implement an algorithm to prune the planned path of unnecessary waypoints. Below I explain the code you used to accomplish this step:

a. Added functions to `planning_utils` package.
- `prune_path()` - first tests longest ray path from first to last point of planned path, if this is unobstructed then removes all intermediate waypoints and returns first and last point, if not then it tries from first to next to last point on path and so until clear path is found. If any unpruned path remains pruning moves to start of that segment and repeats from there until end of path reached.
- `is_ray_path_obstructed()` - checks for any obstructions on the grid between points p0,p1.
- `generate_line()` - generates grid points lying on the 2D line p0,p1 using the Bresenham algorithm.

