# Navigation Final Project Design

## Goal

Complete the official `final_project_nav` assignment on Ubuntu 22.04 and ROS2 Humble. The robot must start at `(0.900, 0.900)`, accept exactly one RViz goal at `(14.100, 14.100)`, follow the A* path through the maze without manual `cmd_vel`, and stop within the official arrival tolerance. The submission must include buildable source, RViz visualization, a clear README, and reproducible validation evidence.

## Source of truth

- Official branch: `TongjiSuperPower/sp_vision_tutorial_27@414a818`, branch `final_project_nav`.
- Assignment PDF: `/home/ubuntu20/下载/27算法组导航方向招新大作业.pdf`.
- Target environment: Ubuntu 22.04, ROS2 Humble, Python 3.10.
- Simulator code and parameters remain unchanged.
- The evaluator may set the fixed goal only once. The implementation may not move the start pose, republish a replacement goal, or publish `cmd_vel` outside the controller server.
- Submission deadline: 2026-10-25.

## Delivery scope

### Required work

1. Complete every required value in `src/sp_nav_bringup/config/nav_params.yaml`.
2. Implement all three `ControllerPlugin` methods in `PidController`.
3. Register and load the controller through pluginlib.
4. Make `bash/sim_nav_all_start.sh` start a working one-click navigation stack.
5. Show the global costmap, A* path, robot pose, and fixed goal in RViz.
6. Add automated controller tests and a bounded end-to-end evaluation script.
7. Write a submission README with build, run, evaluation, design, parameters, and measured results.

### Low-risk bonus work

The controller will smooth motion without modifying the simulator or replacing the supplied A* planner. It will use monotonic path progress, a bounded lookahead target, corner-aware speed reduction, velocity saturation, and acceleration limiting. This improves the executed trajectory while preserving the collision-free A* route.

LQR and MPC are outside the first release. They add tuning and model risk without increasing the score ceiling above 100. They may be considered only after the required run is repeatably successful and the measured score leaves a clear gap.

## Architecture

### Configuration

`nav_params.yaml` connects the supplied nodes with the topics and frames already used by the simulator:

- planner global map: `/global_costmap`
- planner local map: `/local_costmap/costmap`
- global path: `/global_path`
- odometry: `/Odometry`
- odometry frame: `lidar_odom`
- map frame: `map`
- robot frame: `base_link`
- velocity command: `/sentry/cmd_vel`
- planner plugin instance/type: `AStar` / `sp_global_planner/AStarPlanner`
- controller plugin instance/type: `PidController` / `PidController`
- map URI: `sp_nav_bringup/map/maze_map.yaml`

The map safety values will match the supplied simulator unless a bounded experiment proves a safer value: robot radius `0.25 m`, margin `0.05 m`, safe distance `0.70 m`, and ESDF weight `12.0`.

### Controller state

`PidController` owns only control state:

- latest global path
- monotonic nearest-path index
- two-dimensional integral error with anti-windup
- previous command and timestamp for acceleration limiting
- scalar parameters loaded under `PidController.*`

`setPlan()` replaces the path, removes consecutive duplicate points, resets progress and integral state, and records the path frame. It does not alter the planner output or simulator state.

### Path tracking

For each control cycle:

1. Reject an empty or invalid plan with a zero command.
2. Advance the nearest-path index only forward, preventing progress from jumping backward because of odometry noise.
3. Walk along the polyline from the nearest point to a bounded lookahead target.
4. Compute the map-frame position error from the current pose to the lookahead target.
5. Combine proportional position feedback, bounded integral correction, velocity damping from the supplied map-frame velocity, and a feed-forward velocity along the path tangent.
6. Reduce target speed near sharp turns and near the final goal.
7. Clamp command magnitude to the configured maximum and clamp command change by the configured acceleration limit.
8. Publish zero velocity once the final Euclidean distance is within the configured stop tolerance.

The tracker computes feedback in the `map` frame because the supplied controller server provides the pose and measured velocity in that frame. The simulator consumes `/sentry/cmd_vel` as `base_link` planar velocity, so the plugin rotates the final map-frame command into `base_link` with the current pose yaw before returning `TwistStamped`. It leaves `angular.z` at zero because the simulated chassis accepts holonomic translation and the assignment scores path tracking rather than heading control.

### Safety and failure behavior

- Empty path, non-finite pose, non-finite velocity, or non-finite intermediate command produces a zero velocity command.
- A newly received plan resets controller history so old integral or acceleration state cannot affect the new task.
- Speed and acceleration limits apply on every cycle.
- The controller never publishes directly; only the supplied controller server publishes `/sentry/cmd_vel`.
- The end-to-end evaluator terminates processes on timeout and records the failure reason instead of leaving background ROS nodes running.
- The solution does not modify files under `src/sp_nav_sim/sp_nav_sim/sim/` or simulator parameters.

## Testing strategy

### Unit tests

Pure path-tracking calculations will live in a small helper module compiled into the controller plugin and a GTest target. Tests cover:

- duplicate-point removal
- monotonic nearest-index advancement
- lookahead selection on straight and turning paths
- zero command for an empty path
- goal stopping behavior
- speed magnitude limiting
- acceleration limiting
- non-finite input rejection
- corner speed reduction
- path replacement state reset

Each production behavior begins with a failing test and follows the red-green-refactor cycle.

### Build and plugin tests

An Ubuntu 22.04 and ROS2 Humble environment will run:

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

A plugin smoke test must instantiate `pid_controller::PidController` through pluginlib and load all required parameters.

### End-to-end evaluation

A bounded evaluator will start the supplied stack, wait for required nodes and topics, publish the fixed goal once, and record:

- arrival success within the official `0.3 m` behavior-tree tolerance
- elapsed time from goal publication to arrival
- final goal distance
- sampled cross-track error to `/global_path`
- maximum and mean command speed
- timeout and a commanded-but-stationary wall-contact proxy

The final acceptance run also uses the visible RViz and simulator windows. The simulator does not publish an authoritative collision topic, so automated stagnation evidence cannot replace visual confirmation that the robot avoids walls and that the global costmap, path, robot, and goal are all legible.

## Environment strategy

The Ubuntu 20.04 host remains unchanged. Navigation development uses a dedicated Ubuntu 22.04 and ROS2 Humble container with the worktree and build directories bind-mounted from the 50 GB volume. X11 forwarding displays RViz, pygame, and terminal windows on the host desktop. The official Python 3.10 simulator library from commit `414a818` is used unchanged.

If graphics or timing inside the container prevents a representative final run, the same branch and commands will be run on the course Ubuntu 22.04 computer after 2026-10-06. A container success is software evidence; it does not replace the optional course-computer check when timing differs materially.

## Repository and submission

- Development branch: `codex/final-project-nav`.
- Submission branch: `final_project_nav` in `Frido0319/sp_vision_tutorial_27`.
- Generated `build/`, `install/`, `log/`, screenshots, and temporary benchmark files remain ignored.
- Source, tests, evaluation scripts, configuration, and README are committed in small reviewable commits.
- The final push occurs only after a fresh build, tests, bounded evaluation, repository cleanliness check, and comparison against official commit `414a818`.

## Acceptance criteria

The work is complete when all of the following hold:

1. A clean Ubuntu 22.04 ROS2 Humble build succeeds.
2. Unit, plugin, and integration tests pass with no failed test results.
3. One goal publication at `(14.100, 14.100)` moves the robot from `(0.900, 0.900)` to within `0.3 m` without manual commands or simulator changes.
4. The robot does not collide with maze walls during the recorded run.
5. The evaluator reports elapsed time, final error, and cross-track error.
6. RViz visibly shows the required map, path, robot, and goal.
7. The README explains the controller, parameters, commands, measured result, and limitations.
8. The final branch contains complete source and no build artifacts.
