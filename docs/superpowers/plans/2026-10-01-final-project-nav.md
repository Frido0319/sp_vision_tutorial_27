# Navigation Final Project Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a reproducible `final_project_nav` branch that completes the official one-click maze navigation assignment and records objective accuracy, timing, and path-tracking evidence.

**Architecture:** Keep the supplied A* planner, behavior trees, controller server, and simulator unchanged. Add a tested planar path-tracking core behind the provided `PidController` plugin, complete the ROS parameters, and validate it through unit, plugin, bounded headless, and visible RViz runs in Ubuntu 22.04 with ROS2 Humble.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, C++17, rclcpp, pluginlib, GTest, Python 3.10, rclpy, Docker with X11, RViz2, pygame.

## Global Constraints

- Official baseline is `TongjiSuperPower/sp_vision_tutorial_27@414a818` on `final_project_nav`.
- Simulator implementation and simulator parameters remain byte-for-byte unchanged.
- Evaluation uses start `(0.900, 0.900)` and exactly one goal `(14.100, 14.100)`.
- No manual `cmd_vel`, start dragging, second goal, or intermediate goal is allowed.
- The required interface remains `sp_controller_server::ControllerPlugin` with `configure`, `setPlan`, and `computeVelocityCommands`.
- Runtime target is Ubuntu 22.04, ROS2 Humble, and Python 3.10.
- Submission deadline is 2026-10-25.
- Build, install, log, screenshots, and generated metrics do not enter the submission history.
- Production behavior follows test-first red-green-refactor development.

---

### Task 1: Reproducible Ubuntu 22.04 ROS2 Humble environment

**Files:**
- Create: `docker/Dockerfile.humble`
- Create: `bash/nav_dev_container.sh`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: repository root mounted at `/workspace`.
- Produces: image `sp-nav-humble:2026-10-01` and a shell/X11 entry point that preserves the host source tree while placing `build`, `install`, and `log` under the 50 GB worktree.

- [ ] **Step 1: Record the untouched baseline**

Run:

```bash
git status --short --branch
git rev-parse HEAD
git diff --quiet origin/final_project_nav -- . ':!docs/superpowers/**'
```

Expected: clean source baseline apart from committed design/plan documents; parent history contains `414a818`.

- [ ] **Step 2: Add the container definition**

Create `docker/Dockerfile.humble` from `ros:humble-ros-base-jammy`. Install the exact assignment dependencies plus testing and GUI packages:

```dockerfile
FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep python3-pip \
    ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs ros-humble-std-srvs ros-humble-rviz2 \
    ros-humble-behaviortree-cpp-v3 qtbase5-dev libopencv-dev libyaml-cpp-dev \
    ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    xvfb x11-apps git && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pygame numpy Pillow PyYAML
WORKDIR /workspace
CMD ["bash"]
```

- [ ] **Step 3: Add the container launcher**

`bash/nav_dev_container.sh` must support `build`, `shell`, and `gui` modes. It must use `--network host`, bind the repository to `/workspace`, pass `DISPLAY`, bind `/tmp/.X11-unix` only in GUI mode, source `/opt/ros/humble/setup.bash`, and never run privileged or mount host devices.

- [ ] **Step 4: Ignore generated outputs**

Add only these project outputs to `.gitignore`:

```gitignore
build/
install/
log/
artifacts/
```

- [ ] **Step 5: Build the image and baseline workspace**

Run:

```bash
docker build -f docker/Dockerfile.humble -t sp-nav-humble:2026-10-01 .
bash/nav_dev_container.sh build -- colcon build --symlink-install
```

Expected: all official packages compile before controller behavior changes.

- [ ] **Step 6: Commit the environment**

```bash
git add docker/Dockerfile.humble bash/nav_dev_container.sh .gitignore
git commit -m "build: add ROS2 Humble navigation environment"
```

---

### Task 2: Complete and test navigation configuration

**Files:**
- Create: `tests/test_nav_config.py`
- Modify: `src/sp_nav_bringup/config/nav_params.yaml`

**Interfaces:**
- Consumes: topic and frame names from the supplied simulator and servers.
- Produces: a complete ROS parameter file accepted by every node.

- [ ] **Step 1: Write the failing configuration test**

The test loads YAML and asserts these exact required values:

```python
EXPECTED = {
    ('planner_server', 'costmap_topic'): '/global_costmap',
    ('planner_server', 'local_costmap_topic'): '/local_costmap/costmap',
    ('planner_server', 'path_topic'): '/global_path',
    ('planner_server', 'plugin_name'): 'AStar',
    ('planner_server', 'plugin_type'): 'sp_global_planner/AStarPlanner',
    ('controller_server', 'local_path_topic'): '/global_path',
    ('controller_server', 'odom_topic'): '/Odometry',
    ('controller_server', 'odom_frame_id'): 'lidar_odom',
    ('controller_server', 'map_frame_id'): 'map',
    ('controller_server', 'cmd_vel_topic'): '/sentry/cmd_vel',
    ('controller_server', 'base_frame_id'): 'base_link',
    ('controller_server', 'plugin_name'): 'PidController',
    ('controller_server', 'plugin_type'): 'PidController',
}
```

It also rejects `None`, empty strings, non-positive controller frequency, and a map other than `sp_nav_bringup/map/maze_map.yaml`.

- [ ] **Step 2: Verify the test fails on the official skeleton**

Run:

```bash
bash/nav_dev_container.sh build -- python3 tests/test_nav_config.py
```

Expected: FAIL because official parameters are empty.

- [ ] **Step 3: Fill the parameter file**

Use the exact topics and frames above. Start with:

```yaml
AStar:
  lethal_cost: 90
  cost_weight: 5.0

esdf_map_publisher:
  ros__parameters:
    map_yaml: "sp_nav_bringup/map/maze_map.yaml"
    frame_id: "map"
    publish_rate_hz: 5.0
    unknown_as_obstacle: false
    robot_radius: 0.25
    margin: 0.05
    d_safe: 0.70
    w: 12.0
    unknown_cost: 100
    visualize_esdf: false
```

Add controller parameters under `PidController`: `kp`, `ki`, `kd`, `integral_limit`, `lookahead_distance`, `max_speed`, `max_acceleration`, `corner_slowdown_angle`, `corner_speed_ratio`, `goal_slowdown_distance`, and `goal_tolerance`. Initial values must remain below simulator limits `v_max=2.0` and `a_max=2.0`.

- [ ] **Step 4: Verify configuration tests pass**

Run:

```bash
bash/nav_dev_container.sh build -- python3 tests/test_nav_config.py
```

Expected: PASS with every required node value present.

- [ ] **Step 5: Commit configuration**

```bash
git add tests/test_nav_config.py src/sp_nav_bringup/config/nav_params.yaml
git commit -m "config: connect maze navigation stack"
```

---

### Task 3: Test-driven planar path tracker

**Files:**
- Create: `src/sp_controller_server/plugins/path_tracker.hpp`
- Create: `src/sp_controller_server/plugins/path_tracker.cpp`
- Create: `src/sp_controller_server/test/test_path_tracker.cpp`
- Modify: `src/sp_controller_server/CMakeLists.txt`

**Interfaces:**
- Produces: `pid_controller::PathTracker`.
- Consumes: path points, current map position, current map velocity, and `dt`.

The public API is fixed before implementation:

```cpp
struct Point2D { double x; double y; };
struct Velocity2D { double x; double y; };
struct TrackerParams {
  double kp, ki, kd;
  double integral_limit;
  double lookahead_distance;
  double max_speed;
  double max_acceleration;
  double corner_slowdown_angle;
  double corner_speed_ratio;
  double goal_slowdown_distance;
  double goal_tolerance;
};
struct TrackerOutput {
  Velocity2D command;
  std::size_t nearest_index;
  bool goal_reached;
};

class PathTracker {
public:
  explicit PathTracker(TrackerParams params);
  void setPath(std::vector<Point2D> path);
  TrackerOutput step(Point2D position, Velocity2D velocity, double dt);
  const std::vector<Point2D> & path() const;
};
```

- [ ] **Step 1: Add failing tests for path ownership and progress**

Tests must prove that duplicate consecutive points are removed, replacing a path resets progress, and the nearest index never moves backward on noisy positions.

- [ ] **Step 2: Run the focused test target and verify RED**

Run:

```bash
bash/nav_dev_container.sh build -- colcon test --packages-select sp_controller_server \
  --ctest-args -R path_tracker --output-on-failure
```

Expected: compilation or assertion failure because `PathTracker` is not implemented.

- [ ] **Step 3: Implement path storage and monotonic nearest search**

Implement only enough behavior for the first tests. Reject non-finite points when storing the path and leave an empty sanitized path when no valid segment exists.

- [ ] **Step 4: Verify GREEN for path ownership and progress**

Run the focused target again. Expected: PASS.

- [ ] **Step 5: Add failing tests for lookahead and goal stopping**

Cover a straight path, a 90-degree corner, a point within goal tolerance, an empty path, and non-finite pose/velocity input. Empty, invalid, and reached-goal cases must return exact zero velocity.

- [ ] **Step 6: Verify RED, then implement lookahead and stopping**

The lookahead search walks forward by accumulated polyline distance. It does not invent a shortcut segment or modify the stored path.

- [ ] **Step 7: Add failing tests for speed, acceleration, and corner limits**

Assertions:

```cpp
EXPECT_LE(std::hypot(out.command.x, out.command.y), params.max_speed + 1e-9);
EXPECT_LE(std::hypot(out2.command.x - out1.command.x,
                     out2.command.y - out1.command.y),
          params.max_acceleration * dt + 1e-9);
EXPECT_LT(corner_speed, straight_speed);
```

- [ ] **Step 8: Verify RED, then implement bounded PID motion**

Use map-frame vector control:

```text
command = feed_forward(path tangent, scheduled speed)
        + kp * position_error
        + ki * clamped_integral
        - kd * measured_velocity
```

Clamp the integral per axis, clamp vector magnitude, then clamp vector change. Slow down near the goal and when the angle between adjacent path segments exceeds `corner_slowdown_angle`.

- [ ] **Step 9: Run all tracker tests and refactor with tests green**

Expected: all cases pass with no warnings or non-finite outputs.

- [ ] **Step 10: Commit the tracker core**

```bash
git add src/sp_controller_server/plugins/path_tracker.* \
  src/sp_controller_server/test/test_path_tracker.cpp \
  src/sp_controller_server/CMakeLists.txt
git commit -m "feat: add bounded maze path tracker"
```

---

### Task 4: Connect the tracker to the plugin and prove plugin loading

**Files:**
- Modify: `src/sp_controller_server/plugins/pid_controller.hpp`
- Modify: `src/sp_controller_server/plugins/pid_controller.cpp`
- Create: `src/sp_controller_server/test/test_pid_plugin.cpp`
- Modify: `src/sp_controller_server/CMakeLists.txt`

**Interfaces:**
- Consumes: `PathTracker`, ROS parameters under `PidController.*`, `nav_msgs::msg::Path`, current pose, and current map velocity.
- Produces: `geometry_msgs::msg::TwistStamped` with finite `base_link` planar velocity derived from the map-frame tracker command.

- [ ] **Step 1: Write a failing plugin load/configuration test**

Create an rclcpp node with all controller parameters, instantiate the plugin through:

```cpp
pluginlib::ClassLoader<sp_controller_server::ControllerPlugin> loader(
  "sp_controller_server", "sp_controller_server::ControllerPlugin");
auto controller = loader.createSharedInstance("PidController");
```

The test must call `configure`, send a simple path through `setPlan`, call `computeVelocityCommands`, and assert finite non-zero planar motion toward the target.

- [ ] **Step 2: Run the plugin test and verify RED**

Expected: the skeleton returns zero command or fails because tracking parameters are not read.

- [ ] **Step 3: Implement parameter loading and validation**

`configure()` must load every `TrackerParams` field with `require_param`, reject non-positive limits/tolerances, and construct the tracker. No silent fallback values are allowed.

- [ ] **Step 4: Implement `setPlan` and `computeVelocityCommands`**

Convert ROS path points into `Point2D`, calculate `dt` from the steady clock with a bounded initial value `1/control_frequency`, and call `PathTracker::step`. Extract the current `base_link` yaw from `pose.pose.orientation`, rotate the map-frame tracker command into `base_link`, and return:

```cpp
cmd_vel.header.stamp = node_->now();
const double c = std::cos(yaw);
const double s = std::sin(yaw);
cmd_vel.header.frame_id = base_frame_id_;
cmd_vel.twist.linear.x = c * output.command.x + s * output.command.y;
cmd_vel.twist.linear.y = -s * output.command.x + c * output.command.y;
cmd_vel.twist.angular.z = 0.0;
```

- [ ] **Step 5: Run plugin and tracker tests**

Expected: pluginlib loads the class and all focused tests pass.

- [ ] **Step 6: Run a full clean build and test result check**

```bash
bash/nav_dev_container.sh build -- bash -lc '
  rm -rf build install log &&
  colcon build --symlink-install &&
  colcon test --event-handlers console_direct+ &&
  colcon test-result --verbose'
```

Expected: zero failed packages and zero failed tests.

- [ ] **Step 7: Commit plugin integration**

```bash
git add src/sp_controller_server
git commit -m "feat: implement navigation controller plugin"
```

---

### Task 5: Bounded one-click evaluator

**Files:**
- Create: `tools/nav_metrics.py`
- Create: `tools/evaluate_nav.py`
- Create: `tests/test_nav_metrics.py`
- Create: `bash/run_nav_evaluation.sh`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: `/Odometry`, `/global_path`, `/sentry/cmd_vel`, and one published `/goal_pose`.
- Produces: `artifacts/nav-evaluation.json`, process exit 0 on arrival, and non-zero on timeout/collision/startup failure.

- [ ] **Step 1: Write failing metric tests**

Test exact Euclidean goal distance and minimum point-to-segment cross-track distance. Include horizontal, vertical, diagonal, empty-path, and one-point-path cases.

- [ ] **Step 2: Verify metric tests fail**

Run:

```bash
bash/nav_dev_container.sh build -- python3 -m unittest tests/test_nav_metrics.py -v
```

- [ ] **Step 3: Implement pure metric helpers and verify GREEN**

`tools/nav_metrics.py` contains no ROS imports so unit tests remain fast and deterministic.

- [ ] **Step 4: Write the ROS evaluator**

The evaluator must:

1. wait for odometry, path publisher, and controller subscriber readiness;
2. verify the first pose is within `0.15 m` of `(0.900, 0.900)`;
3. publish exactly one `PoseStamped` goal at `(14.100, 14.100)` in `map`;
4. record elapsed time, final error, sampled mean/max cross-track error, command magnitude, and commanded-but-stationary intervals as a wall-contact proxy;
5. declare success after the robot stays within `0.3 m` for five consecutive odometry samples;
6. time out after a configurable bounded duration;
7. write atomic JSON output and exit.

- [ ] **Step 5: Add the process orchestration script**

`bash/run_nav_evaluation.sh` starts the static TF, navigation launch without RViz, simulator, and evaluator as separate process groups. It uses `trap` to send SIGINT and wait for all children on success, failure, or interruption. It never publishes `cmd_vel`.

- [ ] **Step 6: Verify the evaluator on a controlled metric-only run**

Run unit tests and `bash -n bash/run_nav_evaluation.sh`. Expected: PASS.

- [ ] **Step 7: Commit evaluation tooling**

```bash
git add tools/nav_metrics.py tools/evaluate_nav.py tests/test_nav_metrics.py \
  bash/run_nav_evaluation.sh .gitignore
git commit -m "test: add one-click navigation evaluator"
```

---

### Task 6: End-to-end tuning and visible acceptance

**Files:**
- Modify: `src/sp_nav_bringup/config/nav_params.yaml`
- Modify only if required for visibility: `src/sp_nav_bringup/rviz/rviz.rviz`
- Create generated evidence under ignored `artifacts/`

**Interfaces:**
- Consumes: completed controller, official maze, fixed one-click evaluator.
- Produces: repeatable arrival metrics and visible RViz evidence.

- [ ] **Step 1: Run the first bounded end-to-end evaluation**

```bash
bash/nav_dev_container.sh build -- bash/run_nav_evaluation.sh --timeout 180
```

Expected first-run outcome: a measured pass or a specific bounded failure with logs. No unbounded background process may remain.

- [ ] **Step 2: For each failure, add a reproducing test before changing code**

Controller logic failures require a new failing GTest. Parameter-only tuning changes require preserving the same evaluator goal and writing the before/after metrics into the commit message or README notes.

- [ ] **Step 3: Tune only declared parameters**

Tune `kp`, `ki`, `kd`, lookahead, maximum speed, acceleration, corner speed ratio, and goal slowdown. Do not edit simulator files, start pose, goal, collision radius, or timeout to manufacture a pass.

- [ ] **Step 4: Require three consecutive successful automated runs**

Each run must satisfy arrival within `0.3 m`, no timeout, no non-finite command, and no prolonged commanded-but-stationary interval. Preserve all three JSON files outside Git. The later visible run remains the authoritative wall-contact check because the simulator publishes no collision topic.

- [ ] **Step 5: Run visible GUI acceptance**

Use:

```bash
xhost +SI:localuser:root
bash/nav_dev_container.sh gui -- bash/sim_nav_all_start.sh
```

Verify rendered RViz and simulator windows show the global costmap, global path, robot pose, and the single fixed goal. Capture screenshots only as ignored evidence.

- [ ] **Step 6: Commit final parameter and RViz changes**

```bash
git add src/sp_nav_bringup/config/nav_params.yaml src/sp_nav_bringup/rviz/rviz.rviz
git commit -m "perf: tune maze path tracking"
```

---

### Task 7: Submission documentation and final verification

**Files:**
- Modify: `README.md`
- Create: `docs/navigation-controller.md`

**Interfaces:**
- Consumes: final commands, architecture, parameters, and measured metrics.
- Produces: a reviewer-facing submission with no machine-specific absolute paths.

- [ ] **Step 1: Write the README sections**

Include environment, build, launch, one-click evaluation, controller data flow, parameter table, safety behavior, RViz topics, measured results, known limitations, and repository structure. Clearly distinguish automated container evidence from any later course-computer run.

- [ ] **Step 2: Add the controller explanation**

`docs/navigation-controller.md` explains nearest-index monotonicity, lookahead, PID/feed-forward terms, corner and goal slowdown, vector saturation, acceleration limiting, and the required map-to-`base_link` velocity rotation for the supplied simulator.

- [ ] **Step 3: Run documentation and repository checks**

```bash
rg -n '/home/|/tmp/|ubuntu20|TODO|TBD' README.md docs src tests tools bash
git diff --check origin/final_project_nav...HEAD
git status --short
```

Expected: no private absolute paths, no unresolved assignment TODO in student-owned files, no whitespace errors, and no untracked outputs.

- [ ] **Step 4: Perform a fresh final build and test**

Delete only ignored `build`, `install`, and `log` inside the isolated worktree, then rerun clean build, unit/plugin tests, and three bounded evaluations.

- [ ] **Step 5: Commit documentation**

```bash
git add README.md docs/navigation-controller.md
git commit -m "docs: document navigation final project"
```

- [ ] **Step 6: Compare and push the submission branch**

Verify commit history and tree scope, then push:

```bash
git log --oneline origin/final_project_nav..HEAD
git diff --stat origin/final_project_nav...HEAD
git push submission HEAD:final_project_nav
git ls-remote submission refs/heads/final_project_nav
```

Expected: remote hash equals local `HEAD` and GitHub contains complete buildable source.
