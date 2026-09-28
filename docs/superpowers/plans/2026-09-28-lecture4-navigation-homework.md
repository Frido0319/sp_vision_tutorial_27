# Lecture 4 Navigation Homework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the ROS2 Humble QoS debugger so its endpoints communicate by default, sequence loss is counted correctly, receive frame rate is reported once per second, and the diagnostic reasoning is documented.

**Architecture:** Keep the two supplied nodes and parameterized QoS construction. Correct the publisher's offered reliability, remove the subscriber's intentional default bottleneck while preserving it as a runtime experiment, add accurate gap accounting, and compute rate from a monotonic elapsed interval.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, `rclcpp`, custom `SensorData.msg`, colcon, C++17.

## Global Constraints

- Base the work on the official `nav` branch.
- Modify only the homework implementation, documentation, and verification support.
- Keep publisher and subscriber as separate ROS2 processes.
- Preserve runtime parameters for reliability, depth, rate, and callback delay.
- Build and test against ROS2 Humble on Ubuntu 22.04.
- Store the worktree and build artifacts on `/home/ubuntu20/桌面/project/Linux-50G`.

---

### Task 1: Isolated navigation worktree and red checks

**Files:**
- Worktree: `/home/ubuntu20/桌面/project/Linux-50G/sp-course-worktrees/lecture4-nav`
- Create: `lecture4/homework/scripts/check_homework.py`

**Interfaces:**
- Consumes: `origin/nav` and the two starter C++ files.
- Produces: a local `codex/lecture4-homework` branch plus repeatable source checks.

- [ ] **Step 1: Fetch the official navigation branch and create the worktree**

```bash
git fetch origin nav
git worktree add /home/ubuntu20/桌面/project/Linux-50G/sp-course-worktrees/lecture4-nav -b codex/lecture4-homework origin/nav
```

- [ ] **Step 2: Add checks for all three requested behaviors**

The script must require:

```python
assert 'declare_parameter("reliability", "reliable")' in publisher
assert 'declare_parameter("callback_delay_ms", 0)' in subscriber
assert 'lost_count_ += lost;' in subscriber
assert 'received_count_ - last_received_count_' in subscriber
assert 'std::chrono::duration<double>' in subscriber
```

- [ ] **Step 3: Run the checks against the starter**

Run: `python3 lecture4/homework/scripts/check_homework.py`

Expected: exit 1 and failures for all required corrections.

### Task 2: Compatible default QoS

**Files:**
- Modify: `lecture4/homework/src/qos_debugger/src/qos_debugger_pub.cpp`
- Test: `lecture4/homework/scripts/check_homework.py`

**Interfaces:**
- Consumes: publisher `reliability` parameter.
- Produces: a reliable publisher compatible with the starter subscriber.

- [ ] **Step 1: Change the publisher default reliability**

```cpp
this->declare_parameter("reliability", "reliable");
```

- [ ] **Step 2: Run the focused source check**

Run: `python3 lecture4/homework/scripts/check_homework.py --task qos`

Expected: the QoS compatibility check passes.

- [ ] **Step 3: Commit the isolated fix**

```bash
git add lecture4/homework/src/qos_debugger/src/qos_debugger_pub.cpp lecture4/homework/scripts/check_homework.py
git commit -m "fix: align navigation homework QoS defaults"
```

### Task 3: Subscriber loss behavior and accounting

**Files:**
- Modify: `lecture4/homework/src/qos_debugger/src/qos_debugger_sub.cpp`
- Test: `lecture4/homework/scripts/check_homework.py`

**Interfaces:**
- Consumes: message sequence numbers and configurable callback delay.
- Produces: a zero-delay normal mode and accurate cumulative loss count.

- [ ] **Step 1: Remove the intentional default bottleneck**

```cpp
this->declare_parameter("callback_delay_ms", 0);
```

- [ ] **Step 2: Add every positive sequence gap to the cumulative counter**

```cpp
const uint32_t lost = msg->seq - expected_seq_;
lost_count_ += lost;
```

- [ ] **Step 3: Run the focused source checks**

Run: `python3 lecture4/homework/scripts/check_homework.py --task loss`

Expected: delay and gap-accounting checks pass.

- [ ] **Step 4: Commit the isolated fix**

```bash
git add lecture4/homework/src/qos_debugger/src/qos_debugger_sub.cpp
git commit -m "fix: account for subscriber message loss"
```

### Task 4: One-second receive frame-rate report

**Files:**
- Modify: `lecture4/homework/src/qos_debugger/src/qos_debugger_sub.cpp`
- Test: `lecture4/homework/scripts/check_homework.py`

**Interfaces:**
- Consumes: `received_count_`, `last_received_count_`, and `last_report_time_`.
- Produces: measured receive rate in Hz every timer callback.

- [ ] **Step 1: Compute the count delta over monotonic elapsed seconds**

```cpp
const auto now = std::chrono::steady_clock::now();
const double elapsed_seconds =
    std::chrono::duration<double>(now - last_report_time_).count();
const uint32_t received_since_last_report =
    received_count_ - last_received_count_;
const double frame_rate = elapsed_seconds > 0.0
                              ? received_since_last_report / elapsed_seconds
                              : 0.0;
RCLCPP_INFO(this->get_logger(), "接收帧率: %.2f Hz", frame_rate);
last_received_count_ = received_count_;
last_report_time_ = now;
```

- [ ] **Step 2: Run all source checks**

Run: `python3 lecture4/homework/scripts/check_homework.py`

Expected: all checks pass.

- [ ] **Step 3: Commit the isolated fix**

```bash
git add lecture4/homework/src/qos_debugger/src/qos_debugger_sub.cpp
git commit -m "feat: report subscriber receive frame rate"
```

### Task 5: Homework explanations

**Files:**
- Modify: `lecture4/homework/README.md`

**Interfaces:**
- Consumes: final code and actual diagnostic commands.
- Produces: three ordered answers with commands and explanations.

- [ ] **Step 1: Document QoS endpoint diagnosis**

Include:

```bash
ros2 topic info /sensor_data --verbose
ros2 param get /sensor_publisher reliability
ros2 param get /sensor_subscriber reliability
```

Explain the reliable-subscriber versus best-effort-publisher incompatibility and the corrected default.

- [ ] **Step 2: Document loss diagnosis and runtime reproduction**

Include:

```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param set /sensor_subscriber callback_delay_ms 30
ros2 param set /sensor_subscriber callback_delay_ms 0
```

Explain the 100 Hz producer, slow callback backlog, sequence gaps, and corrected cumulative counter.

- [ ] **Step 3: Document frame-rate calculation**

Explain count delta divided by `steady_clock` elapsed seconds and why it measures the subscriber's real processing rate.

- [ ] **Step 4: Commit the documentation**

```bash
git add lecture4/homework/README.md
git commit -m "docs: explain navigation QoS homework"
```

### Task 6: ROS2 Humble build and integration

**Files:**
- Create: `.github/workflows/lecture4-ros2.yml`
- Build output: GitHub Actions runner only.

**Interfaces:**
- Consumes: final navigation homework branch.
- Produces: reproducible ROS2 Humble build and bounded runtime evidence.

- [ ] **Step 1: Add an Ubuntu 22.04 ROS2 Humble workflow**

Use `ros-tooling/setup-ros@v0.7`, install `ros-humble-ros-base`, run `colcon build`, and source both `/opt/ros/humble/setup.bash` and `install/setup.bash`.

- [ ] **Step 2: Run both nodes for a bounded interval**

Start subscriber and publisher in the background, wait four seconds, capture both logs, then terminate them. Require at least one `收到 seq=` line and one `接收帧率:` line.

- [ ] **Step 3: Inspect live endpoint QoS**

While both nodes run, execute `ros2 topic info /sensor_data --verbose` and require publisher and subscriber reliability to be `RELIABLE`.

- [ ] **Step 4: Exercise the delayed loss case**

Run the subscriber with `--ros-args -p callback_delay_ms:=30 -p depth:=1` and require a `检测到丢包` warning plus a cumulative report with a nonzero lost count.

- [ ] **Step 5: Commit and push the workflow with the navigation branch**

```bash
git add .github/workflows/lecture4-ros2.yml
git commit -m "ci: verify Lecture 4 on ROS2 Humble"
```

Expected: the workflow completes successfully on Ubuntu 22.04.

