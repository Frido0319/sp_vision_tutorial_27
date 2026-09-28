# Lecture 4 Navigation Homework Design

## Goal

Complete the ROS2 Humble QoS debugger assignment on the upstream `nav` branch: restore publisher/subscriber compatibility, eliminate the intentional subscriber bottleneck while reporting real sequence loss correctly, print measured receive frame rate once per second, and document the diagnostic commands and reasoning.

## Branch and environment

Lecture 4 is independent from the first three lessons and will be implemented on a local branch based on `origin/nav`. Its worktree and build files live under `/home/ubuntu20/桌面/project/Linux-50G`, the 50 GB ext4 volume with about 40 GB free. The host is Ubuntu 20.04 with ROS Noetic, so ROS2 Humble verification runs in Ubuntu 22.04/ROS2 Humble continuous integration rather than changing the host installation.

## Task 1: QoS compatibility

The starter publisher offers `best_effort` while the starter subscriber requests `reliable`. ROS2 does not match a reliable subscription with a best-effort publisher. Change the publisher's default `reliability` parameter to `reliable`, preserving the existing parameterized QoS construction. The README will use `ros2 topic info /sensor_data --verbose` to show the offered and requested policies.

## Task 2: Loss behavior and accounting

The publisher sends at 100 Hz while the subscriber deliberately sleeps 30 ms per callback, so the single-threaded executor cannot service messages fast enough. Change the subscriber's default `callback_delay_ms` to zero so the normal configuration keeps up. Preserve the runtime parameter so the delay can still reproduce loss intentionally.

When a sequence gap occurs, add the gap size to `lost_count_`; the starter logs the gap but never updates the counter. Reset the sequence baseline when sequence numbers move backward to avoid unsigned underflow and misleading loss totals after wraparound, restart, or another publisher.

## Task 3: Receive frame rate

The one-second timer computes frame rate from the increase in `received_count_` divided by elapsed `steady_clock` time. It logs the current Hz, then updates `last_received_count_` and `last_report_time_`. `steady_clock` is used because elapsed-time measurement must not jump when wall-clock time changes.

## Documentation

Append three answer sections to `lecture4/homework/README.md`. Each section includes the exact command used, what output to inspect, the code change, and the student's explanation. The commands cover topic endpoint inspection, parameter listing/get/set, rebuild and sourcing, and running both nodes.

## Verification

1. Add source-level checks that fail against the starter and assert the three required behaviors without modifying provided interfaces.
2. Build both packages with `colcon build` under ROS2 Humble.
3. Start subscriber and publisher together for a bounded interval, capture logs, and require received messages plus a positive frame-rate report.
4. Inspect `ros2 topic info /sensor_data --verbose` while both nodes run and require compatible reliable endpoints.
5. Run a delayed-subscriber case to require at least one gap warning and a nonzero loss counter.

