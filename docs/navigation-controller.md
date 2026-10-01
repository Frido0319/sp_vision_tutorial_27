# 迷宫导航控制器说明

## 1. 接口与数据流

`PidController` 实现课程给定的 `sp_controller_server::ControllerPlugin`：

- `configure()` 读取并校验全部参数；
- `setPlan()` 拷贝、清洗并接管全局路径；
- `computeVelocityCommands()` 以当前 `map` 系位姿和速度计算 `base_link` 系平面速度。

跟踪数学核心在 `path_tracker.hpp/.cpp`，不依赖 ROS，可以用 GTest 独立验证。

## 2. 路径清洗与进度

设路径点为 `p[0..N-1]`。`setPath()` 忽略 NaN/Inf 和连续重复点，并把最近点索引重置为 0。每次计算仅在当前索引之后向前搜索，因此噪声不会使进度倒退。

行为树会高频重规划。新路径会重置索引和积分误差，但保留上一速度指令；这样加速度限制不会在每次重规划时重新从零开始。空路径仍会立即清零。

## 3. 前视点与拐角

前视点沿折线弧长累计，不使用路径点序号作为距离近似。若前视区间内的相邻线段夹角超过 `corner_slowdown_angle`，目标点被截止在该角点，速度上限乘以 `corner_speed_ratio`。这同时避免两个问题：

1. 在尖角前追逐角点后的点，形成穿过墙角的捷径；
2. 只减速但仍以切角方向行驶。

## 4. PID、前馈与限幅

记前视点误差为 `e = target - position`，方向化前馈速度为 `v_ff`，测得的 `map` 系速度为 `v`：

```text
u_raw = v_ff + kp * e + ki * clamp(∫e dt) + kd * (v_ff - v)
```

处理顺序为：

1. 每轴积分限制在 `±integral_limit`；
2. 根据拐角和终点距离计算当前速度上限；
3. 限制速度矢量的模，而非分别截断 x/y；
4. 限制相邻指令的矢量变化量不超过 `max_acceleration * dt`。

当与终点的距离小于 `goal_slowdown_distance` 时，速度上限随距离线性衰减；进入 `goal_tolerance` 后精确输出零。

## 5. 坐标系旋转

路径、当前位置和 controller server 给出的速度都在 `map` 系，但发放的仿真器把 `/sentry/cmd_vel` 的 x/y 解释为 `base_link` 分量。因此在插件边界执行逆旋转：

```text
vx_base =  cos(yaw) * vx_map + sin(yaw) * vy_map
vy_base = -sin(yaw) * vx_map + cos(yaw) * vy_map
```

插件测试使用 90° yaw 显式验证该方向和符号。

## 6. Controller server 安全边界

`controller_server` 只在成功取得里程计位姿、TF 和有限速度后刷新里程计时间戳。若启动后尚无有效样本，或最近一次有效样本已超过 `odom_timeout_seconds`，本周期直接发布零速度，不调用控制器插件。空路径、插件未加载、位姿查询失败和控制器异常也都走同一零速度出口。

本作业把 `odom_timeout_seconds` 设为 `0.30 s`。该看门狗位于插件外层，所以即使 RViz 或其他 GUI 竞争 CPU 导致仿真里程计暂时停顿，也不会让底盘继续执行过期速度指令。

## 7. 参数

|Parameter|Value|Purpose|
|---|---:|---|
|`kp`|1.40|位置比例项|
|`ki`|0.02|消除稳态误差|
|`kd`|0.35|用测量速度增加阻尼|
|`integral_limit`|0.30|每轴积分限幅|
|`lookahead_distance`|0.35 m|折线弧长前视|
|`max_speed`|1.00 m/s|直线速度上限|
|`max_acceleration`|1.50 m/s²|指令加速度上限|
|`corner_slowdown_angle`|0.65 rad|拐角判定阈值|
|`corner_speed_ratio`|0.30|拐角速度比例|
|`goal_slowdown_distance`|1.00 m|终点减速距离|
|`goal_tolerance`|0.15 m|控制器停车阈值|
|`odom_timeout_seconds`|0.30 s|controller server 里程计超时零速阈值|

## 8. 测试覆盖

`test_path_tracker` 覆盖路径所有权与清洗、进度单调性、高频重规划、速度/加速度限幅、拐角减速、不切角、终点停车和非有限输入。`test_pid_plugin` 通过 pluginlib 动态加载 `PidController`，并验证参数、非零跟踪和坐标旋转。`test_controller_safety` 覆盖未收到里程计、阈值内有效和超时失效三种状态。
