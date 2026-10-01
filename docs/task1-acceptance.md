# Task 1 要求与验收记录

## 1. 本赛季官方功能要求

依据 `Lecture4 Hello Armor&Solver.pptx` 第 27–28 页，自瞄方向大作业第一部分为“云台跟随装甲板”：

- 考核人员运行提交的可执行文件，并把步兵机器人设为自瞄挡位；
- 考核人员在云台可运动范围内移动手持装甲板；
- 范围为 yaw 360°、pitch 约 ±20°；
- 云台应不断指向手持装甲板，且不发生异常运动；
- 考核人员记录 Plotter 软件的输出截图。

课件未要求 Task 1 开火；本实现在所有路径均固定 `fire=false`。

## 2. 实现对应

| 官方要求 | 代码对应 | 当前证据 |
| --- | --- | --- |
| 运行 Task 1 可执行文件 | `src/task_1.cpp`，构建目标 `task_1` | Ubuntu 22.04 全新构建、CLI 和动态库检查通过 |
| 自瞄挡位启用跟随 | 仅 `GimbalMode::AUTO_AIM` 且检测到装甲板时 `control=true` | 代码级已核对；待实车验收 |
| 指向手持装甲板 | 图像→YOLO→PnP/solver→世界系 yaw/pitch→云台命令 | 处理顺序和输出字段已静态核对；待实车验收 |
| 无异常运动 | 无目标/非自瞄/非有限角度时发送禁用零命令；250 ms watchdog | 软件安全路径已检查；待实车运动验收 |
| Plotter 截图 | 发布 `control`、`command_yaw`、`command_pitch`、`target_distance` | 字段已核对；截图必须现场采集 |

## 3. 调试资料要求

`调试须知（必看）.pdf` 明确了现场流程：

- 个人电脑安装 NoMachine，通过网线连接与车辆配对的车载 Ubuntu 电脑；
- NoMachine 的 Host 使用现场提供的 IP；
- PlotJuggler 在 `Streaming` 中选择 `UDP server`，启动后使用界面默认 IP 和端口；
- PDF 截图展示 `127.0.0.1:9870` 和 JSON，但正文要求是使用现场界面默认值；
- `fanblade_point` 是 PDF 中的可视化示例，不是 Task 1 必填字段。

## 4. 验收状态

- **VERIFIED_LOCAL：** Ubuntu 22.04 全新构建、`task_1 --help`、`ldd` 无缺库，处理顺序、AUTO_AIM 门控、不开火、非有限值保护、watchdog 和 Plotter 字段已检查。
- **BLOCKED_EXTERNAL：** Hikrobot 相机采集、`/dev/gimbal` 串口姿态、yaw 360° / pitch ±20° 实体跟随、无异常运动和 PlotJuggler 现场截图。

软件构建通过不等于实车考核通过；后者必须在课程车辆上由考核人员运行并记录。

## 5. 提交与截止时间

- 代码分支：`final_project_aim`。
- 群公告明确第一部分与第二部分将使用统一截止时间；当前已核验材料没有给出最终 DDL。
- 调试 PDF 未规定仓库链接、截图或视频的私聊格式；现场结束前需向考核人员确认。
