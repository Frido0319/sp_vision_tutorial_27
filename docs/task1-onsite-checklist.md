# Task 1 现场调试清单

> 状态边界：截至 2026-10-01，Task 1 已在 Ubuntu 22.04 完成全新构建、CLI、动态依赖和代码级安全检查；尚未连接课程车辆、Hikrobot 相机或 `/dev/gimbal`。以下步骤必须在实验室完成，执行前不得把物理验收标为通过。

## 1. 出发前

- 个人电脑安装 NoMachine：<https://www.nomachine.com/>。
- 携带网线；电脑若没有 RJ45 网口，携带可用的 USB 转网口适配器。
- 代码分支使用 `final_project_aim`，不要在现场临时改 `task_2.cpp` 或 `task_3.cpp`。
- 到场前同步 `final_project_aim` 远程分支；现场构建前用 `git rev-parse HEAD` 记录实际提交哈希。

## 2. 到场连接

1. 地点：开物馆进门左转，从电梯旁楼梯下到地下车库。
2. 用网线连接个人电脑和分配的车载电脑。
3. 向现场学长取得与该车辆一一对应的 Host IP；课程材料没有给出固定 IP、用户名或密码，不能自行猜测。
4. NoMachine 左上角选择 `Add`：`Name` 自定，`Host` 填现场提供的 IP，保存后双击连接。
5. 在车载 Ubuntu 上确认设备：

```bash
lsusb | grep -i '2bdf'
test -e /dev/gimbal && ls -l /dev/gimbal
```

两项缺一都先找现场学长处理，不启动实体云台跟随。

## 3. 获取并构建代码

在车载 Ubuntu 22.04 上执行：

```bash
git clone https://github.com/Frido0319/sp_vision_tutorial_27.git
cd sp_vision_tutorial_27
git fetch origin
git switch final_project_aim
git rev-parse HEAD
source /opt/intel/openvino_2024.6.0/setupvars.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target task_1 --parallel 2
```

`git rev-parse HEAD` 应记录在现场验收截图或日志中。若车载电脑已有课程仓库，先确认没有他人未提交改动，再在原仓库 `fetch`/`switch`，不要直接覆盖。

## 4. PlotJuggler

1. 从车载 Ubuntu 应用栏打开 PlotJuggler。
2. 在 `Streaming` 下方的大方框选择 `UDP server`。
3. 点击 `Start`，IP 与端口使用界面默认值，再点击 `OK`。
4. PDF 截图中展示的是 `127.0.0.1:9870`、协议 `json`，但正文只要求使用默认值；以现场软件的默认值和现场指导为准。
5. 程序运行后，在 `Timeseries List` 查找 `control`、`command_yaw`、`command_pitch`、`target_distance`，拖到右侧 tab 查看曲线。PDF 中的 `fanblade_point` 只是视频示例，不是本 Task 1 的必需字段。

## 5. 启动 Task 1

先确保人员远离云台运动范围、枪口/发射机构处于安全状态，并由现场学长确认可以上电。程序全路径始终发送 `fire=false`，但软件约束不能替代现场断弹和机械安全措施。

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$PWD/io/hikrobot/lib/amd64:${LD_LIBRARY_PATH:-}
./build/task_1 configs/standard.yaml
```

退出使用检测窗口中的 `q` 或 `Esc`。正常退出、普通 C++ 异常或主循环超过 250 ms 未刷新命令时，程序会发送禁用且不开火的零值命令；底层串口永久阻塞、进程被 `SIGKILL`、断电或控制器不接收数据仍不在软件保证范围内。

## 6. 现场验证与证据

以下功能判据来自本赛季 `Lecture4 Hello Armor&Solver.pptx` 第 27–28 页：

- 手持装甲板绕 yaw 约 360°、pitch 约 ±20° 移动。
- 云台应不断指向装甲板，不出现异常运动；本实现保持不开火。
- 由考核人员记录 Plotter 软件输出截图；建议同时展示 `command_yaw`、`command_pitch`、`control` 便于解释。
- 另保存：设备枚举、当前提交哈希、程序稳定运行画面、相机识别画面、NoMachine/PlotJuggler 连接画面。
- 让现场学长明确确认“Task 1 是否通过、是否还需提交截图/视频/仓库链接、最终截止时间”。当前公告仅说第一、二部分共用截止时间，具体时间待后续通知。

## 7. 发给学长的信息模板

```text
学长你好，我是【姓名】【学号】。算法组招新大作业自瞄方向 Task 1 已完成软件实现，代码：
https://github.com/Frido0319/sp_vision_tutorial_27/tree/final_project_aim
现场使用提交：【填写 git rev-parse HEAD】
Ubuntu 22.04 构建与不开火安全检查已通过；实体车辆调试状态：【通过 / 仍有问题，具体为……】。
PlotJuggler 与现场证据：【按要求附截图或视频】
麻烦确认是否还需要其他提交材料，谢谢。
```
