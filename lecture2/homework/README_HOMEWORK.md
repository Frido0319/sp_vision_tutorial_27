# Lecture 2 作业运行与验收说明

本目录已完成三部分：

1. `io/camera.hpp`、`io/camera.cpp`：把 `io/example.cpp` 的海康相机流程封装为 `io::Camera`。
2. `main.cpp`：相机取帧、YOLO 装甲板识别、绿色闭合框、颜色与数字/名称标注。
3. `opencv.cpp`：附加题，使用提供的 `AprilTagDetector` 识别 OpenCV/AprilTag 标志并可视化。

## 1. 指定环境

- Ubuntu 22.04（课程指定版本）
- CMake 3.16+
- C++17 / g++
- OpenCV 4
- OpenVINO 2024.6，安装在 `/opt/intel/openvino_2024.6.0`
- yaml-cpp、Eigen3、fmt、spdlog、libusb-1.0
- Hikrobot MVS 5.1.0 Linux x86_64

APT 依赖：

```bash
sudo apt update
sudo apt install -y g++ cmake libopencv-dev libyaml-cpp-dev \
  libeigen3-dev libfmt-dev libspdlog-dev libusb-1.0-0-dev curl
```

OpenVINO 按课程 PDF 安装 2024.6：

```bash
cd ~/Downloads
curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz \
  --output openvino_2024.6.0.tgz
tar -xf openvino_2024.6.0.tgz
sudo mkdir -p /opt/intel
sudo mv l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64 \
  /opt/intel/openvino_2024.6.0
cd /opt/intel/openvino_2024.6.0
sudo -E ./install_dependencies/install_openvino_dependencies.sh
```

MVS 从海康机器人官网下载“机器视觉工业相机客户端 MVS V5.1.0 (Linux)”并安装。PDF 中命令在 `./` 后误排了一个空格；正确形式是：

```bash
sudo apt install ./MVS-5.1.0_x86_64_....deb
```

## 2. 相机配置

先查看 USB 标识：

```bash
lsusb
```

修改 `configs/camera.yaml` 中的 `vid_pid`，使其与相机的 `VID:PID` 一致。默认值取自课堂示例：

```yaml
camera_name: "hikrobot"
exposure_ms: 5.0
gain: 16.9
vid_pid: "2bdf:0001"
frame_rate: 60.0
timeout_ms: 100
```

课堂建议低曝光以减少运动模糊；海康增益的课件范围是 `[0, 16.9]`。仓库 `example.cpp` 写了 `Gain=20`，与课件不一致，本实现按课件使用 `16.9`。实机仍应根据现场画面调参。

## 3. 构建

在本目录执行：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
cmake -S . -B build -DBUILD_HOMEWORK_TESTS=ON
cmake --build build -j"$(nproc)"
python3 tests/check_homework.py
(cd build && ctest --output-on-failure)
```

## 4. 运行

如果运行时找不到海康 SDK 动态库，先执行：

```bash
export LD_LIBRARY_PATH="$(pwd)/io/hikrobot/lib/amd64:${LD_LIBRARY_PATH}"
```

必做题：

```bash
./build/main
```

附加题：

```bash
./build/opencv
```

两个程序均按 `q` 或 `Esc` 退出。

## 5. 已验证内容

在 Ubuntu 22.04、GCC 11.4、CMake 3.22、OpenCV 4.5.4、OpenVINO 2024.6 环境中已验证：

- 所有目标 `main`、`opencv`、`example`、`model_smoke` 编译成功。
- 10 项源代码/接口验收通过。
- OpenVINO 成功加载 `assets/yolov5.xml` 并完成一帧 CPU 推理。
- 生成的 AprilTag 36h11、ID 10 被正确识别，并绘制绿色闭合框与文字。
- 无相机时两个程序会输出清晰错误并以非零状态退出，不会崩溃。

2026-09-29 又在 Ubuntu 20.04 宿主机上使用官方 MVS 5.1.0 和 Hikrobot
MV-CA016-10UC 相机完成实机验收。相机以 USB3 SuperSpeed（5000 Mbps）连接：

- 必做题 `main` 已完成连续取流、YOLO 识别和绿色闭合框显示。
- 附加题 `opencv` 已识别 AprilTag ID 10、18、24，并显示绿色闭合框和 ID。
- 两项作业的功能正确性均已通过；全分辨率实时识别仍可见一定显示时延，不将其误写为零延迟。

## 6. 线下实机验收

地点：嘉定校区开物馆大厅北侧灰色集装箱。

1. VMware 设置 → USB 控制器 → USB 兼容性设为 USB 3.1。
2. 将海康相机从宿主机连接到 Ubuntu 虚拟机。
3. `lsusb` 确认相机，并核对 `configs/camera.yaml` 的 `vid_pid`。
4. 用 MVS 客户端确认相机可见且可取流。
5. 运行 `./build/main`，确认连续画面、YOLO 识别、绿色四点闭合框、颜色与数字/名称文字均出现。
6. 运行 `./build/opencv`，对准课程提供的 OpenCV/AprilTag 标志，确认绿色框和 ID。

## 7. 提交

- 线上：把自己 fork 后的仓库链接私发群管理员，备注学号、姓名。
- 线下：携带能运行本代码的 Ubuntu 22.04 虚拟机到上述地点连接实机。
- 截止时间以群内当天通知为准；现有群消息给出的 Lecture 2 截止时间是 2026-09-29 23:59。
