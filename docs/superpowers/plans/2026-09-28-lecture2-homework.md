# Lecture 2 Homework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete and verify all mandatory and optional Lecture 2 homework code for the Hikrobot camera, YOLO armor visualization, and AprilTag visualization.

**Architecture:** Implement a direct RAII wrapper around the Hikrobot C SDK, based on `io/example.cpp`, then compose it with the provided YOLO and AprilTag classes in two small live-loop executables. Verify the target Ubuntu 22.04 dependency set and software-only inference paths in a container; reserve physical acquisition for the required offline camera test.

**Tech Stack:** C++17, CMake 3.16+, Hikrobot MVS SDK, OpenCV 4, OpenVINO 2024.6, yaml-cpp, Eigen3, fmt, libusb-1.0.

## Global Constraints

- `Camera` public members are exactly the constructor, destructor, and `read` function.
- Every private member variable in `Camera` ends in `_`.
- Implement by splitting the SDK sequence in `io/example.cpp`; do not use the reference-only `HikRobot` class.
- YOLO visualization uses a closed green four-point contour and labels color plus armor number/name.
- Target Ubuntu version is 22.04 and OpenVINO version is 2024.6.
- Complete the optional AprilTag/OpenCV-logo task.
- Do not push to the team's upstream repository.

---

### Task 1: Camera configuration and public contract

**Files:**
- Create: `lecture2/homework/configs/camera.yaml`
- Create: `lecture2/homework/io/camera.hpp`

**Interfaces:**
- Consumes: YAML path supplied by the executable.
- Produces: `io::Camera::Camera(const std::string &)`, `io::Camera::~Camera()`, and `io::Camera::read(cv::Mat &, std::chrono::steady_clock::time_point &)`.

- [ ] **Step 1: Write a static contract check**

Run a script that rejects extra public methods and private data members without a trailing underscore.

- [ ] **Step 2: Verify the empty starter fails the contract**

Run: `python3 lecture2/homework/tests/check_camera_contract.py`

Expected: FAIL because `camera.hpp` is empty.

- [ ] **Step 3: Add the exact three-function header and YAML configuration**

Declare only the required public methods. Keep helpers and state private; include `handle_`, `grabbing_`, `timeout_ms_`, and camera settings with trailing underscores.

- [ ] **Step 4: Re-run the contract check**

Expected: PASS with the three public methods and trailing-underscore private state.

### Task 2: Direct Hikrobot SDK implementation

**Files:**
- Create: `lecture2/homework/io/camera.cpp`
- Test: `lecture2/homework/tests/check_camera_contract.py`

**Interfaces:**
- Consumes: the Task 1 `io::Camera` declaration and Hikrobot SDK calls demonstrated in `io/example.cpp`.
- Produces: a live BGR frame in caller-owned `cv::Mat` memory and its `steady_clock` timestamp.

- [ ] **Step 1: Add compile/contract tests that include the header and implementation**

The tests must verify the required API and locate the direct SDK calls: enumerate, create, open, start, get buffer, free buffer, stop, close, and destroy.

- [ ] **Step 2: Run tests and record the missing implementation failure**

Expected: FAIL because `camera.cpp` is empty.

- [ ] **Step 3: Implement minimal RAII acquisition**

Load and validate YAML; select the requested USB VID:PID; initialize the camera; convert supported Bayer/BGR formats; copy data before freeing the SDK buffer; clean up in reverse order.

- [ ] **Step 4: Run the contract/source checks**

Expected: PASS.

### Task 3: Mandatory YOLO visualization pipeline

**Files:**
- Modify: `lecture2/homework/main.cpp`

**Interfaces:**
- Consumes: `io::Camera`, `auto_aim::YOLO::detect`, `Armor::points`, `COLORS`, `ARMOR_NAMES`, `tools::draw_points`, and `tools::draw_text`.
- Produces: a live display with a closed green outline and color/name label for every armor.

- [ ] **Step 1: Add a source-level pipeline check**

The check requires camera/Yolo construction, a capture loop, `detect`, explicit green drawing, label drawing, `imshow`, and `q`/Escape exit.

- [ ] **Step 2: Verify the starter fails**

Expected: FAIL because `main.cpp` contains only comments.

- [ ] **Step 3: Implement the required live loop**

Instantiate `io::Camera("./configs/camera.yaml")` and `auto_aim::YOLO("./configs/yolo.yaml", false)`, draw every result explicitly, and report exceptions to stderr.

- [ ] **Step 4: Run the pipeline check**

Expected: PASS.

### Task 4: Optional AprilTag/OpenCV-logo pipeline

**Files:**
- Modify: `lecture2/homework/opencv.cpp`

**Interfaces:**
- Consumes: `io::Camera`, `auto_charge::AprilTagDetector::detect`, `TagDetection::corners`, and drawing tools.
- Produces: a live display with green tag contours and numeric IDs.

- [ ] **Step 1: Add a source-level AprilTag pipeline check**

Require camera/detector construction, capture loop, detection, green closed contour, ID label, display, and clean exit.

- [ ] **Step 2: Verify the starter fails**

Expected: FAIL because `opencv.cpp` contains only comments.

- [ ] **Step 3: Implement the optional live loop**

Construct the provided detector from `./configs/yolo.yaml`, draw each tag, and report exceptions.

- [ ] **Step 4: Run the pipeline check**

Expected: PASS.

### Task 5: Target-environment build and software smoke tests

**Files:**
- Create: `lecture2/homework/tests/check_homework.py`
- Create: `lecture2/homework/tests/model_smoke.cpp`
- Modify: `lecture2/homework/CMakeLists.txt`

**Interfaces:**
- Consumes: the completed project, model files, and Ubuntu 22.04 dependencies.
- Produces: reproducible PASS evidence for build, YOLO model loading/inference, and AprilTag detection without requiring the physical camera.

- [ ] **Step 1: Add opt-in homework tests to CMake**

Use `BUILD_HOMEWORK_TESTS=ON` to build `model_smoke`; leave normal homework targets unchanged.

- [ ] **Step 2: Build an Ubuntu 22.04 verification environment**

Install `build-essential`, `cmake`, `libopencv-dev`, `libyaml-cpp-dev`, `libeigen3-dev`, `libfmt-dev`, `libusb-1.0-0-dev`, and OpenVINO 2024.6.

- [ ] **Step 3: Configure and compile every target**

Run: `cmake -S lecture2/homework -B build/lecture2 -DBUILD_HOMEWORK_TESTS=ON && cmake --build build/lecture2 -j2`

Expected: all targets compile successfully.

- [ ] **Step 4: Run software-only tests**

Run the source/contract check and `model_smoke` from the homework directory.

Expected: the source/contract check passes; OpenVINO loads `assets/yolov5.xml`, blank-frame inference returns safely, and a generated AprilTag is detected and visualized.

### Task 6: Submission-ready handoff

**Files:**
- Create: `lecture2/homework/README_HOMEWORK.md`
- Create: external user-facing guide under the Codex outputs directory.

**Interfaces:**
- Consumes: implementation and verification evidence.
- Produces: exact setup/build/run/submission instructions, known target/host differences, and a clear real-camera test checklist.

- [ ] **Step 1: Document exact dependency and run commands**

Include Ubuntu 22.04, OpenVINO 2024.6, MVS 5.1.0, VMware USB 3.1, build commands, run commands, and `LD_LIBRARY_PATH` if needed.

- [ ] **Step 2: Document verified versus externally blocked acceptance**

Mark compile/model/AprilTag tests as software-verifiable and physical Hikrobot capture as requiring the team camera at the gray container north of Kaiwu Hall.

- [ ] **Step 3: Review the final diff and run all tests again**

Expected: no starter placeholders in required files, no edits to reference-only HikRobot code, and all available tests pass.

- [ ] **Step 4: Create local commits and a portable patch/archive**

Commit only to the local branch, never to `origin`. Produce SHA256 checksums for the patch/archive.
