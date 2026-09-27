# Lecture 2 Homework Design

## Goal

Complete every code deliverable in `lecture2/homework`: wrap the Hikrobot sample into a `Camera` class, build the live camera-to-YOLO visualization pipeline, and complete the optional AprilTag/OpenCV-logo pipeline.

## Source of truth

The implementation follows, in descending order of authority:

1. `Lecture2 Homework.pdf` requirements.
2. The instructor's lecture recording and slide deck.
3. The current repository at commit `47e23977f67a66309b5ea87d582d8406e6894a85`.

The instructor explicitly asks students to split the operations in `io/example.cpp` into `camera.hpp` and `camera.cpp`. The provided `io/hikrobot/` wrapper is reference-only and must not be used as the solution.

## Architecture

### Camera wrapper

`io::Camera` owns one Hikrobot SDK handle. Its public API contains exactly three functions:

```cpp
explicit Camera(const std::string & config_path);
~Camera();
void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
```

The constructor loads YAML configuration, selects a USB camera matching `vid_pid`, creates and opens its handle, applies white-balance/exposure/gain/frame-rate settings, and starts acquisition. `read` obtains one SDK frame, records a steady-clock timestamp, converts supported Bayer or BGR data to owned BGR `cv::Mat` storage, and always releases the SDK buffer. The destructor stops acquisition and closes/destroys the handle without throwing. Every private variable name ends in `_`.

### Required YOLO pipeline

`main.cpp` constructs `io::Camera` and `auto_aim::YOLO`, continuously reads frames, detects armors, draws each four-point contour in green, labels it with its color and armor number/name, displays the result, and exits on `q` or Escape. Detection drawing is done explicitly in `main.cpp` so the submitted result directly demonstrates the homework requirement.

### Optional AprilTag pipeline

`opencv.cpp` constructs the same camera plus `auto_charge::AprilTagDetector`, detects configured OpenCV/AprilTag markers, draws each four-corner contour in green, adds the tag ID, displays the result, and exits on `q` or Escape.

## Configuration

Add `configs/camera.yaml` containing the classroom-recommended Hikrobot settings: camera name, USB VID:PID, exposure in milliseconds, gain, frame rate, and frame timeout. The implementation validates all required values and reports actionable errors.

## Error handling

Construction fails with a descriptive `std::runtime_error` if the configuration is invalid, no matching camera exists, or an SDK initialization/configuration call fails. Frame timeouts also raise a descriptive error. Buffer release is protected by a scope guard so later conversion failures cannot leak an SDK buffer. The destructor performs best-effort cleanup and never throws.

## Verification

Verification has three layers:

1. Compile the complete project in the required Ubuntu 22.04 environment with OpenCV, yaml-cpp, Eigen, fmt, OpenVINO 2024.6, libusb, and the bundled Hikrobot SDK library.
2. Run static checks proving the `Camera` public API has only the required three functions and all private data members end in `_`.
3. Run software-only smoke tests for configuration parsing, model loading, blank-frame YOLO inference, and AprilTag detection/visualization. A real Hikrobot USB camera is not currently attached; live capture remains an explicitly documented offline acceptance step at the team location.

## Scope decisions

- Use the direct SDK split requested by the instructor.
- Do not edit or depend on the reference-only `io/hikrobot/hikrobot.*` implementation.
- Complete the optional AprilTag task, not only the two mandatory tasks.
- Keep the target platform Ubuntu 22.04 even though the host is Ubuntu 20.04; compile and software-smoke-test inside an Ubuntu 22.04 container.
- Do not push to the upstream team repository. Produce a clean local branch/commit and a portable patch/archive for the user to place in their own fork.
