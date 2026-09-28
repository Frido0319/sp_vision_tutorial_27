# Lecture 3 Homework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the mini vision pipeline own frame data, process every frame exactly once with multiple workers, publish coherent statistics, and shut down safely through either `wait()` or destruction.

**Architecture:** Clone the source's reusable image buffer at the ownership boundary, serialize statistics through one mutex, and make `wait()` the single idempotent thread-join protocol used by both callers and the destructor. Preserve the supplied queue, processor, tests, and multi-worker design.

**Tech Stack:** C++17, OpenCV 4, CMake 3.16+, `std::thread`, `std::mutex`, supplied `BlockingQueue<Frame>`.

## Global Constraints

- Do not modify tests, `BlockingQueue`, or `ImageProcessor`.
- Keep at least two workers and one independent producer thread.
- Every input frame must be processed and saved exactly once.
- `Statistics::snapshot()` must return one coherent four-counter view.
- Both explicit `wait()` and destructor-only shutdown must be safe.
- Complete all four report sections in the student's own explanatory language.

---

### Task 1: Stable frame ownership

**Files:**
- Modify: `lecture3/hw/src/frame_source.cpp`
- Test: `lecture3/hw/tests/test_frame.cpp`

**Interfaces:**
- Consumes: camera-like reusable `ImageSequenceSource::buffer_`.
- Produces: a `Frame::image` with independent pixel storage.

- [ ] **Step 1: Run the supplied ownership test and record the expected red result**

Run: `./build/frame_integrity_test`

Expected: exit 1 with `an older frame changed when the source captured a new frame`.

- [ ] **Step 2: Replace the shallow assignment with an owning clone**

```cpp
frame.image = buffer_.clone();
```

- [ ] **Step 3: Rerun the focused test**

Run: `./build/frame_integrity_test`

Expected: `PASS: queued frame owns valid image data`.

- [ ] **Step 4: Commit the isolated fix**

```bash
git add lecture3/hw/src/frame_source.cpp
git commit -m "fix: give queued frames owned image data"
```

### Task 2: Coherent shared statistics

**Files:**
- Modify: `lecture3/hw/include/statistics.hpp`
- Modify: `lecture3/hw/src/statistics.cpp`
- Test: `lecture3/hw/tests/test_statistics.cpp`

**Interfaces:**
- Consumes: concurrent calls to the four increment methods.
- Produces: exact counters and a coherent `StatisticsSnapshot`.

- [ ] **Step 1: Run the supplied statistics test and record the expected red result**

Run: `./build/statistics_test`

Expected: exit 1 with an actual count below 400.

- [ ] **Step 2: Add one mutex protecting all counters**

```cpp
#include <mutex>

mutable std::mutex mutex_;
int produced_ = 0;
int processed_ = 0;
int saved_ = 0;
int corrupted_ = 0;
```

- [ ] **Step 3: Lock every update and snapshot**

```cpp
void Statistics::onProduced()
{
    std::lock_guard<std::mutex> lock(mutex_);
    deliberatelySlowIncrement(produced_);
}

StatisticsSnapshot Statistics::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {produced_, processed_, saved_, corrupted_};
}
```

Apply the same lock pattern to `onProcessed`, `onSaved`, and `onCorrupted`.

- [ ] **Step 4: Rerun the focused test**

Run: `./build/statistics_test`

Expected: `PASS: shared statistics are correct`.

- [ ] **Step 5: Commit the isolated fix**

```bash
git add lecture3/hw/include/statistics.hpp lecture3/hw/src/statistics.cpp
git commit -m "fix: synchronize pipeline statistics"
```

### Task 3: Idempotent shutdown protocol

**Files:**
- Modify: `lecture3/hw/include/pipeline.hpp`
- Modify: `lecture3/hw/src/pipeline.cpp`
- Test: `lecture3/hw/tests/test_shutdown.cpp`
- Test: `lecture3/hw/tests/test_pipeline.cpp`

**Interfaces:**
- Consumes: `Pipeline::start()`, optional caller `wait()`, and destruction.
- Produces: one-shot start plus repeat-safe joining through `wait()`.

- [ ] **Step 1: Run the supplied shutdown and pipeline tests and record both red results**

Run: `./build/shutdown_test && ./build/pipeline_test`

Expected: `shutdown_test` aborts because joinable threads reach destruction, so the second command does not run.

- [ ] **Step 2: Add one-shot lifecycle state**

```cpp
bool started_ = false;
```

- [ ] **Step 3: Make the destructor delegate to the join protocol**

```cpp
Pipeline::~Pipeline()
{
    wait();
}
```

- [ ] **Step 4: Reject a repeated start before creating threads**

```cpp
if (started_)
{
    throw std::logic_error("Pipeline::start() may only be called once");
}
started_ = true;
```

Keep the existing producer-first and workers-second join order. Joinability checks make repeated `wait()` calls harmless.

- [ ] **Step 5: Rebuild and run both focused tests separately**

Run: `cmake --build build -j2 && ./build/shutdown_test && ./build/pipeline_test`

Expected: exit 0 and `PASS: all frames processed exactly once`.

- [ ] **Step 6: Commit the isolated fix**

```bash
git add lecture3/hw/include/pipeline.hpp lecture3/hw/src/pipeline.cpp
git commit -m "fix: make pipeline shutdown safe"
```

### Task 4: Student understanding report

**Files:**
- Modify: `lecture3/hw/report.md`

**Interfaces:**
- Consumes: the final ownership, queue, statistics, and shutdown implementation.
- Produces: four complete explanations consistent with the code.

- [ ] **Step 1: Explain image lifetime and ownership**

State that the source reuses `buffer_`, `cv::Mat` assignment is shallow, `clone()` allocates independent storage, and the queued frame keeps that allocation alive.

- [ ] **Step 2: Explain exactly-once queue behavior**

State that `push` inserts once, mutex-protected `pop` removes once, `close` wakes all waiters, and existing elements drain before workers exit.

- [ ] **Step 3: Explain shared statistics**

Identify producer and worker writers plus main-thread readers, the lost-update race, and the one-mutex coherent snapshot guarantee.

- [ ] **Step 4: Explain both shutdown paths**

Describe explicit `wait()`, destructor-delegated `wait()`, joinability checks, queue closure, and the rejection of repeated `start()`.

- [ ] **Step 5: Verify report completeness and commit**

Run: `test "$(grep -c '^T[D]O(report)' lecture3/hw/report.md)" -eq 0`

Expected: exit 0.

```bash
git add lecture3/hw/report.md
git commit -m "docs: complete Lecture 3 understanding report"
```

### Task 5: Full target verification

**Files:**
- No source changes.
- Build output: `/home/ubuntu20/桌面/project/Linux-50G/sp-course-build/lecture3`

**Interfaces:**
- Consumes: final Lecture 3 source tree.
- Produces: fresh host and Ubuntu 22.04 evidence.

- [ ] **Step 1: Configure outside the system disk workspace**

Run: `cmake -S lecture3/hw -B /home/ubuntu20/桌面/project/Linux-50G/sp-course-build/lecture3 -DBUILD_TESTING=ON`

Expected: configuration succeeds with OpenCV and Threads found.

- [ ] **Step 2: Build and run all supplied tests**

Run: `cmake --build /home/ubuntu20/桌面/project/Linux-50G/sp-course-build/lecture3 -j2 && (cd /home/ubuntu20/桌面/project/Linux-50G/sp-course-build/lecture3 && ctest --output-on-failure)`

Expected: 4/4 tests pass.

- [ ] **Step 3: Run the full checker in Ubuntu 22.04**

Mount the repository and execute `lecture3/hw/scripts/check.sh` in the existing Ubuntu 22.04 verification image, with build and test output directed to the 50 GB volume where possible.

Expected: 2, 3, and 6 worker cases pass; report check passes; final output is `ALL TESTS PASSED`.
