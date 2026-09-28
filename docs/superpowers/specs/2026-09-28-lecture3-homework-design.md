# Lecture 3 Homework Design

## Goal

Repair the mini multi-threaded vision pipeline so every input frame owns stable image data, is processed and saved exactly once, statistics remain coherent under concurrent access, and both explicit waiting and destructor-only shutdown are safe.

## Source of truth

The implementation follows `lecture3/hw/README.md`, the four provided tests, and the report template. Provided tests, `BlockingQueue`, and `ImageProcessor` remain unchanged. The worker count remains at least two.

## Ownership model

`ImageSequenceSource` simulates a camera by reusing `buffer_` for every capture. A normal `cv::Mat` assignment copies only the header and shares pixel storage, so queued frames currently change when the next image overwrites the buffer. `next()` will clone the reusable buffer into `frame.image`. Each queued `Frame` therefore owns its own reference-counted allocation, and moving or copying the `Frame` across the queue preserves that allocation until the last owner releases it.

## Concurrency model

The producer creates frames, increments `produced`, and pushes each frame into one shared blocking queue. After source exhaustion it closes the queue exactly once. Each worker repeatedly calls `pop`; the queue removes one element under its mutex, so one frame can reach only one worker. Closing wakes blocked workers but still allows them to drain elements already queued. A worker exits only when the queue is both closed and empty.

`Statistics` will protect all four counters with one mutex. Every increment and the four-field snapshot hold the same mutex. A single mutex, rather than independent atomics, makes the snapshot a coherent point-in-time view of related counters.

## Shutdown model

`wait()` joins the producer first, then joins every worker. Joinability checks make repeated calls harmless. The destructor delegates to `wait()`, so destroying a started pipeline follows the same protocol and never destroys a joinable `std::thread`. A never-started pipeline also remains safe because no thread is joinable.

`start()` is a one-shot lifecycle operation. Add a private `started_` flag and reject a second call with `std::logic_error`; this prevents duplicate producer/worker creation against a queue that cannot be reopened.

## Error handling

The required tests exercise normal input and shutdown paths. Source construction and image reads continue to throw their existing descriptive exceptions. Worker output failures remain observable through the saved counter. No exception crosses a worker or producer boundary in the supplied normal workload.

## Verification

1. Record the current red baseline for all four supplied tests.
2. Apply one root-cause fix at a time and rerun the directly affected test.
3. Run all four tests from the build directory on the host.
4. Run `scripts/check.sh` inside Ubuntu 22.04, where CMake 3.22 supports its `ctest --test-dir` invocation.
5. Verify the 2, 3, and 6 worker runs each save exactly 20 images and `report.md` contains all four completed sections with no unresolved marker.

