#!/usr/bin/env python3

import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUBLISHER = ROOT / "src/qos_debugger/src/qos_debugger_pub.cpp"
SUBSCRIBER = ROOT / "src/qos_debugger/src/qos_debugger_sub.cpp"


def require(source: str, text: str, description: str, failures: list[str]) -> None:
    if text not in source:
        failures.append(description)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Lecture 4 homework source")
    parser.add_argument(
        "--task", choices=("all", "qos", "loss", "rate"), default="all"
    )
    args = parser.parse_args()

    publisher = PUBLISHER.read_text(encoding="utf-8")
    subscriber = SUBSCRIBER.read_text(encoding="utf-8")
    failures: list[str] = []

    if args.task in ("all", "qos"):
        require(
            publisher,
            'declare_parameter("reliability", "reliable")',
            "publisher must offer reliable QoS by default",
            failures,
        )

    if args.task in ("all", "loss"):
        require(
            subscriber,
            'declare_parameter("callback_delay_ms", 0)',
            "subscriber must not sleep in its default callback path",
            failures,
        )
        require(
            subscriber,
            "lost_count_ += lost;",
            "sequence gaps must update lost_count_",
            failures,
        )

    if args.task in ("all", "rate"):
        require(
            subscriber,
            "received_count_ - last_received_count_",
            "frame-rate calculation must use the received-count delta",
            failures,
        )
        require(
            subscriber,
            "std::chrono::duration<double>",
            "frame-rate calculation must use elapsed steady-clock time",
            failures,
        )
        require(
            subscriber,
            '"\u63a5\u6536\u5e27\u7387: %.2f Hz"',
            "frame rate must be logged in Hz",
            failures,
        )

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1

    print(f"PASS: Lecture 4 source checks ({args.task})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
