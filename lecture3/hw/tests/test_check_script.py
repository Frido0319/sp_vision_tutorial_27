#!/usr/bin/env python3
"""Regression test for the Ubuntu 20.04 CTest invocation in scripts/check.sh."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CheckScriptCompatibilityTest(unittest.TestCase):
    def test_check_script_runs_all_four_ctest_cases(self) -> None:
        result = subprocess.run(
            ["bash", "scripts/check.sh"],
            cwd=ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("100% tests passed, 0 tests failed out of 4", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
