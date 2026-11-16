"""Exercise durable trial decisions through the storage interface."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ConfigTrialTests(unittest.TestCase):
    def test_trial_transaction(self):
        with tempfile.TemporaryDirectory() as work:
            executable = Path(work) / "config-trial"
            subprocess.run(
                [
                    os.environ.get("CXX", "c++"),
                    "-std=c++11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-fsanitize=address,undefined",
                    "-I", str(ROOT / "src"),
                    str(ROOT / "tests" / "config-trial.cpp"),
                    "-o", str(executable),
                ],
                check=True,
            )
            subprocess.run([str(executable)], check=True)
