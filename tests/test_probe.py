"""Exercise the production receive-only 8N1 decoder with known wire data."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ProbeTests(unittest.TestCase):
    def test_serial_tail_timing_and_framing(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "probe"
            subprocess.run([
                os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "src"), str(ROOT / "tests/probe_serial.cpp"), "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True)
