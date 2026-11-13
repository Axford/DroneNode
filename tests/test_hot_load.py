"""Verify the firmware's bounded INI filename parser and result values."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class HotLoadTests(unittest.TestCase):
    def test_root_ini_filenames(self):
        with tempfile.TemporaryDirectory() as work:
            source = Path(work) / "hot_load.cpp"
            executable = Path(work) / "hot_load"
            source.write_text(r'''
#include "HotLoadConfiguration.h"
#include <cassert>

int main() {
    char path[18] = {};
    const char exact[16] = {'a','b','c','d','e','f','g','h','i','j','k','l','.','i','n','i'};
    assert(configuration::hotLoadPath(exact, sizeof(exact), path));
    assert(!strcmp(path, "/abcdefghijkl.ini"));
    const char* invalid[] = {"", ".ini", "../file.ini", "/root.ini", "a/b.ini",
                             "a.INI", "a.txt", "a ini.ini", "abcdefghijklm.ini"};
    for (const auto* name : invalid) assert(!configuration::hotLoadPath(name, strlen(name), path));
    assert(!configuration::hotLoadPath("a\0.ini", 6, path));
    assert(configuration::hotLoadPath("Trial_1-a.ini", 13, path));
    assert(!strcmp(path, "/Trial_1-a.ini"));
}
''')
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra",
                            "-Werror", "-I", str(ROOT / "src"), str(source), "-o", str(executable)],
                           check=True)
            subprocess.run([str(executable)], check=True)
