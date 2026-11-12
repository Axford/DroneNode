"""Compile the production serial core with a host compiler; no device or Arduino runtime."""
import os
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class SerialTransportTests(unittest.TestCase):
    def test_production_transport(self):
        with tempfile.TemporaryDirectory() as work:
            fixtures=json.loads((ROOT/"tests/fixtures/serial-vectors.json").read_text())
            declarations=[]
            for kind in ('mesh','diagnostic'):
                for i,case in enumerate(fixtures[kind]):
                    declarations.append('static const unsigned char %s%d[]={%s};' % (kind,i,','.join(str(b) for b in bytes.fromhex(case['hex']))))
            (Path(work)/"SerialVectors.h").write_text('\n'.join(declarations))
            executable = str(Path(work)/"serial-test")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-I", work, "-I", str(ROOT/"src"), str(ROOT/"tests/serial_transport.cpp"), "-o", executable], check=True)
            subprocess.run([executable], check=True)

    def test_actual_serial_module(self):
        import shutil
        with tempfile.TemporaryDirectory() as work:
            tree=Path(work)
            shutil.copytree(ROOT/"tests/serial_stubs", tree/"src")
            (tree/"src/droneModules").mkdir()
            for name in ("SerialTelemetryModule.cpp", "SerialTelemetryModule.h"):
                shutil.copyfile(ROOT/"src/droneModules"/name, tree/"src/droneModules"/name)
            shutil.copyfile(ROOT/"tests/serial_stubs/NetworkInterfaceModule.h",tree/"src/droneModules/NetworkInterfaceModule.h")
            for name in ("SerialTransport.h", "DroneConsole.h", "DroneConsole.cpp"):
                shutil.copyfile(ROOT/"src"/name,tree/"src"/name)
            executable=str(tree/"module-test")
            subprocess.run([os.environ.get("CXX","c++"),"-std=c++11","-Wall","-Wextra","-I",str(tree/"src"),str(tree/"src/droneModules/SerialTelemetryModule.cpp"),str(ROOT/"tests/serial_module.cpp"),str(tree/"src/DroneConsole.cpp"),"-o",executable],check=True)
            subprocess.run([executable],check=True)
