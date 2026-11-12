"""Host-compile actual telemetry policy/configuration/framing; no device access."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
class CloudTelemetryTests(unittest.TestCase):
    def test_actual_report_policy(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / 'cloud-test')
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', '-I', str(ROOT/'src'), str(ROOT/'tests/cloud_telemetry.cpp'), '-o', executable], check=True)
            subprocess.run([executable], check=True)

    def test_actual_firmware_module(self):
        import shutil
        with tempfile.TemporaryDirectory() as directory:
            tree=Path(directory)/'src'
            shutil.copytree(ROOT/'tests/cloud_stubs', tree)
            (tree/'droneModules').mkdir()
            for name in ('CloudTelemetryModule.cpp','CloudTelemetryModule.h'):
                shutil.copyfile(ROOT/'src/droneModules'/name,tree/'droneModules'/name)
            for name in ('TelemetryReport.h','CloudCommandBatch.h','CloudCommandWire.h','CloudTelemetryHttp.h','CloudTelemetryConfig.h','ConfigurationLine.h','FirmwareMeasurements.h'):
                shutil.copyfile(ROOT/'src'/name,tree/name)
            base=(ROOT/'src/DroneModule.cpp').read_text()
            start=base.index('void DroneModule::saveConfiguration(')
            end=base.index('void DroneModule::reset()',start)
            (tree/'actual_save.inc').write_text(base[start:end])
            executable=str(Path(directory)/'module-test')
            subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-Wall','-Wextra','-fsanitize=address,undefined','-I',str(tree),str(tree/'droneModules/CloudTelemetryModule.cpp'),str(ROOT/'tests/cloud_module.cpp'),'-o',executable],check=True)
            subprocess.run([executable],check=True)
            source=(ROOT/'src/DroneExecutionManager.cpp').read_text()
            start=source.index('void DroneExecutionManager::loadConfiguration(')
            end=source.index('void DroneExecutionManager::saveConfiguration()',start)
            (tree/'actual_loader.inc').write_text(source[start:end])
            channel=(ROOT/'src/DroneLinkChannel.cpp').read_text()
            start=channel.index('void DroneLinkChannel::processQueue()')
            end=channel.index('void DroneLinkChannel::subscribe(',start)
            (tree/'actual_channel.inc').write_text(channel[start:end])
            loader=str(Path(directory)/'loader-test')
            subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-Wall','-Wextra','-fsanitize=address,undefined','-I',str(tree),str(tree/'droneModules/CloudTelemetryModule.cpp'),str(ROOT/'tests/cloud_loader.cpp'),'-o',loader],check=True)
            subprocess.run([loader],check=True)

    def test_directed_mesh_request_and_reply(self):
        with tempfile.TemporaryDirectory() as directory:
            tree = Path(directory)
            source = (ROOT / 'src/DroneLinkManager.cpp').read_text()
            bounds = [
                ('bool DroneLinkManager::cloudTxAvailable(', 'bool DroneLinkManager::sendCloudLink('),
                ('void DroneLinkManager::dispatchCloudRequest(', 'bool DroneLinkManager::cloudRequestPending('),
                ('bool DroneLinkManager::publishLocalObservation(', 'bool DroneLinkManager::publish('),
                ('void DroneLinkManager::receiveCloudLink(', 'void DroneLinkManager::receiveDroneLinkMsg(')
            ]
            (tree / 'actual_cloud_mesh.inc').write_text(''.join(
                source[source.index(start):source.index(end, source.index(start))] for start, end in bounds
            ))
            executable = str(tree / 'cloud-mesh-test')
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=address,undefined', '-I', directory, '-I', str(ROOT/'src'), str(ROOT / 'tests/cloud_mesh.cpp'), '-o', executable], check=True)
            subprocess.run([executable], check=True)
