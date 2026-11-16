"""Exercise the actual WiFiManager configuration and event lifecycle on a host."""
from pathlib import Path
import os,subprocess,tempfile,shutil,unittest
ROOT=Path(__file__).resolve().parents[1]
class WifiDnsTests(unittest.TestCase):
    def test_actual_configuration_and_reconnect_dns(self):
        with tempfile.TemporaryDirectory() as work:
            tree=Path(work)
            shutil.copytree(ROOT/'tests/wifi_stubs',tree,dirs_exist_ok=True)
            for name in ['WiFiManager.cpp','WiFiManager.h','DnsOverride.h']:
                shutil.copy2(ROOT/'src'/name,tree/name)
            executable=tree/'wifi-test'
            subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-Wall','-Wextra','-fsanitize=address,undefined','-I',str(tree),'-I',os.environ.get('DRONELINK_ARDUINOJSON_DIR',str(ROOT/'.pio/libdeps/esp32doit-devkit-v1/ArduinoJson/src')),str(tree/'WiFiManager.cpp'),str(ROOT/'tests/wifi_dns.cpp'),'-o',str(executable)],check=True)
            subprocess.run([str(executable)],check=True)
