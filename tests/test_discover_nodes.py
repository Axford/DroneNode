import importlib.util
import contextlib
import io
import json
import tempfile
from unittest import mock
from pathlib import Path
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('discovery', ROOT/'tools/discover_nodes.py')
discovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(discovery)

# Independent golden bytes cross-checked against the existing gateway CRC implementation.
HELLO1 = bytes.fromhex('fe 04 02 02 00 00 01 00 01 e8 03 00 00 49')
HELLO2 = bytes.fromhex('fe 04 02 02 00 00 02 00 01 e8 03 00 00 2f')


class DiscoveryTests(unittest.TestCase):
    def test_crc_and_two_frames(self):
        self.assertEqual(discovery.crc8_smbus(b'123456789'), 0xf4)
        result = discovery.inspect_stream(b'noise\n'+HELLO1+HELLO2)
        self.assertEqual(result['status'],'mesh-stream')
        self.assertEqual(result['valid_mesh_frames'],2)
        self.assertEqual(result['frames'][0]['source_node'],2)

    def test_one_frame_is_only_possible(self):
        self.assertEqual(discovery.inspect_stream(HELLO1)['status'],'possible-mesh-stream')

    def test_bad_crc_and_incomplete_frames_do_not_identify(self):
        corrupt=HELLO1[:-1]+b'\x00'
        self.assertEqual(discovery.inspect_stream(corrupt+HELLO2[:-2])['valid_mesh_frames'],0)
        self.assertNotEqual(discovery.inspect_stream(b'\xfe\x3f'+b'noise')['status'],'mesh-stream')

    def test_noise_marker_does_not_hide_later_valid_frames(self):
        result=discovery.inspect_stream(b'\xfe\x2f'+HELLO1+HELLO2)
        self.assertEqual(result['valid_mesh_frames'],2)

    def test_logs_need_specific_markers_or_multiple_tags(self):
        self.assertEqual(discovery.inspect_stream(b'generic ESP32 boot\n')['status'],'unrecognized')
        self.assertEqual(discovery.inspect_stream(b'DroneLink maybe\n')['status'],'unrecognized')
        self.assertEqual(discovery.inspect_stream(b'N: [] Init DroneLink core...\r\n')['status'],'probable-dronelink-logs')
        self.assertEqual(discovery.inspect_stream(b'[DLM.s] Creating channel: 1 > 2\n[DMM.sM] Setup: status\n')['status'],'probable-dronelink-logs')
        self.assertEqual(discovery.inspect_stream(b'')['status'],'silent')

    def test_usb_candidates_do_not_treat_monitor_or_bluetooth_as_targets(self):
        ports=[types.SimpleNamespace(device=name,vid=vid) for name,vid in [('monitor',0x043e),('bluetooth',None),('bridge',0x10c4),('arduino',0x2341)]]
        self.assertEqual([p.device for p in discovery.candidate_ports(ports)],['arduino','bridge'])

    def test_receive_never_writes_and_sets_lines_before_open(self):
        events=[]
        class Port:
            def __setattr__(self,key,value):
                events.append((key,value)); object.__setattr__(self,key,value)
            def open(self): events.append(('open',))
            def read(self,size): return HELLO1+HELLO2
            def close(self): events.append(('close',))
            def write(self,*args): raise AssertionError('Discovery must not transmit')
        def factory(**kwargs):
            self.assertEqual(kwargs['baudrate'],115200)
            self.assertEqual((kwargs['bytesize'],kwargs['parity'],kwargs['stopbits']),(8,'N',1))
            self.assertFalse(kwargs['rtscts']); self.assertFalse(kwargs['dsrdtr'])
            self.assertTrue(kwargs['exclusive'])
            return Port()
        ticks=iter([0,0,2])
        raw=discovery.read_port('test',1,factory,clock=lambda:next(ticks))
        self.assertEqual(raw,HELLO1+HELLO2)
        self.assertLess(events.index(('dtr',False)),events.index(('open',)))
        self.assertLess(events.index(('rts',False)),events.index(('open',)))
        self.assertEqual(events[-1],('close',))

    def test_open_failure_closes_handle(self):
        events=[]
        class Port:
            def open(self): raise OSError('busy')
            def close(self): events.append('closed')
        with self.assertRaisesRegex(OSError,'busy'):
            discovery.read_port('test',1,lambda **kwargs:Port())
        self.assertEqual(events,['closed'])


    def test_cli_saved_samples_are_unique_and_json_is_machine_readable(self):
        fake_serial = types.ModuleType('serial')
        fake_serial.Serial = object
        fake_serial.SerialException = type('SerialException',(Exception,),{})
        fake_tools = types.ModuleType('serial.tools')
        port=types.SimpleNamespace(device='test',vid=0x10c4,description='bridge')
        fake_tools.list_ports=types.SimpleNamespace(comports=lambda:[port])
        with tempfile.TemporaryDirectory() as directory:
            reports=[]
            for _ in range(2):
                output=io.StringIO()
                with mock.patch.dict('sys.modules',{'serial':fake_serial,'serial.tools':fake_tools}), mock.patch('sys.argv',['discovery','--json','--capture-dir',directory]), mock.patch.object(discovery,'read_port',return_value=b'Init DroneWire'), contextlib.redirect_stdout(output):
                    self.assertEqual(discovery.main(),0)
                report=json.loads(output.getvalue())
                self.assertFalse(report['programming_target_confirmed'])
                self.assertFalse(report['sends_commands'])
                self.assertEqual(Path(report['ports'][0]['capture']).read_bytes(),b'Init DroneWire')
                self.assertTrue((Path(report['capture_dir'])/'discovery.json').exists())
                reports.append(report)
            self.assertNotEqual(reports[0]['capture_dir'],reports[1]['capture_dir'])

    def test_cli_offline_analysis_does_not_need_pyserial(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'capture.bin'; path.write_bytes(HELLO1+HELLO2)
            output=io.StringIO()
            with mock.patch('sys.argv',['discovery','--analyze',str(path)]), contextlib.redirect_stdout(output):
                self.assertEqual(discovery.main(),0)
            self.assertEqual(json.loads(output.getvalue())['status'],'mesh-stream')

    def test_watchdog_messages_and_board_log_are_evidence_not_board_confirmation(self):
        result=discovery.inspect_stream(b'rst:0x10 (RTCWDT_RTC_RESET),boot:0x33\nI: [] Motherboard v4\nI: [] Init DroneWire...')
        self.assertEqual(result['reset_reasons'],['RTCWDT_RTC_RESET'])
        self.assertEqual(result['reported_motherboard_versions'],[4])
        self.assertEqual(result['status'],'probable-dronelink-logs')

    def test_definition_values_match_firmware(self):
        header=(ROOT/'src/droneModules/SerialTelemetryModule.h').read_text()
        self.assertRegex(header,r'#define\s+SERIAL_TELEMETRY_START_OF_FRAME\s+0xFE\b')
        mesh=(ROOT/'src/DroneMeshMsg.h').read_text()
        self.assertRegex(mesh,r'#define\s+DRONE_MESH_MSG_MAX_PAYLOAD_SIZE\s+48\b')
        impl=(ROOT/'src/DroneLinkManager.cpp').read_text()
        self.assertIn('_CRC8.smbus',impl)


if __name__ == '__main__':
    unittest.main()
