#!/usr/bin/env python3
"""Find ports carrying DroneLink logs/mesh frames; never send commands or program.

A stream identifies a connection, not the attached chip/board or a safe flash target.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time
import tempfile

# Firmware sources: SerialTelemetryModule.h/.cpp, DroneMeshMsg.h,
# DroneLinkManager.cpp (FastCRC8.smbus). No firmware runtime changes.
START_OF_FRAME = 0xFE
HEADER_BYTES = 7
MAX_PAYLOAD_BYTES = 48
KNOWN_MESH_TYPES = set(range(8)) | set(range(10, 18)) | set(range(20, 27))
USB_CANDIDATE_VENDORS = {0x10C4, 0x1A86, 0x0403, 0x067B, 0x2341, 0x303A, 0x239A, 0x2E8A}
LOG_MARKERS = (b'Init DroneLink core', b'Init DroneWire', b'Init DroneFS')


def crc8_smbus(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def inspect_stream(data):
    """Classify a bounded sample. Incomplete/noisy data remains inconclusive."""
    frames = []
    rejected = 0
    pos = 0
    while pos < len(data):
        if data[pos] != START_OF_FRAME or pos + 1 >= len(data):
            pos += 1
            continue
        size = (data[pos + 1] & 0x3F) + 1
        if size > MAX_PAYLOAD_BYTES:
            pos += 1
            continue
        end = pos + 1 + HEADER_BYTES + size + 1
        if end > len(data):
            # A later marker can be real even if an earlier noise marker is incomplete.
            pos += 1
            continue
        packet = data[pos + 1:end]
        kind = packet[6] & 0x3F
        if crc8_smbus(packet[:-1]) != packet[-1] or kind not in KNOWN_MESH_TYPES:
            rejected += 1
            pos += 1
            continue
        frames.append({'tx_node': packet[1], 'source_node': packet[2],
                       'destination_node': packet[4], 'sequence': packet[5], 'type': kind})
        pos = end
    markers = [marker.decode('ascii') for marker in LOG_MARKERS if marker in data]
    tags = sorted(set(re.findall(rb'\[(?:DLM|DMM|ds)\.[A-Za-z]+\]', data)))
    if len(frames) >= 2:
        status = 'mesh-stream'
    elif markers or len(tags) >= 2:
        status = 'probable-dronelink-logs'
    elif frames:
        status = 'possible-mesh-stream'
    elif not data:
        status = 'silent'
    else:
        status = 'unrecognized'
    return {'status': status, 'bytes_read': len(data), 'valid_mesh_frames': len(frames),
            'rejected_candidates': rejected, 'log_markers': markers,
            'log_tags': [tag.decode('ascii') for tag in tags], 'frames': frames[:8],
            'reset_reasons': sorted(set(reason.decode('ascii') for reason in re.findall(rb'rst:0x[0-9a-fA-F]+ \(([A-Z0-9_]+)\)', data))),
            'reported_motherboard_versions': sorted(set(int(v) for v in re.findall(rb'Motherboard v(\d+)', data)))}


def candidate_ports(ports):
    # USB metadata only narrows probes; it never establishes DroneLink identity.
    # Other ports can be selected explicitly with --port.
    return sorted((p for p in ports if p.vid in USB_CANDIDATE_VENDORS), key=lambda p:p.device)


def read_port(port, seconds, serial_factory, clock=time.monotonic, max_bytes=65536):
    """Bounded receive with explicit 115200 8N1, no flow control/reset/write."""
    connection = serial_factory(port=None, baudrate=115200, bytesize=8, parity='N',
                                stopbits=1, timeout=0.2, write_timeout=0.2,
                                xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
    try:
        # Set before open; do not deliberately toggle either line or reset buffers.
        # USB drivers may still transiently change lines when opening a device.
        connection.dtr = False
        connection.rts = False
        connection.port = port
        connection.open()
        deadline = clock() + seconds
        captured = bytearray()
        while clock() < deadline and len(captured) < max_bytes:
            captured.extend(connection.read(min(4096, max_bytes - len(captured))))
        return bytes(captured)
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', action='append', help='Explicit port(s); otherwise probe known USB serial/board vendors')
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--list', action='store_true', help='List port metadata/candidacy without opening devices')
    parser.add_argument('--capture-dir', type=Path, help='Save private raw samples/report locally; never commit them')
    parser.add_argument('--analyze', type=Path, help='Analyze an existing capture without opening hardware')
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds must be greater than zero and at most 60')
    if args.analyze:
        try:
            with args.analyze.open('rb') as capture:
                result = inspect_stream(capture.read(65536))
        except OSError as exc:
            parser.error(f'Cannot read capture: {exc}')
        print(json.dumps(result, indent=2))
        return 0 if result['status'] in ('mesh-stream','probable-dronelink-logs') else 1
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.error('pyserial is required for live discovery; install tools/requirements-validation.txt')
    ports = list(list_ports.comports())
    by_name = {p.device:p for p in ports}
    selected = list(dict.fromkeys(args.port)) if args.port else [p.device for p in candidate_ports(ports)]
    if args.list:
        inventory = [{'port':p.device,'description':p.description,
                      'vid':p.vid,'pid':p.pid,'usb_serial_number':p.serial_number,
                      'candidate':p.device in selected} for p in sorted(ports,key=lambda p:p.device)]
        if args.json:
            print(json.dumps({'opens_ports':False,'ports':inventory},indent=2))
        else:
            for item in inventory:
                print(f'{item["port"]}: {item["description"]} (candidate={item["candidate"]})')
        return 0
    results = []
    capture_dir = None
    if args.capture_dir:
        args.capture_dir.mkdir(parents=True, exist_ok=True)
        capture_dir = Path(tempfile.mkdtemp(prefix='discovery-', dir=args.capture_dir))
    for index, name in enumerate(selected):
        info = by_name.get(name)
        result = {'port': name, 'description': getattr(info,'description',None)}
        try:
            raw = read_port(name, args.seconds, serial.Serial)
            result.update(inspect_stream(raw))
            if capture_dir:
                path = capture_dir / f'port-{index}.bin'
                path.write_bytes(raw)
                result['capture'] = str(path)
        except (serial.SerialException, OSError) as exc:
            result.update(status='unavailable',error=str(exc))
        results.append(result)
    report = {'captured_at_utc':datetime.now(timezone.utc).isoformat(), 'serial_format':'115200 8N1','probe_seconds_per_port':args.seconds,
              'sends_commands':False,'programming_target_confirmed':False,'ports':results,
              'skipped_ports':[p.device for p in ports if p.device not in selected]}
    if capture_dir:
        report['capture_dir'] = str(capture_dir)
        (capture_dir/'discovery.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.json:
        print(json.dumps(report,indent=2))
    else:
        for item in results:
            print(f'{item["port"]}: {item["status"]} ({item.get("bytes_read",0)} bytes, {item.get("valid_mesh_frames",0)} valid frames)')
            if 'error' in item:
                print(f'  {item["error"]}')
        if not results:
            print('No candidate USB serial ports; use --port for an explicit device.')
    if any(item['status'] in ('mesh-stream','probable-dronelink-logs') for item in results):
        return 0
    if results and all(item['status'] == 'unavailable' for item in results):
        return 3
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
