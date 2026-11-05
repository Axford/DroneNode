# DroneNode operations

Status: source-derived guidance with 2026-11-05 host-build, serial-discovery and disposable-bench programming evidence below. Device runtime/telemetry acceptance remains separate. Reviewed: 2026-11-05.

1. Install PlatformIO with the Arduino ESP32 toolchain. [platformio.ini](../platformio.ini) targets `esp32doit-devkit-v1`, pins Espressif32 6.3.2 and lists library dependencies.
2. Review node-specific configuration scripts/Wi-Fi settings and upload/monitor port before programming. Never copy real Wi-Fi credentials into shared documentation.
3. Build with `pio run`; program a connected device with `pio run --target upload`. Configure filesystem upload appropriately (`pio run --target uploadfs`) and inspect serial output at 115200 baud. Board/filesystem/library compatibility must be verified on the actual device.
4. Builds and uploads retain outputs locally. Export firmware/metadata explicitly through the release commands below; no build/upload hook publishes into Server.
5. Validate module discovery/telemetry through Server and Datastore; validate command/readback only once M2 is implemented. Keep physical actuators in a controlled setup for write tests.

Host packet conformance checks and operator serial-discovery tests exist; a complete device/transport regression pipeline is not yet established. M1 records representative hardware/captured traffic. Vendored library tests are not equivalent. Verify docs with `python3 ../DroneLinkDatastore/tools/check_docs.py`.

## Identify likely serial DroneLink devices

Implemented and exercised on 2026-11-05. [tools/discover_nodes.py](../tools/discover_nodes.py) is a reusable operator/agent CLI for identifying **likely connections carrying DroneLink output**. It does not choose a deployment target, transmit protocol/console commands, program firmware, upload a filesystem or deliberately reset a device. User confirmation is a separate step for the intended testing/deployment. USB bridge metadata and a reported motherboard version do not establish the attached ESP32 model or flash layout; mesh source addresses can belong to remote nodes behind the connection.

### Setup and discovery

Use Python 3.9+ and pyserial 3.5 in an isolated environment. From DroneNode:

```sh
python3 -m venv .pio/validation/venv
. .pio/validation/venv/bin/activate
python -m pip install pyserial==3.5

# Metadata inventory only: opens no ports.
python tools/discover_nodes.py --list --json

# Probe candidate ports at 115200 baud, 8 data bits, no parity, 1 stop bit.
python tools/discover_nodes.py --seconds 5 --json

# An explicit port is useful for an unlisted USB vendor or a known candidate.
python tools/discover_nodes.py --port /dev/cu.usbserial-XXXX --seconds 5 --json
```

Automatic enumeration narrows probes to common USB-UART/board vendor IDs. Bluetooth, debug-console and unrelated monitor-control ports are not automatically opened. VID/PID matches are candidates, never identity evidence. An unlisted vendor may still carry DroneLink; select its exact port explicitly. Probes are sequential, with a 0.2-second read timeout, a maximum 64 KiB sample and a configurable window greater than zero and at most 60 seconds per port. A driver read may finish slightly after that window. Busy/inaccessible/disconnected ports are reported individually and not forced open; other candidates still run.

The reader requests exclusive access, disables software/hardware flow control, sets DTR and RTS inactive before opening, and never sends bytes, breaks or buffer-reset commands. Some OS/USB drivers can transiently change control lines on open/close despite those settings. Consequently discovery is passive at the protocol level but cannot promise that every attached board will remain running without a reset. Use `--list` first on operational equipment; user confirmation is required before any deliberate reboot/bootloader or programming operation. A port already owned by a gateway/monitor should be released through its normal workflow first.

### Interpret the evidence

| Status | Evidence / operator interpretation |
| --- | --- |
| `probable-dronelink-logs` | Specific firmware startup text (`Init DroneLink core`, `Init DroneWire`, `Init DroneFS`) or at least two distinct known `[DLM.*]`, `[DMM.*]`, `[ds.*]` tags. A likely DroneNode console connection; not a healthy-runtime or deployment confirmation. |
| `mesh-stream` | At least two complete `0xFE`-prefixed mesh packets with plausible sizes/types and valid CRC8-SMBUS checksums. A likely DroneLink telemetry connection, which may be a modem/bridge rather than a directly programmable ESP32. |
| `possible-mesh-stream` | One valid packet only; extend observation/review the capture before relying on it. |
| `silent` | No bytes in the bounded window; not proof of a non-DroneLink device. Logging may be silent, disabled or on another UART/baud. |
| `unrecognized` | Bytes received without sufficient recognized evidence. Generic ESP32 boot text or the word `DroneLink` alone is insufficient. |
| `unavailable` | Port could not be opened/read. Review the per-port error, permissions, cable and existing monitor/gateway ownership; do not infer device absence. |

JSON includes evidence counts, recognized markers/tags, a bounded mesh-frame summary, observed reset-reason tokens and logged motherboard versions. It intentionally sets `programming_target_confirmed: false` even after a positive match. These are diagnostic heuristics, not authenticated discovery or a full mesh/link codec conformance claim. A CRC match alone is not proof of origin. The firmware remains authoritative: definitions and transmitter use `SERIAL_TELEMETRY_START_OF_FRAME = 0xFE`, a seven-byte mesh header and CRC8-SMBUS. The old framing comment in `SerialTelemetryModule.h` says `0xFD`/link message; this discrepancy is recorded, not silently adopted or changed by this tool.

Agent exit codes: `0` means at least one likely match (`probable-dronelink-logs` or `mesh-stream`); `1` means no likely match (including silent, unrecognized and one-frame-only observations); `2` means invalid arguments/setup such as missing pyserial; `3` means every attempted port was unavailable. A completed metadata-only `--list` exits `0`, without implying any match. Always inspect individual JSON results; a scan may match one port while another is unavailable. Offline `--analyze` exits `0` for likely evidence and `1` otherwise.

### Preserve evidence and confirm the intended target

```sh
python tools/discover_nodes.py --seconds 5 --json --capture-dir .pio/validation/captures
python tools/discover_nodes.py --analyze .pio/validation/captures/discovery-RUN/port-0.bin
python -m unittest discover -s tests -v
```

Each capture run creates a new subdirectory containing a UTC-dated JSON report and raw samples; earlier runs are not overwritten. Samples may contain private console output, addresses, telemetry or configuration. Keep them in ignored `.pio/validation/` storage, review before sharing, and never commit live samples by default. The regular JSON summary reports recognized evidence without printing arbitrary console text. Offline analysis requires only the standard library and examines at most the first 64 KiB.

After identifying a likely port, present its path, USB description, markers/frame counts, reset evidence and ambiguity to the user. Ask the user to confirm whether it is appropriate for the particular test/deployment and whether firmware/filesystem/configuration must be preserved. Do not automatically launch a gateway, transmit discovery requests, select a board profile, erase flash or chain into upload. Reconfirm the connection after unplug/replug or device replacement; a path or USB serial number is not guaranteed to be unique or permanent. Firmware-only upload and filesystem upload remain separate operations.

### Validation evidence and current bench state

Twelve standard-library tests pass: independent CRC/golden frames, noise/resynchronization/truncation, log confidence, candidate filtering, no-transmit/control-line ordering, failed-open cleanup, offline analysis, distinct evidence directories and firmware-definition checks. A live five-second 115200 8N1 scan identified the CP2102 connection as `probable-dronelink-logs` (`Init DroneWire`, logged motherboard v4). Its sample contains power-on and RTC-watchdog reset messages, with no valid mesh frames. Another USB candidate produced unrecognized output; Bluetooth/debug-console/monitor controls were skipped. This is useful discovery evidence, not telemetry/boot-health/programming acceptance.

The user confirmed the detected CP2102 device as a disposable bench setup for this session. That is session-specific authorisation, not a default for later operators or other detected devices. No device has been flashed or filesystem replaced by discovery.

## Firmware build and manual serial programming — 2026-11-05

PlatformIO 6.2.0 and pyserial 3.5 were installed into a project-local Python 3.12 environment; [tools/requirements-validation.txt](../tools/requirements-validation.txt) pins those direct dependencies. The tested ESP32 platform remains 6.3.2, Arduino ESP32 framework 2.0.9, Xtensa GCC 8.4.0+2021r2-patch5 and esptool 4.5.1. Ranged/floating libraries resolved to the versions recorded locally in `.pio/validation/dependencies.txt`; this validates that resolved environment, not a historical deployed dependency set or an immutable firmware release.

### Build recovery and isolation

The user authorised disabling the long-unused Nunchuck/WiiChuck module. `platformio.ini` omits the unavailable WiiChuck registry dependency and excludes `NunchuckJoystickModule.cpp`; `DroneExecutionManager` no longer includes or instantiates it. The source remains for reference. Existing configurations requesting `Nunchuck` are unsupported by this build and reach the existing unknown-type error path. Other module implementations and protocol definitions are unchanged.

The original `chain+` dependency scanner evaluated a vendored uthash C macro as Python `exit(-1)` and silently terminated before compilation. `lib_ldf_mode = chain` avoids that conditional-preprocessor scanner; the C++ compiler still evaluates source conditionals normally. A complete isolated build then succeeded: RAM 51,616/327,680 bytes (15.8%); application flash 1,276,189/1,310,720 bytes (97.4%). The generated application image is 1,281,952 bytes, SHA-256 `4160e1aa79d81590d0d63ba1fb95eff6fa1ba22ce769a8e083c29e5cf0b66937`. Existing warnings include packed attributes, deprecated library APIs and vendored non-void functions lacking a return; no unrelated firmware fixes were bundled into this validation.

The obsolete OTA-only `--port=3232` flag is now commented out with the optional OTA settings. Serial uploads must not pass that flag. Select serial/monitor ports explicitly rather than relying on the old machine-specific monitor-port default. Do not enable OTA settings for a serial programming test.

The existing post-build hook still writes firmware/version to a sibling Server directory. Validation used an isolated local clone plus its own dummy sibling Server output directory, so the real gateway firmware artifacts were not replaced. To reproduce from DroneNode with the validation environment active:

```sh
python -m pip install -r tools/requirements-validation.txt
export PLATFORMIO_CORE_DIR="$PWD/.pio/validation/core"
validation_root=$(mktemp -d "${TMPDIR:-/tmp}/dronelink-validation.XXXXXX")
git clone --local --no-hardlinks . "$validation_root/DroneNode"
mkdir -p "$validation_root/DroneLinkServer/public/firmware"
# Carry only the reviewed, uncommitted build-recovery changes if present.
git diff --binary -- platformio.ini src/DroneExecutionManager.cpp > "$validation_root/build-input.patch"
if [ -s "$validation_root/build-input.patch" ]; then
  git -C "$validation_root/DroneNode" apply "$validation_root/build-input.patch"
fi
cd "$validation_root/DroneNode"
pio run -e esp32doit-devkit-v1
```

Keep the virtual environment active: the existing build flag invokes `python git_rev_macro.py`. Record the base source revision and any applied patch alongside image hashes and resolved dependency versions. The current `GIT_COMMIT` marker contains the base commit only and does not label dirty worktrees; do not claim an uncommitted image exactly matches that commit. This session's local `.pio/validation/build-manifest.json` and `source-changes.patch` record the distinction. They are not release artifacts. Builds/download caches and private device captures remain ignored local files.

### Manual-boot bench workflow

The user confirmed the discovered CP2102-connected ESP32 as a disposable bench setup. That confirmation applies to this session/device, never automatically to another detected port. On devices needing manual boot entry:

1. Confirm the actual target and whether its flash/filesystem must be preserved. Discovery remains a separate receive-only-at-protocol-level CLI.
2. In the **isolated checkout**, set `board_upload.before_reset = no_reset` and `upload_speed = 115200` in the selected environment before building/programming. These are session-specific settings, not blanket defaults for all nodes. Rebuild after changing the environment; do not assume `nobuild` artifacts survive a configuration change.
3. Hold BOOT, tap RESET/EN, then release BOOT. Once the user reports ready, upload directly; do not open an intervening monitor/probe or reset the board before the upload.
4. Run `pio run -e esp32doit-devkit-v1 --target upload --upload-port /dev/cu.usbserial-XXXX` with the explicitly confirmed port. Keep normal uploader verification enabled. Do not force voltage, flash-layout or eFuse overrides.
5. Require a successful uploader result and image/hash verification, then inspect startup at 115200 8N1. Firmware upload does not include an `uploadfs` operation; filesystem/configuration changes require a separate reviewed step. A successful compile/upload does not establish healthy module execution or telemetry parity.

The loader identifies the current bench chip as ESP32-D0WDQ6-V3, revision 3.0. Earlier automatic-reset and standalone flash-ID attempts returned `ff/ffff` with unknown capacity; one later upload received no serial data. A fresh user-entered manual boot followed immediately by a direct `no_reset` upload succeeded in 77.43 seconds. The tool wrote the 1,281,952-byte application image and reported `Hash of data verified`; the overall PlatformIO result was SUCCESS. Bootloader/partition images were part of the standard firmware upload, while no `uploadfs` operation or eFuse change was performed. Startup and telemetry acceptance remain separate. Treat the earlier failed IDs as session observations, not a permanent hardware-fault diagnosis. See [Espressif's boot-mode guidance](https://docs.espressif.com/projects/esptool/en/release-v4/esp32/advanced-topics/boot-mode-selection.html) for control-line/manual-boot behavior; suspected strap/power problems require actual hardware evidence, not permanent eFuse changes guessed from a console sample.

### Discovery evidence and limits

The finished CLI was exercised on the same bench connection and replays its private saved samples. An initial five-second sample contained `Init DroneWire` and motherboard v4, correctly producing `probable-dronelink-logs`. A later short read contained generic power-on/RTC-watchdog reset output only, correctly producing `unrecognized` with exit code 1. Earlier evidence is retained, but a short inconclusive sample is not promoted to identity merely because the path matched previously. Twelve automated tests pass. Representative mesh telemetry, query captures and healthy startup acceptance are separate from this diagnostic detection evidence.

### Post-programming telemetry evidence — 2026-11-05

The successful upload was followed by a 20-second serial observation (2,786 bytes): all three startup markers appeared, Management/UDP/NMEA modules were configured, setup completed, and the UDP interface started as node 11. No watchdog-reset message appeared in that window. This establishes the observed startup path only, not sensor, actuator or long-duration reliability. No filesystem image was uploaded.

A separate 20-second **passive** UDP observation on port 8007, filtered to the bench node's logged IP, received four mesh HELLO packets; all four passed length and CRC8-SMBUS checks. To capture parameter traffic, a separate bounded diagnostic exchange used the existing gateway encoders with default gateway address 254 (only node 11 had appeared in the passive sample), a Management-channel subscription and read-only QUERY/NAMEQUERY requests. This is distinct from the reusable discovery CLI, which sends no commands. A future operator must confirm an unused local mesh address before an active capture; passive visibility alone cannot prove address availability across the Domain.

The subscribed capture received 23 mesh packets, all with valid lengths/CRC, including 16 link messages with UINT8, UINT32, CHAR and NAME types. Read-only parameters included build, heap, IP and uptime; NAMEQUERY for parameter 13 returned `uptime`. All five transmitted QUERY/NAMEQUERY requests contained exactly one ignored zero payload byte. No parameter/actuator value was written and no configuration was saved. HELLO, subscription and ACK messages are protocol traffic, not telemetry values; the subscription adds runtime routing/subscription state and is not an authenticated identity or persistence operation. Rebooting a disposable bench node clears that runtime session when required; do not assume a capture tool automatically unsubscribes.

The actual gateway mesh parser accepted all 23 received frames. Both existing gateway JS and frontend TS link codecs decoded and re-encoded all 16 captured link messages identically, agreeing with an independent diagnostic decode. NAME text was compared through the text reader, preserving the known gateway `valueArray()` API difference. This is physical bench UDP/query evidence for the observed cases, not blanket mesh/serial/CRC or full codec parity. No binary serial telemetry, numeric-array boundary packets, framing-helper repair or long-duration/hardware safety acceptance is claimed.

Private session evidence lives under ignored `.pio/validation/`: `build-manifest.json`, `source-changes.patch`, `dependencies.txt`, build/upload logs, uniquely saved serial samples, `bench-udp-capture.json` and `bench-subscribed-udp-capture.json`. The manifest records exact source/image/capture hashes, the dirty base revision and successful upload/capture scope. Raw packets include device/network/configuration values: do not promote them to public fixtures without reviewing and sanitizing selected cases. The next compatibility work can turn those reviewed cases into durable captured-traffic tests; it is not automatically included in SEP-01's already completed software subset.


## SEP-05 local release exports

`python3 tools/release.py export-metadata --output-dir <exports>` produces a full annotated catalog without compiling. `python3 tools/release.py export-firmware --environment esp32doit-devkit-v1 --metadata <metadata.tgz> --output-dir <exports>` performs a fresh build in a temporary clone without a Server sibling, then packages that build's application image/version. Both commands require clean committed inputs. `python3 tools/release.py verify <archive.tgz>` verifies contents and the adjacent checksum. The export includes source/input hashes, exact metadata pin and resolved build environment. Ordinary local builds/uploads can still use uncommitted work.

Use the existing PlatformIO environment (activate its Python environment so the build flag's `python` command is available). `DRONELINK_PIO` can explicitly select the PlatformIO executable; standard PlatformIO cache settings remain explicit operator inputs. No export invokes upload or selects a device. Outputs must go to an explicit directory outside tracked source. Preserve archives and their adjacent `.sha256` files together. Different content gets a different immutable identity; conflicting existing bytes are never overwritten. See [SEP-05](../../DroneLinkDatastore/docs/planning/repository-separation.md#sep-05-proposed-plan--2026-11-05) for Server handoff/selection and remaining acceptance.

Run `python3 -m unittest discover -s tests -p 'test_release*.py'`. The frozen original catalog records all 51 entries. The parser skips source fragments without a catalog `@type`, as the original generator did; unfinished Proa/Nunchuck sources do not acquire new catalog entries. Reviewed annotation-only corrections align PanSerial/PanTilt heading names, ADS1015 max, ODrive state address, TankSteer addresses and LSM9DS1 inheritance with existing source definitions. No runtime constants or module registration changed. Historical build/runbook evidence above predates removal of the sibling-write hook.
