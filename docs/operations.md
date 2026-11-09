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

`python3 tools/release.py export-metadata --output-dir <exports>` produces a full annotated catalog without compiling. `python3 tools/release.py export-firmware --environment esp32doit-devkit-v1 --metadata <metadata.tgz> --output-dir <exports>` performs a fresh build in a temporary clone without a Server sibling, then packages that build's application image/version. Both commands require clean committed inputs; metadata paths come from the Git tree and input bytes/modes are compared with committed blobs even when index flags hide changes. `python3 tools/release.py verify <archive.tgz>` verifies contents and the adjacent checksum. The export includes source/input hashes, exact metadata pin, measured effective board, actual embedded Git marker and resolved build environment. Ordinary local builds/uploads can still use uncommitted work.

Use the existing PlatformIO environment (activate its Python environment so the build flag's `python` command is available). `DRONELINK_PIO` can explicitly select the PlatformIO executable; standard PlatformIO cache settings remain explicit operator inputs. No export invokes upload or selects a device. Outputs must go to an explicit directory outside tracked source. Preserve archives and their adjacent `.sha256` files together. Different content gets a different immutable identity; conflicting existing bytes are never overwritten. See [SEP-05](../../DroneLinkDatastore/docs/planning/repository-separation.md#sep-05-proposed-plan--2026-11-05) for Server handoff/selection and remaining acceptance.

Run `python3 -m unittest discover -s tests -p 'test_release*.py'`. The frozen original catalog records all 51 entries. The parser skips source fragments without a catalog `@type`, as the original generator did; unfinished Proa/Nunchuck sources do not acquire new catalog entries. Reviewed annotation-only corrections align PanSerial/PanTilt heading names, ADS1015 max, ODrive state address, TankSteer addresses and LSM9DS1 inheritance with existing source definitions. No runtime constants or module registration changed. Historical build/runbook evidence above predates removal of the sibling-write hook.

## SEP-07 bench preservation checkpoint — 2026-11-06

The user confirmed the existing connected bench node for [SEP-07](../../DroneLinkDatastore/docs/planning/sep-07-bench-preservation.md). Initial receive-only console observation on the same CP2102 connection found 71 `invalid header: 0xffffffff` messages and ten RTC-watchdog resets in five seconds, with no application startup markers. This is ROM boot failure evidence, not a diagnosed firmware regression or proof of erased flash. The USB connection is the console; earlier binary mesh telemetry was observed over UDP, not this console port.

After user-entered manual boot, esptool 4.5.1 identified ESP32-D0WDQ6-V3 revision 3.0, but the RAM-helper flash probe returned manufacturer/device ff/ffff. A subsequent connection without a fresh manual boot timed out. Following a full power cycle and fresh manual boot, a direct `--no-stub flash_id` probe at 115200 reported manufacturer/device 5e/4016 and 4 MB. Do not force flash-voltage/eFuse settings based on the failed probe. The precise cause of the transient flash inaccessibility remains unresolved.

The validation Python environment contains PlatformIO/pyserial; esptool is supplied by PlatformIO at `.pio/validation/core/packages/tool-esptoolpy/esptool.py`, rather than installed as a top-level Python module. Invoke that script using `.pio/validation/venv/bin/python`. With an explicitly confirmed target in fresh manual boot, use `--chip esp32 --port <confirmed-port> --baud 115200 --before no_reset --after no_reset`. `flash_id` is read-only. A full `read_flash 0 0x400000 <private-backup.bin>` is appropriate only after independently confirming this board's 4 MB capacity. Keep backup bytes and logs ignored/private: flash can contain credentials and node files. Verify completion, length and SHA256 before any write; an incomplete file is not a backup. A 460800-baud backup attempt failed on serial packet corruption; the retry stays at the previously reliable 115200 baud and needs a fresh manual boot.

Firmware programming, healthy startup, node-file restoration and current gateway acceptance remain separately evidenced in the owning plan. Twelve discovery tests pass; historical sanitized query/response fixtures now exercise the actual gateway mesh parser and both link codecs in Datastore's coordinated suite. These tests do not turn the failed initial startup into successful hardware acceptance.

### Verified application programming and preservation

The complete 4,194,304-byte backup succeeded at 115200 baud, SHA256 `4546a574aaac0919d4853ab08b5f81c1d5373bfaf4c3591e9c6ea85677a5e573`. Its bootloader, partition table and application matched the earlier programmed-image hashes exactly. Thus the initial invalid-header output did not establish erased/damaged firmware. A normal RTS reset subsequently reached setup-complete with all three application markers and no invalid headers or watchdog resets over 25 seconds. Detailed raw backup/captures and manifests remain private in `.pio/validation/sep07/`; no restoration rehearsal is claimed.

The selected SEP-05 application has source/build marker `79e82d9`, immutable firmware identity `bfbeac0a6981180ab79a1a4d316a68d496d4ac391ce602ddf6b3f1e098ec561b` and image SHA256 `7ddaaee631bda1adfe690542845a04512f2e60c234beedd9c8a96e61b102f504`. The private backup's partition table confirms app0 at 0x10000 with 0x140000 bytes, sufficient for the selected 1,282,016-byte image. Programming used the existing verified Server prepared release, not an untracked rebuild.

A first `write_flash` attempt after manual EN/BOOT again lost flash communication and never verified. Its log is retained as a failure; do not treat a failed attempt as proof that no sectors changed. Following another user-entered full power cycle/manual boot, the direct 115200-baud `write_flash 0x10000 <verified-firmware.bin>` succeeded in 69.4 seconds and reported `Hash of data verified`, then reset normally. Only the application slot was targeted; no bootloader, partition table, filesystem image or eFuse was written. This board's session observations support using a fresh cold boot when flash access fails; they do not establish a universal recovery rule or the electrical cause.

Post-programming startup again reaches setup-complete with zero invalid-header/watchdog messages over 25 seconds. The actual gateway observes build `79e82d9`; all three original node-file paths, lengths and SHA256 values match their pre-test backups, and the temporary support file is absent. Gateway telemetry, actual browser module inspection and file read/edit/upload/separate-Restart/restoration evidence is recorded in the central plan and Server operations guide. USB-console and UDP results do not establish binary serial telemetry, mesh-wide firmware distribution, other hardware configurations or endurance reliability.


## M1.2 serial software checks — 2026-11-08

Run `python3 -m unittest discover -s tests -v`: 22 tests pass, including a host compiler-backed suite for actual production framing/writer and SerialTelemetry/Console code using narrow platform adapters. The core run enables AddressSanitizer/UndefinedBehaviorSanitizer, tests all mesh payload lengths, shared literal fixtures, sequence wrap, malformed streams, queue loss, priority, partial writes and reentrancy. Module tests cover final-byte delivery, null/failed port, rebind, exact TX and concurrent ordinary console output. These are software checks; DLM's unchanged production CRC path is retained and inspected, not replaced by a stub-derived physical acceptance claim.

The historical catalog fixture remains immutable; the serial counter help description is an explicit reviewed expectation correction. An isolated candidate build passes for esp32doit-devkit-v1 with the established ESP32 platform/toolchain; clean-source exports and physical acceptance are recorded separately in the [owning plan](../../DroneLinkDatastore/docs/planning/m1-contracts-and-telemetry.md#m12-firmware-serial-repair-plan--2026-11-08).

For Serial0 testing, upgrade the gateway first: diagnostic framing activates automatically, with no opt-in flag. Back up the node's current files, enable SerialTelemetry with `port 0`/115200 in a deliberate temporary `config.ini`, verify readback, then restart separately. USB provides the UART bridge and common ground; no extra radio/wires are required for the approved DevKit. Release the host serial handle before programming. Restore the original configuration and verify all original file hashes after the run. This paragraph is a procedure, not a claim that programming or binary serial acceptance passed.


### Approved Serial0 physical acceptance — 2026-11-08

The existing attached DevKit was reprogrammed with the reviewed ad03634 application image at 0x10000 only. The user waived a new full-flash backup and confirmed that this board often reports unreliable flash contents/identity while programming successfully. The original three node files had already been freshly read and hashed; historical full-flash/firmware copies remain preserved. ROM/RAM-helper probes returned ff/ffff or connection errors; these are retained failures, not a diagnosis or evidence of erased flash. A direct standard 115200-baud write with flash mode/frequency/size kept wrote 1,283,936 bytes in 69.5 s and reported hash verification. No force/security bypass, bootloader, partition, filesystem-image or eFuse operation was used. The follow-up 25-second reset capture reached setup with zero invalid-header/watchdog messages, and actual UDP readback confirmed build ad03634 and original files.

A temporary SerialTelemetry module 3 on Serial0/115200 was uploaded with byte-exact readback; uptime stayed 90→90 until separate Restart 90→0. The final named 43.66-second serial run exercised HELLO/subscriptions/read-only QUERY/NAMEQUERY, actual DLM bad-CRC rejection, consecutive no-padding frames, one-byte fragmentation, noise/declarations of 49 and 64 payload bytes, a corrupt-frame diagnostic burst and close/reopen renegotiation. It received 134 valid mesh packets and 81 diagnostic records, with zero host mesh/diagnostic rejections. Module counters changed [sent,RX handoffs,framing rejects] 59/86/0→104/155/2; eight CRC-failure strings and four explicit loss/gap indications were observed. Diagnostic loss under pressure is expected best-effort behavior, not guaranteed per-fault logging. No actuator value was changed.

Prior partial harness runs are retained privately: startup greetings can be lost if sent before telemetry activation, query publications need established subscriptions, repeated ordinary parameter values are rate-limited by priority (up to 8 s), and USB reopen can reset the board. Acceptance uses bounded HELLO/subscription renegotiation and unthrottled NAME queries where appropriate; firmware rate limiting is unchanged. Earlier startup noise can produce parser rejection candidates, distinct from valid runtime records. Do not generalize this short run into boot reliability, universal truncated-frame recovery or endurance evidence.

The original config.ini was restored with exact readback (uptime 30→30), then separately restarted 30→0. All three original paths/lengths/SHA256 values match afterward; build ad03634 remains installed. The actual legacy inspector/physical UDP flow survives gateway stop/restart and browser reload with no page errors; provider calls were stubbed and no cloud deployment/credentials were involved. The [central ledger](../../DroneLinkDatastore/docs/planning/m1-contracts-and-telemetry.md#m12-firmware-serial-repair-plan--2026-11-08) owns exact image/release/capture hashes and gateway companions. Raw evidence is private under `.pio/validation/m1-serial/`.

## M1-F inner-link validation — implementation

The addressed link receive path now calls `DroneLinkMsg::tryParse` before copying or publishing a mesh payload. It accepts exactly 5 header bytes plus 1–16 declared bytes, with complete 4-byte ADDR/UINT32/FLOAT elements. Invalid input leaves the previous message unchanged, increments a rejection counter and emits a rate-bounded diagnostic; existing routing/ACK/retry behavior remains upstream. Valid replacement clears unused payload capacity. Query payload contents, reserved addresses and float encodings remain representable.

`python3 -m unittest discover -s tests` passes 23 tests, including the actual production message implementation compiled with address/undefined sanitizers and independent shared vectors, every type/length combination and rejection atomicity. This is host verification; the M1-F central plan owns isolated export, physical publication and managed rollout acceptance. No physical acceptance is claimed by this source commit.

### M1-F bounded physical acceptance complete — 2026-11-09

Committed guard firmware `c017040` passes the isolated export and attached DevKit serial/UDP/inspector/reconnect gate. Five malformed inner-link cases reject before publication and each subsequent read-only query succeeds; a16-byte nonzero NAMEQUERY remains valid. App0 write hash and on-device build marker verify. Original3 file hashes match after temporary Serial0 configuration restoration and separate restart. The new firmware stays installed with the original configuration. Central M1-F records image/capture hashes, measured slot fit and the board's intermittent programming behavior; this remains bounded bench evidence, excluding other boards/endurance.

## CloudTelemetry G.1 configuration and local verification

CloudTelemetry is absent unless explicitly configured. G.1 implements the device producer only: **do not activate packed uploads against staging until G.2 supplies and verifies `/addPackets`**. No G.1 device programming or managed activation has occurred. The existing USB-connected ESP32 is the intended later bench target, subject to live identity/file/application inventory;4G remains deferred. See the [owning M1-G plan](../../DroneLinkDatastore/docs/planning/m1-g-device-http-telemetry.md).

Illustrative disabled configuration (names must match actual configured modules):

```ini
node = 11
[CloudTelemetry = 20]
  status = 0
  endpoint = "https://staging.dronewright.com"
  domain = "REPLACE_WITH_FIXTURE_DOMAIN"
  apiKey = "REPLACE_WITH_FIXTURE_KEY"
  reportInterval = 10000
  subscriptions = @>Management.*
  subscriptions = @>GPS.location, Hull>4.8
  subscriptions = 12>5.*
```

Repeated subscription lines append and overlapping selections deduplicate. Local symbolic names and lazy peer-node names are supported; peer module/parameter names are not. Malformed/range/oversize settings disable only this module and identify the configuration line without echoing values; valid unavailable targets remain pending. Resolved/pending counters describe address resolution, not proof of live connectivity. Endpoint/Domain/key/subscriptions apply after explicit Upload and separate Restart and survive configuration save, including an open `publish` list. Quoted long strings escape quote/backslash. Domain is limited to200 printable bytes, key512, HTTPS origin256, decoded setting768 and64 selection clauses/4096 selection bytes. The key remains accessible through deliberate local file access; it is never a public packet parameter.

`reportInterval` is uint32 parameter8,1000–86400000ms, default10000. It may be tuned through existing local parameter tools; changes retain pending observations and start a fresh interval after the current cycle. `interval` retains its existing module-loop meaning. Parameter9 reports attempted requests/admitted pipeline counts/failed-or-uncertain requests;10 resolved clauses/pending clauses/new-address drops;11 free heap/minimum heap/worker stack high-water bytes/largest free block;12 max service gap/report duration/last HTTP status;13 captured/coalesced publications/tracked addresses. These are local diagnostics, excluded from this module's own upload. Buffered admission counts are not durable packet delivery or persistence evidence.

Run actual-production policy/framing and module/loader/save/channel harnesses:

```sh
python3 -m unittest discover -s tests -p 'test_cloud_telemetry.py' -v
python3 -m unittest discover -s tests -v
pio run -e esp32doit-devkit-v1
```

Host C++ tests use address/undefined sanitizers and narrow platform adapters; they do not measure real FreeRTOS scheduling or ESP32 TLS resources. The complete suite also creates temporary self-signed certificates with the installed OpenSSL and an owned loopback-only TLS fault server. No test opens a serial device or contacts staging. The operator helper supports explicit controlled faults without automating hardware or deployment:

```sh
python3 tools/cloud_telemetry_acceptance.py serve --bind 127.0.0.1 --port 8443 --cert /private/tmp/test-cert.pem --key /private/tmp/test-key.pem --mode partial
python3 tools/cloud_telemetry_acceptance.py analyze --input /private/tmp/metrics.json
```

`serve` modes are valid/partial/stall/lost/oversize/redirect. A later explicitly chosen LAN bind can support a confirmed bench run; no endpoint, network or device is chosen automatically. Request logs contain only size/count/mode. `analyze` accepts a private array of numeric metric objects and emits only freeHeap/minHeap/stackBytes/serviceGapMs/reportDurationMs/httpStatus summaries. Capture actual request sizes/counts, reconnect/handshake and encoding/complete-report times separately during bench acceptance. This helper is a fault fixture, not proof of the production Server's admission contract.

G.1 final local evidence:27 Python suite tests pass (including sanitizer-backed C++ harnesses and real loopback TLS fixture);38 coordinated protocol checks pass. A clean isolated-cache PlatformIO build fits the unchanged1,310,720-byte application slot:1,274,697 bytes flash and52,544 bytes static RAM. These are link-time sizes, not runtime heap/stack measurements. Initial TLS build exceeded the slot; `-flto` and `-fno-exceptions` now retain all modules and the existing partition layout. Firmware has no C++ exception handlers; required allocations use nothrow/failure paths. The post hook removes the framework's conflicting link flag and selects GCC plugin-aware archivers for both main and cloned library builders. Invalidate old non-plugin archive caches when changing LTO options; a clean isolated build cache established the recorded pass. Existing vendored sensor-library warnings remain. No release archive, hardware claim or deployment is implied; G.3/G.4 own those gates.


## M1-G G.3 clean release checkpoint — 2026-11-09

Clean committed source `cc930ba46baad083a2686313ad5030be4009aee2` now has verified metadata and firmware exports under ignored `.pio/releases/m1-g3-cc930ba/`, each with its adjacent checksum. Metadata identity `bda17326dac65137f15327d8208ae1915e30f1c14a08741c9df3f8f87bb5fb73` contains52 modules: the51 legacy entries with their reviewed annotation corrections and CloudTelemetry. Existing wire/header/catalog addresses are unchanged. Firmware identity `a395f5ad4c9ff4cc6cbfe3c36677826f1108e3d0b4e08df63e8f5643101f50ac` pins that metadata, embeds marker `cc930ba` and carries application SHA256 `831452b8073743b3a021b9c89df32968411d4dc41cf2422beca8c21731d9bd82` (1,281,744 image bytes).

The exporter built a fresh temporary Git clone without a Server sibling or prior build objects. Activate `.pio/validation/venv`, set `PLATFORMIO_CORE_DIR` to the existing validation core, `PLATFORMIO_LIBDEPS_DIR` to the explicitly selected dependency cache and `DRONELINK_PIO` to that environment's `pio`, then use the export/verify commands above. G.3 used PlatformIO6.2.0, ESP32 platform6.3.2, Arduino ESP32 2.0.9 and Xtensa GCC8.4.0+2021r2-patch5, with a fresh clone-local build cache and the existing resolved library cache. Exact library versions and artifact hashes are in the [sanitized evidence](../../DroneLinkDatastore/docs/planning/m1-g3-release-evidence.json); original complete build manifests stay beside the local artifacts. This records the resolved environment, not a claim that floating dependency declarations reproduce identically after a fresh download.

The release links with1,275,977 bytes flash and52,544 bytes static RAM, within the unchanged1,310,720-byte app slot; the packaged binary includes image overhead. These supersede the earlier G.1 candidate sizes for this release. Existing dependency/deprecation warnings remain; no warning repair is included. All27 firmware tests and both archive verifiers pass. No programming, file upload, serial/reset operation or runtime heap/stack/TLS measurement occurred. G.4 owns actual device preservation and acceptance; use these exact verified bytes at its concrete execution gate. Documentation-only follow-up commits do not change this frozen runtime source.

## M1-G G.4 measurement readiness correction

Local `GET /measurements` now exposes numeric diagnostics independently of CloudTelemetry status. The actual Arduino `loop()` entry records cumulative iterations, maximum entry-to-entry gap in milliseconds and gaps strictly greater than100ms. Counters begin after setup and reset on reboot; compare disabled/enabled runs from separate deliberate restarts with the same collector load. These are whole-loop gaps, distinct from CloudTelemetry's existing module-service-gap parameter. No reset/write action is provided by this endpoint.

A bounded64-event history records before/after request, connection, response and complete-report resource snapshots: free/minimum heap, largest8-bit heap block and worker stack high-water bytes. The connection phase is labeled `dnsTcpTlsOrReuse`: the installed secure client's hostname operation includes DNS/TCP/TLS and may reuse a connection. This measures its combined elapsed time and resource boundary; it does not isolate a TLS-only handshake or prove transient largest-block minima between samples. Encode/write chunks instead expose call counts, total microseconds and maximum chunk microseconds, avoiding heap scans on the encoding hot path and preserving event capacity. Subtract successive counters for run totals; chunk counters are uint32 and a fresh-restart bounded20minute run avoids normal accumulation rollover. Complete report/request timing includes instrumentation overhead.

Measurement state uses fixed storage and a short cross-core lock; heap/stack scans, allocation and local response formatting occur outside that lock. The local response contains no endpoint, key, configuration, packet or identity strings. It is excluded from cloud publications. The endpoint inherits the existing local diagnostic surface's access policy and adds a temporary bounded response allocation; resource figures include the instrumented firmware and diagnostic collector overhead. It neither pauses module processing deliberately nor performs network work in the main-loop hook. No true ESP32 timing/resource success is inferred from host tests or static sizes.

Prepare a private output directory, then explicitly run the read-only collector when device activation is approved:

```sh
python3 tools/collect_measurements.py --origin http://192.168.1.32 --seconds 60 --interval 0.5 --output /private/acceptance/new-disabled.jsonl
python3 tools/collect_measurements.py --origin http://192.168.1.32 --seconds 900 --interval 0.5 --output /private/acceptance/new-enabled.jsonl
```

The collector accepts only an explicit local HTTP IP origin, uses only GET, rejects redirects/proxies, caps each response at64KiB, duration at1200seconds and event history at64. It creates a new0600 JSONL file without overwriting previous evidence, retains only whitelisted numeric/phase fields, and reports missing records, observed reboot/counter resets and unreachable samples. Reachability errors produce explicit gaps and collection resumes; malformed/schema-mismatched responses stop with partial evidence retained. It never supplies credentials, changes device parameters, creates a fault or starts itself automatically. First snapshot establishes a baseline and excludes pre-run phase events. Fresh restarts and sequential captures must be recorded; no samples during a disconnect may be invented. If required phase records are missing, rerun that fault/capture before acceptance. Endpoint requests themselves consume resources; use identical polling for disabled/enabled comparison and record its observed impact.

Host checks cover rollover/100ms threshold, history overwrite/ordering, aggregate timings, actual local-endpoint serialization into the collector schema, unknown secret-field removal and outage/resumption with0600 output. Stock DNS/TLS fault setup, actual observations, physical identity, programming preservation and managed activation remain G.4 execution gates. A new clean-source release and updated preview must supersede the original G.3 candidate before programming this correction.

## G.4 controlled DNS and transport faults

`wifi.json` now accepts an optional configuration-only `dnsServer` containing one canonical dotted IPv4 address. Apply through explicit file Upload and separate Restart, like other network-file changes. With the field absent, the original resolver behavior remains: the existing enable-time8.8.8.8 workaround and ordinary DHCP behavior are unchanged. Invalid/null/non-string/unspecified/loopback/multicast DNS settings produce a fixed diagnostic without echoing the setting and retain the existing behavior; they do not prevent Wi-Fi recovery. Existing network/password fields remain compatible. Wi-Fi credentials are no longer printed by the loader or connection task.

```json
{"dnsServer":"192.168.1.50"}
```

The address above is an example, not a selected workstation address. Preserve every existing field/network entry if adding it to an existing file. If wifi.json was originally absent, preserve that fact; a DNS-only file retains the existing default-network selection and must be removed at cleanup. Record its addition and exact bytes privately. The override sets all three installed lwIP resolver slots, including the fallback slot, to the chosen server in a queued lwIP-thread callback after GOT_IP. It also handles an already-connected start and reapplies during the existing10second connected check, including retry after a logged callback scheduling failure. No static IP configuration is introduced. Effective resolver behavior after real DHCP/renewal/reconnect still requires the physical test.

Changing resolver slots does not invalidate cached DNS answers. Use a **new unique `.test` hostname for each DNS case**, and confirm the fixture logs `matched:1` for that request before claiming a DNS result. TTL0 responses avoid fixture-created positive caching; they are not proof that an earlier cache was cleared. Record fixture query/response evidence, elapsed worker cleanup, local measurements and recovery. Never infer a blackhole from merely using a nonexistent hostname, a different TCP port or a lack of successful uploads.

The fault helper supplies the following explicit, foreground-only controls. It never selects an interface automatically, alters DNS/router/firewall configuration, opens a serial port, programs a node or contacts staging. Real LAN binds and privileged port53 use remain part of the later approved execution. Tests bind ephemeral loopback ports only.

```sh
python3 tools/cloud_telemetry_acceptance.py certificates --name run001.cloud-fault.test --output-dir /private/acceptance/new-certificates
python3 tools/cloud_telemetry_acceptance.py dns --bind WORKSTATION_IPV4 --port 53 --name run001.cloud-fault.test --mode answer --answer WORKSTATION_IPV4
python3 tools/cloud_telemetry_acceptance.py serve --bind WORKSTATION_IPV4 --port 8443 --cert /private/acceptance/new-certificates/valid.pem --key /private/acceptance/new-certificates/key.pem --mode valid
```

Replace the placeholder with the independently confirmed LAN address. Configure the node's test endpoint as `https://run001.cloud-fault.test:8443`, temporary Domain text and API key **`G4-DUMMY-ONLY`**. HTTP fixture requests with other keys receive403 without echo/logging. Never point staging credentials at this fixture. The server accepts at most eight connections, bounds body reads at4096 bytes and socket inactivity at6seconds, and closes owned connections/interrupted waits on shutdown. TLS handshakes run in the bounded workers rather than blocking the listener. Certificate generation writes a new private directory/key and refuses existing output directories. The matching self-signed, wrong-host and genuinely expired certificates are test-only; generate fresh matching certificates before a later session because they have one-day validity. The insecure ESP32 should accept them for certificate-authentication purposes; host verified/unverified TLS tests verify fixture distinction only, not ESP32 acceptance.

| Case | Fixture control and expected distinction |
| --- | --- |
| DNS success | `dns --mode answer --answer WORKSTATION_IPV4`; exact single-name IN A answer, TTL0 |
| DNS negative reply | `dns --mode nxdomain`; matching query gets NXDOMAIN, not timeout |
| DNS blackhole | `dns --mode silent`; matching query is counted with rcode-1 and gets no reply |
| TLS handshake stall / rejection | `serve --mode tls-stall --delay 15` / `tls-reject`; TCP accepts, then no handshake progress / deliberately non-TLS response |
| Certificate policy | Select valid.pem, wrong-host.pem or expired.pem; retain dummy credentials |
| HTTP stalls / slow progress | `stall --delay 15` waits before headers; `slow-response --delay 60` spreads body delivery across that total delay |
| Disconnect / redirect | `lost` / `redirect`; no report replay or redirect following |
| Invalid framing / summary | `oversize`, `header-oversize`, `malformed`; bounded rejection |
| Policy / unexpected success | `status-401`, `status-404`, `status-409`, `status-429`, `status-503`, `unexpected-200`; none counts as full202 admission |
| Partial admission | `partial` returns503 with conservative prefix counts; never infer durable storage from this dummy endpoint |

The DNS fixture is a bounded exact-name UDP test resolver, not a recursive/general DNS service. It refuses other names, ignores malformed/compressed/additional-record requests and logs only numeric query/match/rcode counters and whitelisted mode text. In particular it is not a general `dig` endpoint; use `+noedns` if inspecting manually. Restore the original wifi.json/DNS and staging endpoint/key through separate Upload/Restart before managed acceptance.

### TCP connection timeout preparation

A server which accepts TCP then stalls TLS is not a TCP connection timeout. For that separate case, the tool only writes a narrow macOS PF SYN-drop rule:

```sh
python3 tools/cloud_telemetry_acceptance.py tcp-drop-rule --node NODE_IPV4 --workstation WORKSTATION_IPV4 --interface en1 --port 8443 --output /private/acceptance/new-tcp-drop.conf
```

It creates a new0600 file and reports `applied:false`; it never invokes pfctl. The rule matches only incoming IPv4 SYNs from the named ESP32 to the named workstation/test port/interface. At physical execution, first verify actual interface/addresses, a fresh TCP connection (no reused connection), effective filtering and capture SYN/no-response evidence. Applying firewall state requires the explicit G.4 execution approval. Inspect the loaded PF root rules for the expected `com.apple/*` anchor and verify the reserved child `com.apple/dronelink-g4` is empty before using it. The current source workstation's `/etc/pf.conf` declares that wildcard, but that does not establish currently loaded rules. If unavailable/occupied, stop this scenario and revise its control rather than replacing the root ruleset.

After those checks, the manual approved sequence is `sudo pfctl -a com.apple/dronelink-g4 -nf <rule-file>` (syntax preview), `sudo pfctl -a com.apple/dronelink-g4 -f <rule-file>`, and `sudo pfctl -E`, retaining the returned enable token. Cleanup clears only that owned child with `sudo pfctl -a com.apple/dronelink-g4 -F rules` and releases only that token with `sudo pfctl -X <token>`. Verify the child is empty and normal connection recovery works. Never flush root rules/states or globally disable PF. No firewall command was executed during software preparation; actual firewall availability/packet evidence remains an execution gate.

Run the actual WiFiManager loader/event harness using the selected installed ArduinoJson6 include directory: `DRONELINK_ARDUINOJSON_DIR=<ArduinoJson/src> python3 -m unittest discover -s tests -v`. Without that variable the test uses the normal `.pio/libdeps/esp32doit-devkit-v1/ArduinoJson/src`. Tests exercise actual production loading with platform adapters, queued DNS application/default/invalid/reconnect handling and credential-safe logs, plus real loopback DNS/TLS/HTTP exchanges, certificate validation distinctions, worker bounds/shutdown and non-applying TCP rule generation. No new dependency archive or protocol/catalog change is required.


## Bench module: unreliable flash identification

For the CP2102-connected ESP32-D0WDQ6-V3 revision3 bench module, MAC `78:21:84:98:78:18`, the user confirms that flash identification can report incorrectly (including `0xffffff`) while the module can still be programmed successfully. Historical programming/hash verification for this bench target is recorded above. During G.4, ROM download mode and chip/MAC communication succeeded even though identification returned `0xffffff`. Treat that ID and esptool's associated communication warning as unreliable diagnostics for this specific module, not proof that its flash cannot be read or programmed. Do not generalize the quirk to another board.

Use the independently established matching-target capacity/layout and actual partition/application contents, followed by successful programming verification/readback hashes, as acceptance evidence. Preserve existing files/application before replacement where the agreed gate requires it. A false identification result alone must not block attempting the already-authorized non-destructive partition/application reads; a failed or invalid content read is separate evidence and does not silently waive preservation. This quirk does not authorize eFuse changes, voltage forcing, whole-chip erasure or a full-flash write. Boot-strap captures alone do not establish the cause of the misleading ID.
