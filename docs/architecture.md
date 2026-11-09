# DroneNode architecture

Status: observed source. Reviewed: 2026-11-04.

[main.cpp](../src/main.cpp) constructs DroneSystem and the OTA manager; `setup` initialises them and `loop` runs the system while no OTA update is in progress.

[DroneSystem](../src/DroneSystem.h) owns hardware/service access: filesystem, I/O/serial registration, module and execution managers, Wi-Fi/web services, LEDs and logging. [DroneModule](../src/DroneModule.h) is the module base, with I2CBaseModule for bus-backed devices. DroneModuleManager handles active modules; DroneExecutionManager loads configuration scripts and resolves parameter subscriptions.

[DroneLinkManager](../src/DroneLinkManager.h) and DroneLinkChannel implement pub/sub and mesh interaction. Link message addresses identify node, channel/module and parameter, with typed payloads; the outer mesh envelope handles routing, priorities, ACKs and service messages. See the [shared packet baseline](../../DroneLinkDatastore/docs/contracts/packet.md) rather than duplicating wire definitions here.

`src/droneModules/` contains sensors, actuators, telemetry, navigation/sailing and KiteBot/ODrive modules. `data/` contains example filesystem/configuration scripts. `lib/` vendors libraries; their tests do not constitute a project firmware regression suite.

[Module authoring notes](../src/droneModules/README.md) contain useful legacy guidance, but references to JSON loading/moduleSchema are older than current config-script/metadata tooling. Confirm actual module registration and header annotations against source. DroneNode [release tooling](../tools/release.py) now extracts catalog metadata and explicitly exports local metadata/firmware archives. [post_build.py](../post_build.py) no longer registers sibling-write actions. Server owns help rendering from the exported catalog; it does not parse firmware source. The release handoff and pending acceptance are tracked in SEP-05.

## Existing per-node startup configuration

Startup behaviour is configured per DroneNode, as confirmed by the user on 2026-11-05. [DroneSystem](../src/DroneSystem.cpp) selects normal `/config.ini` or safe-mode `/safeMode.ini` configuration, loads it through DroneExecutionManager and completes module setup. [DroneModule::setup](../src/DroneModule.cpp) honours the configured status parameter, disabling a module whose status is zero. Example [arm](../data/arm.txt) and [disarm](../data/disarm.txt) files show enabling/disabling navigation and turn-rate modules, but example files should not be mistaken for the startup entry path used by this source revision. Multi-node Drone grouping must preserve per-node configuration ownership; it does not imply a new Drone-wide startup switch. Hardware startup/failsafe outcomes still require validation for the installed configuration.

## Existing routing information for cloud polling

[DroneLinkManager](../src/DroneLinkManager.h) maintains per-node routing information including `heard`, `lastHeard`, `nextHop` and interface; [route maintenance](../src/DroneLinkManager.cpp) checks old routes. The user confirmed cloud command polls cover the node itself by default and may explicitly include additional reachable domain node addresses when bridging. Reuse this routing knowledge rather than creating a competing reachability model. Queue polling is an accepted target integration, not an already implemented command transport in these files; advertised reachability remains best effort.

## Accepted target responsibility boundary

See [target architecture](../../DroneLinkDatastore/docs/architecture/target.md) and [ADR-0003](../../DroneLinkDatastore/docs/decisions/0003-repository-boundaries.md) for the accepted target allocation and [SEP](../../DroneLinkDatastore/docs/planning/repository-separation.md) for review/extraction gates. No runtime relocation is implemented.

Accepted on 2026-11-05: this repository owns firmware, hardware interfaces and autonomous execution. This is target ownership; the observed source described above has not yet moved.

## Workbench integration boundary

The accepted workbench baseline uses a lightweight local Server gateway; direct browser-to-node/WebSocket firmware work is deferred. Reuse existing UDP/serial routing, HTTP diagnostics/file operations and explicit restart. Uploading a config/supporting file does not mean the startup-loaded runtime configuration has changed; keep Upload and Restart separate. Node files are the authoritative editing starting point, while cloud backups are optional by hosting context and never auto-deploy. Preserve/verify these existing node paths during initial cloud changes; modern shared-Client workbench delivery follows that release. No firmware change is implemented by this documentation.

## Iridium command target — later S1

Later S1 satellite command intent is node-initiated polling/retrieval from the Iridium store-and-forward service, followed by unpacking/executing commands. The cloud Server submits to the service without a separate satellite delivery queue/retry loop. Exact modem/service APIs and packing remain implementation work.

## SEP-01 protocol authority and evidence — 2026-11-05

Firmware headers remain authoritative. The only firmware edit in SEP-01 corrects QUERY/NAMEQUERY comments: the packet requires at least one payload byte, ignored by query handlers. Definitions and runtime behavior are unchanged. Datastore's portable protocol package freezes the reviewed headers and relevant message implementation with exact checksums, base revision and the explicit comment amendment; it distributes constants to JS/TS consumers, not to firmware.

The actual `DroneLinkMsg.cpp`, packed structs and 16 independent non-query fixtures compile/run on the host using platform/I/O stubs. This is software layout/message evidence, not ESP32, device or transport validation. Query captures, framing and physical parity remain pending. See [SEP-01 evidence](../../DroneLinkDatastore/docs/planning/sep-01-protocol-compatibility.md).

## Operator serial discovery — 2026-11-05

[tools/discover_nodes.py](../tools/discover_nodes.py) is firmware-development/operator tooling that passively identifies likely serial connections from specific log markers or CRC-checked mesh frames. It changes no firmware/runtime definitions and is not a gateway replacement, transport adapter migration, authenticated identity service or automatic uploader. The [local runbook](operations.md) separates likelihood/evidence from user confirmation of an appropriate test/deployment target and records serial-open/control-line limitations. Automated tests exercise classification and the no-transmit lifecycle; live samples remain in ignored local storage.

## Disabled legacy Nunchuck module — 2026-11-05

The user authorised disabling the unused Nunchuck/WiiChuck integration during build recovery. The PlatformIO source filter excludes `NunchuckJoystickModule.cpp`, the WiiChuck library dependency is removed, and `DroneExecutionManager` no longer includes/instantiates the module. Its source remains as reference; configuration scripts requesting `Nunchuck` now follow the existing unknown-type error path. No wire IDs, packet definitions or other module implementation changed. Build verification and the discovered external-tooling issues are recorded in [operations](operations.md).


## Serial telemetry and shared Serial0 diagnostics

The bounded M1.2 serial implementation uses `SerialTransport.h` for raw-size-checked mesh assembly and fixed-buffer record serialization. SerialTelemetry hands a complete mesh frame to the existing DLM path on the final byte; DLM retains its CRC check before routing. Transport RX counts complete handoffs, not CRC acceptance; rejected counts framing/length errors. Port absence/failed claims and Stream rebinding are guarded.

`DroneConsole` is the application's Print sink. When SerialTelemetry owns Serial0, it automatically frames live logger/direct diagnostic output while sharing complete-record write ownership with mesh TX. Other UARTs retain mesh-only framing and other UART0 owners retain logging suppression. Console commands remain disabled during module ownership. Fixed storage and diagnostic loss are deliberate; no startup replay or delivery guarantee is added. The [shared serial contract](../../DroneLinkDatastore/docs/contracts/packet.md#proposed-serial-diagnostic-extension) owns record bytes, limits and compatibility; frozen protocol releases are unchanged.

## CloudTelemetry — G.1 local implementation

[The M1-G plan](../../DroneLinkDatastore/docs/planning/m1-g-device-http-telemetry.md) owns accepted policy and later integration/bench gates. `CloudTelemetryModule` is an opt-in DroneModule, not a mesh network interface. It subscribes to explicit local/peer publications through existing DLM lifetimes; unresolved peer names stay pending, while remote channels/parameters are numeric. `ConfigurationLine` and generic DroneModule load/save hooks preserve long file-only settings, quoted escaping and repeated subscriptions without putting credentials into packet parameters. Ordinary settings retain their former199-byte limit before the legacy small-index token parsers. Root node configuration remains distinct from failed/unknown module sections.

`TelemetryReport` owns bounded64-address latest NAME/value facets, versioned cycle snapshots, cadence and fair rotation. Publications arriving during a report remain eligible for the next cycle; attempted snapshots retire even on partial/lost responses, unattempted snapshots remain pending. Each future transport gets independent state. The existing channel queue now delivers a selected publication once per internal module even when wildcard and specific subscriptions overlap; distinct identical publications still deliver separately.

A single16KiB-stack worker owns copied HTTP settings, the complete immutable cycle snapshot and a WiFiClientSecure. It never dereferences the DroneModule or filesystem; the owner thread receives completion through release/acquire publication. Disable/lifecycle generation fences prevent old completion from clearing or changing a re-enabled instance. OTA shutdown notification occurs before filesystem unmount. Cancellation waits for safe library return rather than deleting a live TLS task. The worker retains one idle task while configured, closes TLS between report cycles and reuses the connection between successful batches within a cycle.

`CloudTelemetryHttp` streams escaped metadata plus one incremental base64 stream in256-byte chunks; it allocates no full outgoing JSON document or encoded-body String. It bounds response header lines/total and body consumption, supports bounded chunked responses, validates exact202/count/complete responses and never follows redirects or replays failures. The worker uses existing DNS/TCP/TLS libraries with explicit phase timeouts and an elapsed progress budget; the installed resolver can wait about31seconds. No hard five-second total/cancellation bound is claimed. HTTPS certificate verification is deliberately disabled under the accepted G tradeoff; the key is excluded from logs and published telemetry.

G.1 is host/software implementation evidence only. Staging still needs G.2's additive `/addPackets` endpoint and matched activation; installed release exports, ESP32 heap/stack/latency and peer/managed acceptance remain separate. Existing single-packet consumers and protocol bytes are unchanged.


## Local measurement ownership

`FirmwareMeasurements` owns bounded secret-free main-loop and worker phase measurements. Main-loop entry records gaps regardless of module state; the worker observes connection/request/response/report boundaries and aggregates encode/write costs through the existing clock/network seam. A local read-only diagnostic route snapshots the fixed state under a short lock, then samples/formats outside it. No packet constants, module/catalog IDs, cloud payloads, scheduling or admission semantics change. The [operations guide](operations.md#m1-g-g4-measurement-readiness-correction) defines counter interpretation and collector limits; actual ESP32 acceptance remains external evidence.
