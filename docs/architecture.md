# DroneNode architecture

Status: observed source. Reviewed: 2026-11-04.

[main.cpp](../src/main.cpp) constructs DroneSystem and the OTA manager; `setup` initialises them and `loop` runs the system while no OTA update is in progress.

[DroneSystem](../src/DroneSystem.h) owns hardware/service access: filesystem, I/O/serial registration, module and execution managers, Wi-Fi/web services, LEDs and logging. [DroneModule](../src/DroneModule.h) is the module base, with I2CBaseModule for bus-backed devices. DroneModuleManager handles active modules; DroneExecutionManager loads configuration scripts and resolves parameter subscriptions.

[DroneLinkManager](../src/DroneLinkManager.h) and DroneLinkChannel implement pub/sub and mesh interaction. Link message addresses identify node, channel/module and parameter, with typed payloads; the outer mesh envelope handles routing, priorities, ACKs and service messages. See the [shared packet baseline](../../DroneLinkDatastore/docs/contracts/packet.md) rather than duplicating wire definitions here.

`src/droneModules/` contains sensors, actuators, telemetry, navigation/sailing and KiteBot/ODrive modules. `data/` contains example filesystem/configuration scripts. `lib/` vendors libraries; their tests do not constitute a project firmware regression suite.

[Module authoring notes](../src/droneModules/README.md) contain useful legacy guidance, but references to JSON loading/moduleSchema are older than current config-script/metadata tooling. Confirm actual module registration and header annotations against source. Server `parseModules.js` extracts metadata/help, and [post_build.py](../post_build.py) copies firmware/version into sibling Server. These cross-repo generation steps need explicit validation.

## Existing per-node startup configuration

Startup behaviour is configured per DroneNode, as confirmed by the user on 2026-11-05. [DroneSystem](../src/DroneSystem.cpp) selects normal `/config.ini` or safe-mode `/safeMode.ini` configuration, loads it through DroneExecutionManager and completes module setup. [DroneModule::setup](../src/DroneModule.cpp) honours the configured status parameter, disabling a module whose status is zero. Example [arm](../data/arm.txt) and [disarm](../data/disarm.txt) files show enabling/disabling navigation and turn-rate modules, but example files should not be mistaken for the startup entry path used by this source revision. Multi-node Drone grouping must preserve per-node configuration ownership; it does not imply a new Drone-wide startup switch. Hardware startup/failsafe outcomes still require validation for the installed configuration.

## Existing routing information for cloud polling

[DroneLinkManager](../src/DroneLinkManager.h) maintains per-node routing information including `heard`, `lastHeard`, `nextHop` and interface; [route maintenance](../src/DroneLinkManager.cpp) checks old routes. The user confirmed cloud command polls cover the node itself by default and may explicitly include additional reachable domain node addresses when bridging. Reuse this routing knowledge rather than creating a competing reachability model. Queue polling is an accepted target integration, not an already implemented command transport in these files; advertised reachability remains best effort.

## Accepted target responsibility boundary

See [target architecture](../../DroneLinkDatastore/docs/architecture/target.md) and [ADR-0003](../../DroneLinkDatastore/docs/decisions/0003-repository-boundaries.md) for the accepted target allocation and [SEP](../../DroneLinkDatastore/docs/planning/repository-separation.md) for review/extraction gates. No runtime relocation is implemented.

Accepted on 2026-11-05: this repository owns firmware, hardware interfaces and autonomous execution. This is target ownership; the observed source described above has not yet moved.
