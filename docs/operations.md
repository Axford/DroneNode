# DroneNode operations

Status: source-derived; runtime/hardware not verified for this documentation baseline. Reviewed: 2026-11-04.

1. Install PlatformIO with the Arduino ESP32 toolchain. [platformio.ini](../platformio.ini) targets `esp32doit-devkit-v1`, pins Espressif32 6.3.2 and lists library dependencies.
2. Review node-specific configuration scripts/Wi-Fi settings and upload/monitor port before programming. Never copy real Wi-Fi credentials into shared documentation.
3. Build with `pio run`; program a connected device with `pio run --target upload`. Configure filesystem upload appropriately (`pio run --target uploadfs`) and inspect serial output at 115200 baud. Board/filesystem/library compatibility must be verified on the actual device.
4. The post-build hook copies firmware and commit marker to sibling `DroneLinkServer/public/firmware/`, including on build. Preserve the intended destination before running builds; a build is not confined to this repo's output directory.
5. Validate module discovery/telemetry through Server and Datastore; validate command/readback only once M2 is implemented. Keep physical actuators in a controlled setup for write tests.

Project-owned firmware conformance/hardware regression automation is not yet established. M1 adds codec fixtures and records representative hardware/captured traffic. Vendored library tests are not equivalent. Verify docs with `python3 ../DroneLinkDatastore/tools/check_docs.py`.
