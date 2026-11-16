"""PlatformIO hook: builds and uploads never publish to another checkout.

Firmware/metadata handoff is explicit through tools/release.py. PlatformIO
retains the application image in its normal project-local build directory.
"""

# The TLS-enabled application retains the existing OTA slot layout. Link-time
# optimization must be enabled on the link driver as well as compilation.
from pathlib import Path

Import("env")
env.ProcessUnFlags("-fno-lto")
# A single LTO partition keeps the serialized ESP32 binary within its slot.
env.AppendUnique(LINKFLAGS=["-flto", "-flto-partition=one"])
# GCC's plugin-aware archiver preserves LTO symbol indexes in static libraries.
env.Replace(AR="xtensa-esp32-elf-gcc-ar", RANLIB="xtensa-esp32-elf-gcc-ranlib")
# Dependency builders cloned the platform environment before this post hook.
for builder in env.GetLibBuilders():
    builder.env.Replace(AR="xtensa-esp32-elf-gcc-ar", RANLIB="xtensa-esp32-elf-gcc-ranlib")


# The ELF usage check omits the serialized ESP32 image headers, alignment and
# checksum. Check the actual file against the OTA app partition before use.
def check_application_image(source, target, env):
    image = Path(target[0].get_abspath())
    bytes_on_flash = image.stat().st_size
    slot_size = int(env.BoardConfig().get("upload.maximum_size"))
    if bytes_on_flash > slot_size:
        print(f"Application image exceeds its partition: {bytes_on_flash} > {slot_size} bytes")
        image.unlink()
        return 1

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", check_application_image)
