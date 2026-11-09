"""PlatformIO hook: builds and uploads never publish to another checkout.

Firmware/metadata handoff is explicit through tools/release.py. PlatformIO
retains the application image in its normal project-local build directory.
"""

# The TLS-enabled application retains the existing OTA slot layout. Link-time
# optimization must be enabled on the link driver as well as compilation.
Import("env")
env.ProcessUnFlags("-fno-lto")
env.AppendUnique(LINKFLAGS=["-flto"])
# GCC's plugin-aware archiver preserves LTO symbol indexes in static libraries.
env.Replace(AR="xtensa-esp32-elf-gcc-ar", RANLIB="xtensa-esp32-elf-gcc-ranlib")
# Dependency builders cloned the platform environment before this post hook.
for builder in env.GetLibBuilders():
    builder.env.Replace(AR="xtensa-esp32-elf-gcc-ar", RANLIB="xtensa-esp32-elf-gcc-ranlib")
