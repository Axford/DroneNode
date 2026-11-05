"""PlatformIO hook: builds and uploads never publish to another checkout.

Firmware/metadata handoff is explicit through tools/release.py. PlatformIO
retains the application image in its normal project-local build directory.
"""
