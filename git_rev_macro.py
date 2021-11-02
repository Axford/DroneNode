import subprocess

revision = (
    subprocess.check_output(["git", "log", "-n1", "--format=%h"])
    .strip()
    .decode("utf-8")
)
print("-D GIT_COMMIT='\"%s\"'" % revision)

#build_flags =
print("-D __ESP32__")
