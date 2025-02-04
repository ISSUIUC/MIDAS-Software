import subprocess
import sys
import zlib
from pathlib import Path

assert sys.version_info >= (3, 5)

Import("env")

right_version = True
try:
    parts = subprocess.run(["python3", "--version"], capture_output=True).stdout.decode("utf-8")[7:].rstrip().split(".")
except:
    right_version = False
if tuple(int(p) for p in parts) < (3, 9, 5):
    right_version = False

if not right_version:
    raise Exception("Python could not be found or version < 3.9.5")

subprocess.run(["python3", "-m", "pip", "install", "lark"], check=True)

output = subprocess.run(["python3", "-m", "reflect", "src/log_format.h"], check=True, capture_output=True)

with Path("src/log_header.h").open("wb") as checksum_file:
    checksum_file.write(output.stdout)

env.Replace(
    AR="xtensa-esp32s3-elf-gcc-ar",
    RANLIB="xtensa-esp32s3-elf-gcc-ranlib"
)

def extra_http_configuration(env, node):
    path = node.get_path().removeprefix(env["PROJECT_BUILD_DIR"]).replace("\\", "/").removeprefix("/" + env["PIOENV"])

    if path.startswith("/Framework"):
        return env.Object(
            node,
            CCFLAGS=env["CCFLAGS"] + ["-w"]
        )
    elif path.startswith("/src/"):
        return env.Object(
            node,
            CCFLAGS=env["CCFLAGS"] + ["-Wall", "-Wextra", "-Wno-unused-function"]
        )
    else:
        return env.Object(
            node,
            CCFLAGS=env["CCFLAGS"] + "-Wno-cpp"
        )

env.AddBuildMiddleware(extra_http_configuration)
