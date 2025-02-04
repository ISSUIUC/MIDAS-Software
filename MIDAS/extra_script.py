import subprocess
import sys
import zlib
from pathlib import Path

assert sys.version_info >= (3, 5)

Import("env")

def find_python():
    if sys.platform == "win32":
        which_name = "where"
    else:
        which_name = "which"

    try:
        pythons = subprocess.run([which_name, "python"], check=True, capture_output=True).stdout.decode("utf-8")
    except:
        return None
    for python_name in pythons.splitlines():
        if "platformio" in python_name:
            continue
        try:
            version_string = subprocess.run([python_name, "--version"], capture_output=True, check=True).stdout.decode("utf-8")
        except:
            continue
        version = tuple(int(part) for part in version_string[7:].rstrip().split("."))
        if version >= (3, 9, 5):
            return python_name
    return None

python_name = find_python()
if python_name is None:
    raise Exception("Python could not be found or version < 3.9.5")
print("Using", python_name)

subprocess.run([python_name, "-m", "pip", "install", "lark"], check=True)

output = subprocess.run([python_name, "-m", "reflect", "src/log_format.h"], check=True, capture_output=True)

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
