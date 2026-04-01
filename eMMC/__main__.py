import subprocess
from pathlib import Path
import os
import serial

current_root = Path.cwd()
os.chdir(Path(__file__).absolute().parent)

subprocess.run(["platformio", "run", "-e", "eMMC", "-t", "upload"])

print("Listing connected devices:")
subprocess.run(["platformio", "device", "list"])

port = input("Choose a port name from the above list: ")
with serial.Serial(port, 9600) as ser:
    ser.write(b"<restart>\n")
    start = ser.readline()
    if start != b"eMMC Connected\r\n":
        raise Exception(f"Unexpected message: {start=}")
    while True:
        command = input(">> ")
        if command == "help":
            print("Available commands: ls, help, quit, dump <file>, rmall")
        elif command == "quit":
            break
        elif command == "ls":
            ser.write(b"ls\n")

            text = ser.readline()
            while text != b"" and text != b"<done>\r\n":
                print(text[:-1].decode("utf-8", errors="ignore"))
                text = ser.readline()
        elif command.startswith("dump "):
            file_name = command[5:]
            ser.write(b"dump\n")
            ser.write(file_name.encode("utf-8") + b"\n")

            line = ser.readline()
            if line != b"Success\r\n":
                print(f"Could not read file! (got {line!r})")
                continue

            line = ser.readline()
            print(line.decode("utf-8", errors="ignore")[:-2])

            b0 = ser.read()[0]
            b1 = ser.read()[0]
            b2 = ser.read()[0]
            b3 = ser.read()[0]

            size = (b3 << 24) + (b2 << 16) + (b1 << 8) + b0

            with open(current_root / file_name, "wb") as out_file:
                while size != 0:
                    buf = ser.read(size=min(1024, size))
                    out_file.write(buf)
                    size -= len(buf)
            line = ser.readline()
            if line != b"<done>\r\n":
                raise Exception(f"Fatal error when reading file! (Got {line!r})")
            print(f"Wrote into {current_root / file_name}!")
        elif command == "rmall":
            if input("rmall deletes all files, are you sure? (y/n): ") == "y":
                ser.write(b"rmall\n")
                text = ser.readline()
                while text != b"" and text != b"<done>\r\n":
                    print(text[:-1].decode("utf-8", errors="ignore"))
                    text = ser.readline()
            else:
                print("invalid / no response, try again.")
        elif command == "":
            pass
        else:
            print("Unknown Command")