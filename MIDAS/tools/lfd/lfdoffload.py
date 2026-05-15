"""
LFD offload script

Michael Karpov, 2026
"""

import serial
import sys
from enum import IntEnum
import time

p_s = sys.argv[1]

port = serial.Serial(p_s, 115200, timeout=0.1)

class MShellCommand:
    class Result(IntEnum):
        OK = 0
        ERR_UNSPECIFIED = 1
        ERR_INVALID_CMD = 2
        ERR_NO_CMD = 3
        ERR_INVAL_ARGC = 4
        ERR_INVAL_ARGUMENT = 5
        ERR_INVAL_ARG_RANGE = 6
        ERR_INVAL_FSM = 7

    def __init__(self, cmd, long=False):
        self.cmd = cmd
        self.long = long

    def enc(self) -> bytes:
        return f"{self.cmd}\n".encode()

class MShellExecutor:
    def __init__(self, cmd_list: list[MShellCommand], port: serial.Serial):
        self.cmds = cmd_list
        self.port: serial.Serial = port

    class Output:
        def __init__(self, cmd_res: MShellCommand.Result, out: str):
            self.res = cmd_res
            self.out = out

    def get_response(port: serial.Serial) -> Output:
        dat = port.read_until(b"<done> ").decode()[:-len("<done> ")].strip()
        rval = port.read_until(b"\n").decode()
        return MShellExecutor.Output(MShellCommand.Result(int(rval)), dat)


    def run(self):
        for cmd in self.cmds:
            print(f">> {cmd.enc()}")
            self.port.write(cmd.enc())
            out = MShellExecutor.get_response(self.port)
            print(f"<< {out.out}")

            if(out.res != MShellCommand.Result.OK):
                print("ERR: Command failed!")
                return

    def run_single(cmd: MShellCommand, port: serial.Serial) -> Output:
        print(f">> {cmd.enc()}")
        port.write(cmd.enc())
        out = MShellExecutor.get_response(port)
        print(f"<< {out.out}")
        return out

# HILSIM ONLY!
port.write("\n".encode())
time.sleep(0.1)
port.write("&".encode())
time.sleep(2)

e = port.read_all()


MShellExecutor.run_single(MShellCommand("echo 0"), port)

port.write(f"lfd {sys.argv[2]}\n".encode())

i = 0
size_out = 0
debug_buf = bytearray()  # capture first chunk to compare against the .launch on disk

with open(sys.argv[3], "wb+") as outf:
    while True:
        a = port.read()
        outf.write(a)
        if len(debug_buf) < 256 and a:
            debug_buf.extend(a)
            if len(debug_buf) >= 256:
                print(f"\n[raw first {len(debug_buf)} bytes from port]: {bytes(debug_buf)!r}")
        size_out += len(a)
        i += 1
        if i % 100 == 0:
            print(f"Outputting to {sys.argv[3]}... {size_out}/? B", end="\r")

