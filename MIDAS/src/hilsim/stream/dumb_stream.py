import serial
import sys
import time
import threading
import struct

# INSTR 1 (REPORT_EN) 

report_1 = bytearray([b"#"[0], 1, 2, 250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

print(report_1, len(report_1))
# sys.exit()

a = serial.Serial(sys.argv[1])


def enable_report(serial, sensor_id: int, interval: int):
    print(f"Enabling report on sensor #{sensor_id}  ({interval}ms)")
    intv = interval.to_bytes(length=4, byteorder='little')
    disc = sensor_id.to_bytes(length=1, byteorder='little')
    rep = bytearray([b"#"[0], 1, disc[0], intv[0], intv[1], intv[2], intv[3], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    serial.write(rep)

def write_raw_pkt(serial):

    accel = struct.pack("<f", 1.234) + struct.pack("<f", 3.21) + struct.pack("<f", 1.3385)
    print("accel raw", accel)

    rep = bytearray([b"$"[0], 0, 0, 0, 0, 1]) + accel + bytearray([176, 177])
    serial.write(rep)

def to_pktdata(arr):
    if(len(arr) < 7):
        return
    
    if(arr[len(arr) - 1] == b"%"[0]):
        arr = arr[:-1]


    
    ts_bin = bytearray([arr[0], arr[1], arr[2], arr[3]])
    etype_bin = bytearray([arr[4]])
    size_bin = bytearray([arr[5]])
    disc_bin = bytearray([arr[6]])
    ts = int.from_bytes(ts_bin, byteorder='little')
    etype = int.from_bytes(etype_bin, byteorder='little')
    size = int.from_bytes(size_bin, byteorder='little')
    disc = int.from_bytes(disc_bin, byteorder='little')
    rest = arr[7:]


    # print right format for each etype
    if etype == 0:
        print(f"[{etype}] {ts} ({size}) - {arr[6:]}")

    if etype == 1:
        print(f"[{etype}] {ts} ({size}) #{disc} - {rest}")

n = 0
def out():
    global n
    while True:
        time.sleep(0.01)
        while a.in_waiting:

            # print(a.read_all())

            d = a.read_until(b"%")
            # n += 1
            # if(n % 100 == 0):
            to_pktdata(d)
            # print(d)
            # print(a.read_all())

p = threading.Thread(target=out, daemon=True)
p.start()
print("START")
time.sleep(1)


def sequence():
    time.sleep(0.5)
    enable_report(a, 1, 50)
    time.sleep(3)
    print("write..")
    write_raw_pkt(a)
    


g = threading.Thread(target=sequence, daemon=True)
g.start()

while True:
    time.sleep(0.5)
    # print(n)