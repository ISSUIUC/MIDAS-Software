import serial
import sys
import time
import math
import random

UINT32_T = 4
FILENAME = "./lunaboo.launch"

com = serial.Serial(sys.argv[1], baudrate=115200)

DISC_MAP = {
    1: 12, # lowg
    2: 12, # highg
    3: 12, # baro
    4: 16, # cont
    5: 8, # voltage
    6: 24, # gps
    7: 12, # mag
    8: 120, # orientation
    9: 24, # lowglsm
    10: 4, # fsmstate
    11: 40, # kalman
    12: 5, # pyro
    13: 8, # camera
}

FILTER = [4, 8, 10, 11, 12, 13] # dont stream these

fp = open(FILENAME, "rb")
fp.read(UINT32_T) # read checksum, discard i dont care

class Entry:
    def __init__(self, ts: int, disc: int, data: bytearray, crc: bytearray):
        self.ts = ts
        self.disc = disc
        self.data = data
        self.crc = crc

def get_size(disc: int):
    if disc in DISC_MAP:
        return DISC_MAP[disc]
    else:
        return 1


def read_entry():
    global fp


    disc = int.from_bytes(fp.read(UINT32_T), byteorder='little')
    ts = int.from_bytes(fp.read(UINT32_T), byteorder='little')

    size = get_size(disc)
    data = fp.read(size)

    assert size > 0, f"empty size: {disc} {ts} {size}"

    if len(data) == 0:
        assert False, "EOF :("
    crc = bytearray([0, 0])

    return Entry(ts, disc, data, crc)

# ROWS_SKIP = 2087693 - 5000
ROWS_SKIP = 2064768 - 5000
ROW_END = 100000000

def cur_millis():
    return math.floor(time.time() * 1000)

def stream_entry(e: Entry):
    global com

    rep = bytearray([b"$"[0], 0, 0, 0, 0, e.disc]) + e.data + e.crc
    com.write(rep)


def stream_data():
    rows_read = 0
    cur_entry = None

    while rows_read < ROWS_SKIP + 1:
        rows_read += 1
        cur_entry = read_entry()

    start_ts = cur_entry.ts
    next_ts = cur_entry.ts
    start_epoch = cur_millis()

    while rows_read < ROW_END:

        time_since_stream_start = cur_millis() - start_epoch
        time_elapsed_in_log = next_ts - start_ts

        # 

        while time_since_stream_start > time_elapsed_in_log:
            rows_read += 1
            cur_entry = read_entry()
            next_ts = cur_entry.ts
            time_elapsed_in_log = next_ts - start_ts

            time_since_stream_start = cur_millis() - start_epoch
            latency = time_since_stream_start - time_elapsed_in_log

            if cur_entry.disc not in FILTER:
                # we may need to skip some entries if we're super behind...
                if latency < random.random()*1000:
                    stream_entry(cur_entry)

            if rows_read % 200 == 0:
                print(time_since_stream_start, time_elapsed_in_log)








    print("DONE!")
    



def main():
    input("Run? Any key works.")
    stream_data()


main()