import json
import serial
import time
import random
import math

# com0com setup:
# COM1 <-> COM16
# COM2 <-> COM17
# COM18 <-> COM19
# COM20 <-> COM21

com_in = ["COM16", "COM17", "COM19", "COM21"]

s_time = time.time()

def get_packet():
    delta_s = time.time() - s_time - 2
    # edits packet based on elapsed time to simulate ascent!!
    packet = json.load(open("./telem_packet.json"))
    packet['pressure'] = 1000 - (delta_s*10)
    packet['BNO_YAW'] = (math.sin(delta_s/3) * math.pi/2) + math.pi/2
    packet['KX_IMU_ax'] = delta_s*0.05
    packet['RSSI'] = (math.sin(delta_s/3) * 75) - 75

    # print(packet['BNO_YAW'])
    return packet


ports = [serial.Serial(c, write_timeout=1) for c in com_in]

def enc(dict):
    json_s = json.dumps(dict) + "\n"
    return json_s.encode("utf-8")

print("wait 2s")
time.sleep(2.0)
print("Start sending:")

i = 0

while True:
    time.sleep(0.01)
    p = ports[i]
    p.write(enc(get_packet()))
    print(f"Sent packet to port {com_in[i]}")

    i = (i + 1) % len(ports)


    pass