import json
import serial
import time
import random
import math
import paho.mqtt.publish as publish

# com0com setup:
# COM1 <-> COM16
# COM2 <-> COM17
# COM18 <-> COM19
# COM20 <-> COM21


# print("Sleep 1s")
# time.sleep(1)
# publish.single("Control-Sustainer", "payload", hostname="localhost")
# print("sent")


# exit(0)
com_in = ["COM16", "COM17", "COM19", "COM21"]
# com_in = ["COM16"]

s_time = time.time()

def get_packet():
    delta_s = time.time() - s_time - 2
    # edits packet based on elapsed time to simulate ascent!!
    packet = json.load(open("./telem_packet.json"))
    packet['value']['barometer_altitude'] = (delta_s*10)
    packet['value']['gps_altitude'] = (delta_s*10)
    packet['value']['latitude'] = (math.sin(delta_s / 5) * 80)
    packet['value']['longitude'] = (math.sin(delta_s / 3) * 180)
    packet['value']['highG_ax'] = delta_s*0.05
    packet['value']['highG_ay'] = delta_s*0.1
    packet['value']['highG_az'] = (delta_s**2)*0.0001
    packet['value']['battery_voltage'] = math.fabs(math.sin(delta_s/5) * 14)
    packet['value']['tilt_angle'] = math.floor(math.fabs(math.sin(delta_s/3) * 64))
    packet['value']['RSSI'] = (math.sin(delta_s/3) * 75) - 75



    # print(packet['BNO_YAW'])
    return packet


ports = [serial.Serial(c, write_timeout=3) for c in com_in]

def enc(dict):
    json_s = json.dumps(dict) + "\n"
    return json_s.encode("utf-8")

print("wait 1s")
time.sleep(1)
print("Start sending:")

i = 0

while True:
    time.sleep(0.05)
    p = ports[i]
    p.write(enc(get_packet()))
    print(f"Sent packet to port {com_in[i]}")

    i = (i + 1) % len(ports)


    pass