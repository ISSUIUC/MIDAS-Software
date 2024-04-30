import json
import serial
import time
import random
import math
import paho.mqtt.publish as publish
import datetime

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
# com_in = ["COM16", "COM17", "COM19", "C"]

s_time = time.time()

class Sim():
    def __init__(self) -> None:
        self.__alt = 0
        self.__vel = 0
        self.__acc = 0
        self.__time = 0

        self.__launchdelay = 5
        self.__burntime1 = 5
        self.__burntime2 = 12
        self.__burnaccel1 = 120
        self.__burnaccel2 = 70
        self.__delay = 2
        self.__s = datetime.datetime.now().timestamp()
        self.__start = datetime.datetime.now().timestamp()

    def getaccel(self, fltime):

        if(fltime < self.__launchdelay):
            # before launch
            return 0

        if(fltime < self.__launchdelay + self.__burntime1):
            return self.__burnaccel1

        if(fltime < self.__launchdelay + self.__burntime1 + self.__delay):
            return -9.8

        if(fltime < self.__launchdelay + self.__burntime1 + self.__delay + self.__burntime2):
            return self.__burnaccel2

    def step(self):
        dt = datetime.datetime.now().timestamp() - self.__s
        self.__s = datetime.datetime.now().timestamp()
        fltime = datetime.datetime.now().timestamp() - self.__start

        accel = self.getaccel(fltime)

        print(accel)
        self.__vel += accel * dt
        self.__alt ++ self.__vel * dt





    def get_alt(self):
        return self.__alt
    
    def get_accel(self):
        return self.__acc
    
    def get_vel(self):
        return self.__vel


def get_packet(sim: Sim):
    delta_s = time.time() - s_time - 1
    # edits packet based on elapsed time to simulate ascent!!
    packet = json.load(open("./telem_packet.json"))
    packet['value']['barometer_altitude'] = sim.get_alt()
    packet['value']['altitude'] = sim.get_alt()
    packet['value']['latitude'] = (math.sin(delta_s / 5) * 80)
    packet['value']['longitude'] = (math.sin(delta_s / 3) * 180)
    packet['value']['highG_ax'] = sim.get_accel()
    packet['value']['highG_ay'] = 0.5
    packet['value']['highG_az'] = 0.2
    packet['value']['battery_voltage'] = math.fabs(math.sin(delta_s/5) * 14)
    packet['value']['tilt_angle'] = math.fabs(math.sin(delta_s/3) * 180)
    packet['value']['FSM_state'] = math.floor(math.fabs(math.sin(delta_s/3) * 12))
    packet['value']['RSSI'] = (math.sin(delta_s/3) * 75) - 75
    packet['value']['sat_count'] = math.floor(math.fabs(math.sin(delta_s/3) * 12))



    # print(packet['BNO_YAW'])
    return packet


ports = [serial.Serial(c, write_timeout=3) for c in com_in]

def enc(dict):
    json_s = json.dumps(dict) + "\n"
    return json_s.encode("ascii")

print("wait 1s")
time.sleep(1)
print("Start sending:")

i = 0
s = Sim()

while True:


    for port in ports:
        write_in = port.read_all().decode()
        if(len(write_in) > 0):
            print("READ:", port.name, write_in)


    
    packet = enc(get_packet(s))
    p = ports[i]
    p.write(packet)
    print(f"Sent packet to port {com_in[i]}")
    i = (i + 1) % len(ports)
    time.sleep(0.3)

    s.step()

    # time.sleep(0.1)
    # p = ports[i]
    # p.write(packet)
    # print(f"Sent packet to port {com_in[i]}")
    # i = (i + 1) % len(ports)


    pass