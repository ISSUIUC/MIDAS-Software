import asyncio
import tkinter as tk
from tkinter import ttk
from bleak import BleakClient, BleakScanner
import threading
import time
import ctypes as ct
import struct

# struct BLEPacket {
#     int32_t lat;
#     int32_t lon;
#     uint16_t gps_alt;
#     uint16_t baro_alt;
#     uint16_t highg_ax; 
#     uint16_t highg_ay; 
#     uint16_t highg_az;
#     uint16_t tilt;
#     uint8_t batt_volt;
#     uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
#     uint32_t pyro; // 8 bit continuity
# };

class BLEStruct(ct.Structure):
    _pack_ = 1
    _fields_ = (('lat', ct.c_int32),
                ('lon', ct.c_int32),
                ('gps_alt', ct.c_uint16),
                ('baro_alt', ct.c_uint16),
                ('highg_ax', ct.c_uint16),
                ('highg_ay', ct.c_uint16),
                ('highg_az', ct.c_uint16),
                ('tilt', ct.c_uint16),
                ('batt_volt', ct.c_uint8),
                ('fsm_callsign_satcount', ct.c_uint8),
                ('pyro', ct.c_uint32)
            )
    
    def __str__(self):
        sb =  f"GPS   Lat: {self.lat}, Lon: {self.lon}, Alt: {self.gps_alt}\n"
        sb += f"BARO  Alt: {self.baro_alt}\n"
        sb += f"KX    x: {self.highg_ax}, y: {self.highg_ay}, {self.highg_az}\n"
        sb += f"ORI   tilt: {self.tilt}\n"
        sb += f"VOLT  {self.batt_volt}\n"
        sb += f"FCS   {self.fsm_callsign_satcount}\n"
        sb += f"PYRO  {self.pyro}\n\n"
        return sb


def vdd_decode_from_bytes(bin_bytes):
    return BLEStruct.from_buffer_copy(bin_bytes)

MIDAS_BLE_SERVICE_UUID = "c4558284-48c6-4808-a4d4-437bacd0b2e2"
MIDAS_BLE_CHAR_UUID = "6e79501b-2bab-4504-9049-1ba734276cd4"

def scan_for_midasble():
    devices = asyncio.run(BleakScanner.discover(timeout=3))
    devices_with_names = [d for d in devices if d.name is not None]
    return [d for d in devices_with_names if "MIDASBLE" in d.name]

class BLECL():
    def __init__(self, dev):
        self.dev = dev
    
    def __enter__(self):
        asyncio.run(self.dev.connect())
        return self.dev

    def __exit__(self, a, b, c):
        asyncio.run(self.dev.disconnect())    


def main():
    LATEST_DEV = []
    while True:
        cmd = input("> ")

        match cmd.split()[0].lower():
            case "help":
                print("help, scan, con <n>, ls")
            case "scan":
                print("Scanning for MIDAS BLE...")
                LATEST_DEV = scan_for_midasble()
                for i, device in enumerate(LATEST_DEV):
                    print(f"[{i}] {device.name}   ({device.address})")
            case "ls":
                for i, device in enumerate(LATEST_DEV):
                    print(f"[{i}] {device.name}   ({device.address})")
            case "con":
                cmd_argv = cmd.split()
                if len(cmd_argv) != 2:
                    print("con requires 2 arguments\n  con <n>")
                    continue

                idx = int(cmd_argv[1])
                print(f"Connecting to #{idx} ({LATEST_DEV[idx]})")

                cl = BleakClient(LATEST_DEV[idx])
                with BLECL(cl) as client:
                    print("Connection successful")

                    while True:
                        time.sleep(1)
                        char_d = asyncio.run(client.read_gatt_char(MIDAS_BLE_CHAR_UUID))
                        data = vdd_decode_from_bytes(char_d)

                        print(str(data))
                
                
                


main()