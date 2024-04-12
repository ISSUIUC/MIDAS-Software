import serial
import time

ptest = serial.Serial("COM15")
print("Waiting:")
while True:
    time.sleep(0.05)
    o = ptest.read_all().decode()

    if(o != ""):
        print(o)


