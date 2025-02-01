import serial
import serial.tools
import serial.tools.list_ports
import time
import json

import csv
import os

def write_to_csv(filename, data):
    file_exists = os.path.exists(filename)

    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)

        if not file_exists:
            writer.writerow(['Timestamp', 'fsmstate', 'global_armed', 'a_armed', 'a_firing', 'b_armed', 'b_firing', 'c_armed', 'c_firing', 'd_armed', 'd_firing'])

        writer.writerow(data)

device = None
# Look for midas comport
for comport in serial.tools.list_ports.comports():
    if comport.vid == 0x303a:
        # This is an espressif device
        print(comport.name, "is an Espressif device")
        device = comport
        break

print(device.device)

if not device:
    print("MIDAS is not connected!")
    exit()

# make this a command line argument
file = open(r"data43.launch", "rb")

# Read the json file
SIZES = { int(k): v for k, v in json.load(open("../struct_sizes.json", 'r')).items() }
print(SIZES)

test_list=file.read(4)
print("Checksum", hex(int.from_bytes(test_list, byteorder='little')))
ser = serial.Serial(
    port=comport.device,         
    baudrate=115200,       
    timeout=None       
)
print(ser.write('!'.encode('ascii')))
print("Magic", ser.read_until('\n'.encode('ascii'))) # Should be our magic
print("Checksum", hex(int(ser.read_until('\n'.encode('ascii'))))) # Should be our magic
print("Garbage", ser.read_until('\n'.encode('ascii')))
print("Garbage", ser.read_until('\n'.encode('ascii')))
print("Garbage", ser.read_until('\n'.encode('ascii')))
print("Garbage", ser.read_until('\n'.encode('ascii')))
print("Garbage", ser.read_until('\n'.encode('ascii')))

counter = 0


start_time = time.perf_counter()
prev = None
while True:
    tag = file.read(4)
    if not tag:
        break 
    
    tag = int.from_bytes(tag, byteorder='little')
    timestamp = file.read(4)
    timestamp = int.from_bytes(timestamp, byteorder='little')
    # print(tag, int.from_bytes(timestamp, byteorder='little'))

    if tag in SIZES:
        size = SIZES[tag]
        # print(size)
        
        data = file.read(size)
        # print(data)

        ser.write(tag.to_bytes(1, byteorder='little'))   
        # ser.write(size.to_bytes(4, byteorder='little'))
        ser.write(data)
        content = (ser.read())
        # data = bytes.decode(content, encoding="ascii")
        # if len(content) != 0:
        #     # print(content)
        #     if ("Error") in (data):
        #         print((content))
        if content != prev:
            prev = content
            print(counter, file.tell(), int.from_bytes(content))
        
        previous_pyro_array = [timestamp, int.from_bytes(content), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read()), int.from_bytes(ser.read())]

        write_to_csv("pyro_data.csv", previous_pyro_array)
    else:
        raise ValueError(f"Unknown tag: {tag}")
    counter += 1

ser.close()
end_time = time.perf_counter()
print("Done in ", end_time - start_time)
