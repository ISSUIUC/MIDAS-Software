import serial
import serial.tools
import serial.tools.list_ports
import time
import json

device = None
# Look for midas comport
for comport in serial.tools.list_ports.comports():
    if comport.vid == 0x303a:
        # This is an espressif device
        print(comport.name, "is an Espressif device")
        device = comport
        break

if not device:
    print("MIDAS is not connected!")
    exit()
file = open(r"data271.launch", "rb")

# Read the json file
SIZES = { int(k): v for k, v in json.load(open("../struct_sizes.json", 'r')).items() }
print(SIZES)

test_list=file.read(4)
print("Checksum", hex(int.from_bytes(test_list, byteorder='little')))
ser = serial.Serial(
    port=comport.name,         
    baudrate=115200,       
    timeout=None       
)
print(ser.write('!'.encode('ascii')))
print("Magic", ser.read_until('\n'.encode('ascii'))) # Should be our magic
print("Checksum", hex(int(ser.read_until('\n'.encode('ascii'))))) # Should be our magic
counter = 0

start_time = time.perf_counter()
while True:
    tag = file.read(4)
    if not tag:
        break 
    
    tag = int.from_bytes(tag, byteorder='little')
    timestamp = file.read(4)

    # print(tag, int.from_bytes(timestamp, byteorder='little'))

    if tag in SIZES:
        size = SIZES[tag]
        # print(size)
        
        data = file.read(size)
        # print(data)

        ser.write(tag.to_bytes(1, byteorder='little'))   
        # ser.write(size.to_bytes(4, byteorder='little'))
        ser.write(data)
        content = ser.read_until('\n'.encode('ascii'))
        data = bytes.decode(content, encoding="ascii")
        if len(content) != 0:
            # print(content)
            if ("Error") in (data):
                print((content))
            # print(counter, file.tell(), content)
    else:
        raise ValueError(f"Unknown tag: {tag}")
    counter += 1

ser.close()
end_time = time.perf_counter()
print("Done in ", end_time - start_time)
