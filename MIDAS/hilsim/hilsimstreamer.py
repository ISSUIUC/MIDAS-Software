import serial

ser = serial.Serial(
    port='COM4',         
    baudrate=9600,       
    timeout=1            
)

file = open(r"C:\Users\sharm\Downloads\data10.launch", "rb")
SIZES = { 1: 12, 2: 12, 3: 12, 4: 20, 5: 4, 6: 20, 7: 12, 8: 84, 9: 24, 10: 4, 11: 40, 12: 9 }

test_list=file.read(4)
print(''.join(format(x, '02x') for x in test_list))
while True:
    tag = file.read(4)
    if not tag:
        break 
    
    tag = int.from_bytes(tag, byteorder='little')
    
    
    timestamp = file.read(4)  

    print(int.from_bytes(timestamp, byteorder='little'))
    print(tag)
    if tag in SIZES:
        size = SIZES[tag]
        print(size)
        
        data = file.read(size)
        print(data)

        ser.write(tag.to_bytes(4, byteorder='little'))   
        # ser.write(size.to_bytes(4, byteorder='little'))   
        ser.write(data)
    else:
        raise ValueError(f"Unknown tag: {tag}")

ser.close()
