import serial.tools.list_ports
print([p.name for p in serial.tools.list_ports.comports()])