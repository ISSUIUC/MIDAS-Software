import serial
import pandas

import glob
import sys

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def csv_to_protobuf(data):
    pass

# Get first serial port...
if __name__ == "__main__":
    ports = serial_ports()
    if len(ports) == 0:
        print("You need to connect MIDAS")
        exit()
    MIDAS = serial.Serial(ports[0], 9600)
    # Read the csv
    csv = pandas.read_csv("flight_computer.csv")
    #for x in csv
    MIDAS.write()    
