import serial
import pandas

import glob
import sys

import hilsimpacket_pb2
import os

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

def csv_to_protobuf(data) -> hilsimpacket_pb2.HILSIMPacket:
    packet = hilsimpacket_pb2.HILSIMPacket()
    packet.imu_high_ax = 1
    packet.imu_high_ay = 2
    packet.imu_high_az = 3

    packet.barometer_altitude = 4
    packet.barometer_temperature = 5
    packet.barometer_pressure = 6

    packet.imu_low_ax = 7
    packet.imu_low_ay = 8
    packet.imu_low_az = 9
    packet.imu_low_gx = 10
    packet.imu_low_gy = 11
    packet.imu_low_gz = 12

    packet.mag_x = 13
    packet.mag_y = 14
    packet.mag_z = 15

    packet.ornt_roll = 16
    packet.ornt_pitch = 17
    packet.ornt_yaw = 18
    return packet

# Get first serial port...
if __name__ == "__main__":
    ports = serial_ports()
    if len(ports) == 0:
        print("You need to connect MIDAS")
        exit()
    print(f"Connecting to {ports[0]}")
    MIDAS = serial.Serial(ports[0], 9600)
    # Read the csv
    csv = pandas.read_csv(os.path.dirname(os.path.abspath(sys.argv[0])) + "/flight_computer.csv")
    print("Connected")
    while True:
        packet = csv_to_protobuf(True)
        data = packet.SerializeToString()
        # Encode the length of package in 2 bytes and then output the the information
        MIDAS.write(len(data).to_bytes())
        MIDAS.write(data)
        print("Wrote data")
        output = MIDAS.read_all()
        print(output)
