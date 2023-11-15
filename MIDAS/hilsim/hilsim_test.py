import serial
import pandas

import glob
import sys

import hilsimpacket_pb2
import os
import time

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

def csv_to_protobuf(parsed_csv) -> hilsimpacket_pb2.HILSIMPacket:
    hilsim_packet = hilsimpacket_pb2.HILSIMPacket()
    
    hilsim_packet.imu_high_ax = parsed_csv['highg_ax']
    hilsim_packet.imu_high_ay = parsed_csv["highg_ay"]
    hilsim_packet.imu_high_az = parsed_csv["highg_az"]
    hilsim_packet.barometer_altitude = parsed_csv["barometer_altitude"]
    hilsim_packet.barometer_temperature = parsed_csv["temperature"]
    hilsim_packet.barometer_pressure = parsed_csv["pressure"]
    hilsim_packet.imu_low_ax = parsed_csv["ax"]
    hilsim_packet.imu_low_ay = parsed_csv["ay"]
    hilsim_packet.imu_low_az = parsed_csv["az"]
    hilsim_packet.imu_low_gx = parsed_csv["gx"]
    hilsim_packet.imu_low_gy = parsed_csv["gy"]
    hilsim_packet.imu_low_gz = parsed_csv["gz"]
    hilsim_packet.mag_x = parsed_csv["mx"]
    hilsim_packet.mag_y = parsed_csv["my"]
    hilsim_packet.mag_z = parsed_csv["mz"]

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
        MIDAS.write([len(data)])
        MIDAS.write(data)
        time.sleep(0.01)
        output = MIDAS.read_all()
        print(output)
