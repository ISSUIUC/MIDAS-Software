import serial
import pandas

import glob
import sys

import hilsimpacket_pb2
import os
import time
import rocketstate_pb2

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

    for port in ports:
        try:
            s = serial.Serial(port)
            #s.close()
            return s
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return None

def csv_to_protobuf(parsed_csv) -> hilsimpacket_pb2.HILSIMPacket:
    hilsim_packet = hilsimpacket_pb2.HILSIMPacket()
    
    hilsim_packet.imu_high_ax = parsed_csv['highg_ax']
    hilsim_packet.imu_high_ay = parsed_csv["highg_ay"]
    hilsim_packet.imu_high_az = parsed_csv["highg_az"]
    hilsim_packet.barometer_altitude = parsed_csv["barometer_altitude"]
    hilsim_packet.barometer_temperature = parsed_csv["temperature"]
    hilsim_packet.barometer_pressure = parsed_csv["pressure"]
    hilsim_packet.imu_low_lsm_ax = parsed_csv["ax"]
    hilsim_packet.imu_low_lsm_ay = parsed_csv["ay"]
    hilsim_packet.imu_low_lsm_az = parsed_csv["az"]
    hilsim_packet.imu_low_lsm_gx = parsed_csv["gx"]
    hilsim_packet.imu_low_lsm_gy = parsed_csv["gy"]
    hilsim_packet.imu_low_lsm_gz = parsed_csv["gz"]
    hilsim_packet.mag_x = parsed_csv["mx"]
    hilsim_packet.mag_y = parsed_csv["my"]
    hilsim_packet.mag_z = parsed_csv["mz"]

    hilsim_packet.imu_low_ax = 0
    hilsim_packet.imu_low_ay = 0
    hilsim_packet.imu_low_az = 0
    hilsim_packet.ornt_roll = 0
    hilsim_packet.ornt_pitch = 0
    hilsim_packet.ornt_yaw = 0
    hilsim_packet.ornt_rollv = 0
    hilsim_packet.ornt_pitchv = 0
    hilsim_packet.ornt_yawv = 0
    hilsim_packet.ornt_rolla = 0
    hilsim_packet.ornt_pitcha = 0
    hilsim_packet.ornt_yawa = 0
    hilsim_packet.ornt_ax = 0
    hilsim_packet.ornt_ay = 0
    hilsim_packet.ornt_az = 0
    hilsim_packet.ornt_gx = 0
    hilsim_packet.ornt_gy = 0
    hilsim_packet.ornt_gz = 0
    hilsim_packet.ornt_mx = 0
    hilsim_packet.ornt_my = 0
    hilsim_packet.ornt_mz = 0
    hilsim_packet.ornt_temp = 0
    return hilsim_packet

# Get first serial port...
if __name__ == "__main__":
    MIDAS = serial_ports()
    if MIDAS == None:
        print("You need to connect MIDAS")
        exit()

    MIDAS.write(b'!')
    print("Reading")
    MIDAS.flush()
    # Read input
    print(MIDAS.read_until())
    print(MIDAS.read_until())
    print(MIDAS.read_until())
    print(MIDAS.read_until())
    # Read the csv
    csv = pandas.read_csv(os.path.dirname(os.path.abspath(sys.argv[0])) + "/flight_computer.csv", index_col=0)
    for index, line in csv.iterrows():
        packet = csv_to_protobuf(line)
        data = packet.SerializeToString()
        # Encode the length of package in 2 bytes and then output the the information
        MIDAS.write([len(data)])
        print("writing {} bytes".format(len(data)))
        MIDAS.write(data)
        time.sleep(0.01)
        size = int.from_bytes(MIDAS.read())
        r = MIDAS.read(size)
        state = rocketstate_pb2.RocketState()
        state.ParseFromString(r)
        print(state.rocket_state)
        #output = MIDAS.read_all()
        #print(output)
