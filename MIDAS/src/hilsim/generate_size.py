import hilsimpacket_pb2

def main():
    hilsim_packet = hilsimpacket_pb2.HILSIMPacket()
    hilsim_packet.imu_high_ax = 0
    hilsim_packet.imu_high_ay = 0
    hilsim_packet.imu_high_az = 0
    hilsim_packet.barometer_altitude = 0
    hilsim_packet.barometer_temperature = 0
    hilsim_packet.barometer_pressure = 0
    hilsim_packet.imu_low_ax = 0
    hilsim_packet.imu_low_ay = 0
    hilsim_packet.imu_low_az = 0
    hilsim_packet.imu_low_gx = 0
    hilsim_packet.imu_low_gy = 0
    hilsim_packet.imu_low_gz = 0
    hilsim_packet.mag_x = 0
    hilsim_packet.mag_y = 0
    hilsim_packet.mag_z = 0
    print(vars(hilsimpacket_pb2.HILSIMPacket))
    print(len(hilsim_packet.SerializeToString()))

if __name__ == '__main__':
    main()
