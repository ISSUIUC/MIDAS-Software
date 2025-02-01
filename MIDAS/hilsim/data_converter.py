import argparse
import pathlib
import enum
import struct
import csv

class ReadingDiscriminant(enum.Enum):
    ID_LOWG = 1
    ID_HIGHG = 2
    ID_BAROMETER = 3
    ID_CONTINUITY = 4
    ID_VOLTAGE = 5
    ID_GPS = 6
    ID_MAGNETOMETER = 7
    ID_ORIENTATION = 8
    ID_LOWGLSM = 9
    ID_FSM = 10
    ID_KALMAN = 11
    ID_PYRO = 12

def discriminant_to_field_names(disc):
    match disc:
        case ReadingDiscriminant.ID_LOWG:
            return ["accel_x", "accel_y", "accel_z"]
        case ReadingDiscriminant.ID_HIGHG:
            return ["accel_x", "accel_y", "accel_z"]
        case ReadingDiscriminant.ID_BAROMETER:
            return [None, None, "baro_alt"]
        case ReadingDiscriminant.ID_CONTINUITY:
            return [None, None, None, None, None]
        case ReadingDiscriminant.ID_VOLTAGE:
            return [None]
        case ReadingDiscriminant.ID_GPS:
            return [None, None, None, None, None, None]
        case ReadingDiscriminant.ID_MAGNETOMETER:
            return [None, None, None]
        case ReadingDiscriminant.ID_ORIENTATION:
            return [1, "ang_pos_y", "ang_pos_x", "ang_pos_z", "ang_vel_y", "ang_vel_x", "ang_vel_z", "ang_accel_y", "ang_accel_x", "ang_accel_z", "accel_x", "accel_y", "accel_z", "imu_gyro_x", "imu_gyro_y", "imu_gyro_z", None, None, None, None, None, "alpha"]
        case ReadingDiscriminant.ID_LOWGLSM:
            return ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z", "accel_x", "accel_y", "accel_z"]
        case ReadingDiscriminant.ID_FSM:
            return [None]
        case ReadingDiscriminant.ID_KALMAN:
            return ["kalman_pos_x", "kalman_pos_y", "kalman_pos_z", "kalman_vel_x", "kalman_vel_y", "kalman_vel_z", "kalman_accel_x", "kalman_accel_y", "kalman_accel_z"]
        case ReadingDiscriminant.ID_PYRO:
            return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def line_to_bytes(line):
    all_packets = []
    for disc in ReadingDiscriminant:
        current_packet = []
        cols_to_save = discriminant_to_field_names(disc)
        
        current_packet.append(disc.value.to_bytes(4, 'little'))

        current_packet.append(struct.pack('f', float(line['time'])))

        for col in cols_to_save:
            if col is None:
                current_packet.append(struct.pack('f', 0))
            elif col == 1:
                current_packet.append(col.to_bytes(4, "little"))
            elif col == 0:
                current_packet.append(col.to_bytes(1, "little"))
            else:
                current_packet.append(struct.pack('f', float(line[col])))

        all_packets.append(current_packet)
    
    return all_packets



parser = argparse.ArgumentParser()
parser.add_argument("infile", type=pathlib.Path)
parser.add_argument("outfile", type=pathlib.Path)

args = parser.parse_args()

in_file = args.infile
out_file = args.outfile

all_rows = []

with open(in_file, 'r') as f:
    csv_reader = csv.DictReader(f)

    for row in csv_reader:
        all_rows.append(line_to_bytes(row))

num = 0xf3626b6a

with open(out_file, 'wb') as f:
    f.write(num.to_bytes(4, 'little'))
    for row in all_rows:
        for packet in row:
            for data in packet:
                f.write(data)
    
