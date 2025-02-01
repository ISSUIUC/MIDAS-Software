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

"""
This function takes a sensor's discriminant (which is basically an ID)
and returns all the data that needs to be saved

That data can take 4 forms:

string column name: name of a column in the given csv
None: a field that does not matter/ is not in the csv
1: a boolean that needs to be padded to 4 bytes
0: a 1 byte value that does not need to be padded

These cases are very specific to the current csv/ MIDAS configuration,
but can be changed for different configurations
"""
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
            return ["kalman_pos_x", "kalman_pos_y", "kalman_pos_z", "kalman_vel_x", "kalman_vel_y", "kalman_vel_z", "kalman_accel_x", "kalman_accel_y", "kalman_accel_z", "baro_alt"]
        case ReadingDiscriminant.ID_PYRO:
            return [0, 0, 0, 0, 0, 0, 0, 0, 0]

"""
Takes a csv line and returns a list of each of the sensor packets
that will be written as bytes
"""
def line_to_bytes(line):
    all_packets = []
    for disc in ReadingDiscriminant:
        current_packet = []
        cols_to_save = discriminant_to_field_names(disc)
        
        # Write the discriminant
        current_packet.append(disc.value.to_bytes(4, 'little'))

        # Write the timestamp as an integer
        current_packet.append(int(float(line['time']) * 1000).to_bytes(4, 'little'))

        for col in cols_to_save:
            # Writes a 4 byte float
            if col is None:
                current_packet.append(struct.pack('f', 0))
            # Writes a boolean 1 with 4 bytes padding
            elif col == 1:
                current_packet.append(col.to_bytes(4, "little"))
            # Writes a boolean 0 with no padding
            elif col == 0:
                current_packet.append(col.to_bytes(1, "little"))
            # Writes the value of the column as a float
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
buf = 0x00

with open(out_file, 'wb') as f:
    f.write(num.to_bytes(4, 'little'))
    for row in all_rows:
        for packet in row:
            for data in packet:
                f.write(data)
