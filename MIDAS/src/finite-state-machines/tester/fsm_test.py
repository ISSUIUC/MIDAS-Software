import argparse
import pathlib
import pandas as pd
import fsm
import math
import matplotlib.pyplot as plt

# Run configuration for sg1.4-telemega-booster.csv for booster data

# ACCEL_STRING = "accel_x"
# TIME_STRING = "time"
# HEIGHT_STRING = "altitude"
# SPEED_STRING = "speed"

# Run configuration for sustainer pysim data

ACCEL_STRING = "accel_x"
TIME_STRING = "time"
HEIGHT_STRING = "pos_x"
SPEED_STRING = "vel_x"

parser = argparse.ArgumentParser()
parser.add_argument("filename", type=pathlib.Path)
parser.add_argument("-s", "--stage", help="boost/sus", default="sus")

args = parser.parse_args()

state_machine = fsm.SustainerFSM()

if args.stage == "boost":
    state_machine = fsm.BoosterFsm()

df = pd.read_csv(args.filename)
packet = fsm.StateEstimate()

state_rows = []
table_data =  [["Time (ms)","State Transition"]]

for i, line in df.iterrows():
    # Accel configuration for booster data
    #packet['acceleration'] = -(float(line[ACCEL_STRING])) / 9.81

    # Accel configuration for sustainer data
    packet['acceleration'] = (float(line[ACCEL_STRING])) / 9.81

    packet['altitude'] = float(line[HEIGHT_STRING])
    packet['current_time'] = float(line[TIME_STRING])
    packet['jerk'] = 0

    if i > 0 and line[TIME_STRING] - df[TIME_STRING][i - 1] != 0:
        packet['jerk'] = float((line[ACCEL_STRING] - df[ACCEL_STRING][i - 1]) / (line[TIME_STRING] - df[TIME_STRING][i - 1])) / 9.81

    packet['vertical_speed'] = float(line[SPEED_STRING])

    state, transition_reason, current_time = state_machine.tick_fsm(packet)

    if transition_reason != "":
        print("\t".join([fsm.FSM_STATE_TO_STRING[state], transition_reason, str(state.value)]))
        table_data.append([current_time, fsm.FSM_STATE_TO_STRING[state]])
    
    state_rows.append(state.value)

top_state = max(state_rows)

df = pd.read_csv(args.filename)

yticks = [fsm.FSM_STATE_TO_STRING[s] for s in fsm.FSMState]
ynums = [i for i in range(14)]

plt.yticks(ynums, yticks)
plt.xlabel("Time (s)")
plt.plot(df["time"], state_rows)

# Lines for SG1.4 flight events
# plt.vlines(60, 0, top_state, label="Booster Ignition", colors="r", linestyles='--')
# plt.vlines(64, 0, top_state, label="Booster Burnout", colors="b", linestyles='--')
# plt.vlines(65, 0, top_state, label="Sustainer Ignition", colors="g", linestyles='--')
# plt.vlines(70, 0, top_state, label="Sustainer Burnout", colors="c", linestyles='--')

plt.legend()

fig, ax = plt.subplots(figsize = (4,4)) 

table = plt.table(cellText =  table_data, loc = 'center')
table.scale(1.2, 1.2)

ax.axis("off")

plt.show()

