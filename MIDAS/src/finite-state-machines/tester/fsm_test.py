import argparse
import pathlib
import pandas as pd
import fsm

parser = argparse.ArgumentParser()
parser.add_argument("filename", type=pathlib.Path)

args = parser.parse_args()

sustainer_fsm = fsm.SustainerFSM()

df = pd.read_csv(args.filename)

for i, line in df.iterrows():
    packet = fsm.StateEstimate()
    packet['acceleration'] = float(line['acceleration'])
    packet['altitude'] = float(line['height'])
    packet['current_time'] = float(line['time'])
    packet['jerk'] = 0

    if i > 0 and line['time'] - df['time'][i - 1] != 0:
        packet['jerk'] = float((line['acceleration'] - df['acceleration'][i - 1]) / (line['time'] - df['time'][i - 1]))

    packet['vertical_speed'] = float(line['speed'])

    state, transition_reason = sustainer_fsm.tick_fsm(packet)

    if transition_reason != "":
        print("\t".join([line['state_name'], fsm.FSM_STATE_TO_STRING[state], transition_reason]))
    # input()
