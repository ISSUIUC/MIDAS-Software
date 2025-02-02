from matplotlib import pyplot as plt
import pandas as pd

df = pd.read_csv('pyro_data.csv')

    # STATE_IDLE = 0
    # STATE_FIRST_BOOST = 1
    # STATE_BURNOUT = 2
    # STATE_COAST = 3
    # STATE_SUSTAINER_IGNITION = 4
    # STATE_SECOND_BOOST = 5
    # STATE_FIRST_SEPARATION = 6
    # STATE_APOGEE = 7
    # STATE_DROGUE_DEPLOY = 8
    # STATE_DROGUE = 9
    # STATE_MAIN_DEPLOY = 10
    # STATE_MAIN = 11
    # STATE_LANDED = 12 
    # FSM_STATE_COUNT = 13

y_labs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
y_labs_map = ['STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST', 'STATE_FIRST_SEPARATION', 'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY', 'STATE_MAIN', 'STATE_LANDED', 'FSM_STATE_COUNT']

for index, row in df.iterrows():
    if (index > 0 and df.loc[df.index[index]]["a_armed"] != df.loc[df.index[index-1]]["a_armed"] or
        df.loc[df.index[index]]["b_armed"] != df.loc[df.index[index-1]]["b_armed"] or
        df.loc[df.index[index]]["c_armed"] != df.loc[df.index[index-1]]["c_armed"] or
        df.loc[df.index[index]]["d_armed"] != df.loc[df.index[index-1]]["d_armed"] or
        df.loc[df.index[index]]["a_firing"] != df.loc[df.index[index-1]]["a_firing"] or
        df.loc[df.index[index]]["b_firing"] != df.loc[df.index[index-1]]["b_firing"] or
        df.loc[df.index[index]]["c_firing"] != df.loc[df.index[index-1]]["c_firing"] or
        df.loc[df.index[index]]["d_firing"] != df.loc[df.index[index-1]]["d_firing"] or 
        df.loc[df.index[index]]["global_armed"] != df.loc[df.index[index-1]]["global_armed"]
        ):

        plt.axvline(x = df.loc[df.index[index]]["Timestamp"], color = 'red')
        

plt.yticks(y_labs, y_labs_map)

plt.ylim(0, 15)

plt.plot(df['Timestamp'], df['fsmstate'])
plt.show()