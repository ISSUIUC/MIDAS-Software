from matplotlib import pyplot as plt
import pandas as pd

df = pd.read_csv('pyro_data.csv')

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
        

plt.plot(df['Timestamp'], df['fsmstate'])
plt.show()