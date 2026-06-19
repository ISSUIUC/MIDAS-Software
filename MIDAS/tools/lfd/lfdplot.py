"""
python lfdplot.py data8_flight.csv imu.highg_acceleration.ax barometer.altitude

Michael Karpov, 2026
"""
import argparse, sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


def plot_columns(df, columns, time_col="timestamp_ms"):
    fig, axes = plt.subplots(len(columns), 1, sharex=True, figsize=(10, 2.5 * len(columns)))
    if len(columns) == 1:
        axes = [axes]
    t = df[time_col]
    for ax, col in zip(axes, columns):
        ax.plot(t, df[col], linewidth=0.8)
        ax.set_ylabel(col)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel(time_col)
    fig.tight_layout()
    return fig

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", type=Path, help="flight csv from lfdconvert")
    p.add_argument("columns", nargs="+", help="column names to plot")
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    missing = [c for c in args.columns if c not in df.columns]
    if missing:
        print(f"unknown column(s): {missing}\n")
        sys.exit(1)

    plot_columns(df, args.columns)
    plt.show()


if __name__ == "__main__":
    main()
