#!/usr/bin/env python3
"""Plotting for FSM SILSIM output CSVs."""

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import mpl_toolkits.axisartist as AA
import pandas as pd

STATE_NAMES = ["SAFE", "PYRO_TEST", "ARMED", "BOOST", "COAST", "DROGUE", "MAIN", "LANDED"]
PYRO_COLORS = ["tab:red", "tab:purple", "tab:orange", "tab:brown"]

def _add_pyro_lines(ax, time_s, df):
    for i in range(4):
        firing_col = f"pyro_{i}_firing"
        consumed_col = f"pyro_{i}_consumed"
        if firing_col not in df.columns:
            continue

        color = PYRO_COLORS[i % len(PYRO_COLORS)]
        firing = df[firing_col].astype(bool)
        consumed = df[consumed_col].astype(bool)

        fired_edges = firing & ~firing.shift(1, fill_value=False)
        for j in fired_edges[fired_edges].index:
            ax.axvline(time_s.iloc[j], color=color, linestyle="-", alpha=0.8, linewidth=1.5,
                       label=f"Pyro {i} fired" if j == fired_edges[fired_edges].index[0] else None)

        consumed_edges = consumed & ~consumed.shift(1, fill_value=False) & ~firing
        for j in consumed_edges[consumed_edges].index:
            ax.axvline(time_s.iloc[j], color=color, linestyle="--", alpha=0.6, linewidth=1.5,
                       label=f"Pyro {i} consumed" if j == consumed_edges[consumed_edges].index[0] else None)


def plot(df, title="FSM SILSIM", save_path=None):
    time_s = df["timestamp_ms"] / 1000.0

    fig = plt.figure(figsize=(16, 8))
    host = host_subplot(111, axes_class=AA.Axes, figure=fig)
    fig.subplots_adjust(left=0.12, right=0.82, top=0.93, bottom=0.08)

    par_state = host.twinx()
    par_accel = host.twinx()
    par_vel = host.twinx()

    par_state.axis["right"] = par_state.new_fixed_axis(loc="left", offset=(-50, 0))
    par_accel.axis["right"] = par_accel.new_fixed_axis(loc="right", offset=(0, 0))
    par_vel.axis["right"] = par_vel.new_fixed_axis(loc="right", offset=(50, 0))

    host.plot(time_s, df["altitude_m"], color="tab:green", linewidth=1.5)
    host.set_xlabel("Time (s)")
    host.set_ylabel("Altitude (m)")
    host.axis["left"].label.set_color("tab:green")
    host.grid(True, alpha=0.2, linestyle="--", color="tab:green")

    par_state.plot(time_s, df["state"], color="tab:blue", linewidth=1.5)
    par_state.set_ylabel("FSM State")
    par_state.set_yticks(range(len(STATE_NAMES)))
    par_state.set_yticklabels(STATE_NAMES, fontsize=8)
    par_state.axis["right"].label.set_color("tab:blue")
    par_state.axis["right"].major_ticklabels.set_color("tab:blue")

    par_accel.plot(time_s, df["acceleration_g"], color="tab:orange", linewidth=0.4, alpha=0.35)
    par_accel.set_ylabel("Accel (G)")
    par_accel.axis["right"].label.set_color("tab:orange")

    par_vel.plot(time_s, df["vertical_speed_mps"], color="tab:cyan", linewidth=0.4, alpha=0.35)
    par_vel.set_ylabel("V. Speed (m/s)")
    par_vel.axis["right"].label.set_color("tab:cyan")

    if "cruise_lockout" in df.columns:
        lockout = df["cruise_lockout"].astype(bool)
        host.fill_between(time_s, 0, 1, where=lockout, transform=host.get_xaxis_transform(),
                          color="tab:red", alpha=0.06)

    _add_pyro_lines(host, time_s, df)

    handles, labels = host.get_legend_handles_labels()
    if handles:
        by_label = dict(zip(labels, handles))
        host.legend(by_label.values(), by_label.keys(), loc="upper right", fontsize=8)

    fig.suptitle(title)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()
