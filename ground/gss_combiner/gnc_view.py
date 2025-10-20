import serial
import sys
import threading
import time
import re
import math


class Position:
    px = 0
    py = 0
    pz = 0

class Velocity:
    vx = 0
    vy = 0
    vz = 0

class Acceleration:
    ax = 0
    ay = 0
    az = 0

class Orientation:
    pitch = 0
    roll = 0
    yaw = 0
    tilt = 0

class Matrix:
    def __init__(self, _xdim, _ydim):
        # Converts an eigen .data() stream back to a _x*_y matrix

        self._x = _xdim
        self._y = _ydim

        self._data = []
        for i in range(_xdim):
            self._data.append([])
            for _ in range(_ydim):
                self._data[i].append(0)

    def from_eigen_stream(self, arr, begin_idx) -> int:
        # Patches the data into this matrix, and returns the end index
        n = self._x * self._y
        for i in range(self._x):
            for j in range(self._y):
                self._data[i][j] = arr[begin_idx + j + (i*self._x)]
        return begin_idx + n



# # Test matrix class

# a = Matrix(2, 2)

# c = [1, 2, 3, 4]

# r = a.from_eigen_stream(c, 0)
# print(r)
# print(a._data[1][0])


# sys.exit(0)


class RocketData:
    position = Position()
    velocity = Velocity()
    acceleration = Acceleration()
    raw_accel = Acceleration()
    altitude: float = 0
    orientation = Orientation()
    K_mat = Matrix(9, 4)
    P_k_mat = Matrix(9, 9)
    verify: float = 0

    def from_csvf(self, csvf_split: list[str]):
        try:
            # Validate all values can be converted to float before making any changes
            validated_values = []
            for i in range(18):
                validated_values.append(float(csvf_split[i]))

            # Only update if all validations passed
            self.position.px = validated_values[0]
            self.position.py = validated_values[1]
            self.position.pz = validated_values[2]
            self.velocity.vx = validated_values[3]
            self.velocity.vy = validated_values[4]
            self.velocity.vz = validated_values[5]
            self.acceleration.ax = validated_values[6]
            self.acceleration.ay = validated_values[7]
            self.acceleration.az = validated_values[8]
            self.altitude = validated_values[9]
            self.raw_accel.ax = validated_values[10]
            self.raw_accel.ay = validated_values[11]
            self.raw_accel.az = validated_values[12]
            self.orientation.pitch = validated_values[13] * (180/math.pi)
            self.orientation.roll = validated_values[14] * (180/math.pi)
            self.orientation.yaw = validated_values[15] * (180/math.pi)
            self.orientation.tilt = validated_values[16] * (180/math.pi)
            self.verify = validated_values[17]
        except:
            print("err decode data")

    def from_csvf_mat(self, csvf_split: list[str]):
        try:
            # Calculate total number of values needed
            k_mat_size = self.K_mat._x * self.K_mat._y  # 9x4 = 36
            pk_mat_size = self.P_k_mat._x * self.P_k_mat._y  # 9x9 = 81
            total_values = k_mat_size + pk_mat_size + 1  # +1 for verify

            # Validate all values can be converted to float before making any changes
            validated_values = []
            for i in range(total_values):
                validated_values.append(float(csvf_split[i]))

            # Only update if all validations passed
            # Update K_mat
            idx = 0
            for i in range(self.K_mat._x):
                for j in range(self.K_mat._y):
                    self.K_mat._data[i][j] = validated_values[idx]
                    idx += 1

            # Update P_k_mat
            for i in range(self.P_k_mat._x):
                for j in range(self.P_k_mat._y):
                    self.P_k_mat._data[i][j] = validated_values[idx]
                    idx += 1

            # Update verify
            self.verify = validated_values[idx]
        except:
            print("err decoding mat")



data = RocketData()
def inputprocess():
    global data
    s = serial.Serial(sys.argv[1])
    while True:
        dat_raw = s.read_until()
        strdat = dat_raw.decode().strip()

        if strdat.startswith(";") and strdat.endswith("!"):
            # normal data
            fixed_str = strdat[1:-1]
            split_data = fixed_str.split(",")
            data.from_csvf(split_data)

        if strdat.startswith("&") and strdat.endswith("!"):
            # matrix data
            fixed_str = strdat[1:-1]
            split_data = fixed_str.split(",")
            data.from_csvf_mat(split_data)






# CHATGPT CODE FURTHER ---------------


# rocket_ui_tabs.py
import math
import time
import threading
from collections import deque
from typing import Optional

import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ----------------------------
# Tkinter UI with tabs
# ----------------------------
class RocketDashboard(tk.Tk):
    """
    Live dashboard for RocketData that's being updated by another thread.
    Right side uses a Notebook with:
      - Tab 1 (Kinematics): Position (x,y,z), Velocity (x,y,z), Acceleration (x,y,z)
      - Tab 2 (Orientation & Raw): Orientation (pitch,roll,yaw) and Raw accel (x,y,z)
      - Tab 3 (Matrices): K_mat (9x4) and P_k_mat (9x9)
    Left side shows raw readouts.
    """
    def __init__(self, rocket: RocketData, lock: Optional[threading.Lock] = None,
                 poll_ms: int = 100, history_sec: float = 30.0):
        super().__init__()
        self.title("Rocket Telemetry Dashboard")
        self.geometry("1350x820")
        self.minsize(1100, 720)

        self.rocket = rocket
        self.lock = lock
        self.poll_ms = poll_ms
        self.history_sec = history_sec

        # Style
        self.configure(bg="#111111")
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TLabel", foreground="#EAEAEA", background="#111111", font=("Segoe UI", 10))
        style.configure("Header.TLabel", foreground="#FFFFFF", background="#111111", font=("Segoe UI Semibold", 11))
        style.configure("Value.TLabel", foreground="#FFFFFF", background="#111111", font=("Consolas", 12))
        style.configure("TNotebook", background="#111111")
        style.configure("TNotebook.Tab", font=("Segoe UI", 10))

        # Layout: left = raw readouts, right = graphs in tabs
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        self._build_left_raw_panel()
        self._build_right_tabs()

        # History buffers
        max_pts = max(10, int((1000 / self.poll_ms) * self.history_sec))
        self.t_hist = deque(maxlen=max_pts)

        # Position
        self.px_hist = deque(maxlen=max_pts)
        self.py_hist = deque(maxlen=max_pts)
        self.pz_hist = deque(maxlen=max_pts)

        # Velocity
        self.vx_hist = deque(maxlen=max_pts)
        self.vy_hist = deque(maxlen=max_pts)
        self.vz_hist = deque(maxlen=max_pts)

        # Acceleration (filtered)
        self.ax_hist = deque(maxlen=max_pts)
        self.ay_hist = deque(maxlen=max_pts)
        self.az_hist = deque(maxlen=max_pts)

        # Orientation
        self.pitch_hist = deque(maxlen=max_pts)
        self.roll_hist  = deque(maxlen=max_pts)
        self.yaw_hist   = deque(maxlen=max_pts)

        # Raw accel
        self.rax_hist = deque(maxlen=max_pts)
        self.ray_hist = deque(maxlen=max_pts)
        self.raz_hist = deque(maxlen=max_pts)

        self._t0 = time.time()
        self.after(self.poll_ms, self._poll_and_update)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI BUILDERS ----------
    def _build_left_raw_panel(self):
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsw", padx=12, pady=1)
        for i in range(2):
            left.grid_columnconfigure(i, weight=0)

        def row(lbl: str):
            r = ttk.Label(left, text=lbl, style="Header.TLabel")
            v = ttk.Label(left, text="0", style="Value.TLabel")
            r.pack(anchor="w", pady=(0, 0))
            v.pack(anchor="w")
            return v

        ttk.Label(left, text="Raw Readouts", style="Header.TLabel").pack(anchor="w", pady=(0, 0))

        # Position
        self.lbl_px = row("Position X:")
        self.lbl_py = row("Position Y:")
        self.lbl_pz = row("Position Z:")

        # Velocity
        self.lbl_vx = row("Velocity X:")
        self.lbl_vy = row("Velocity Y:")
        self.lbl_vz = row("Velocity Z:")

        # Acceleration (filtered)
        self.lbl_ax = row("Accel X:")
        self.lbl_ay = row("Accel Y:")
        self.lbl_az = row("Accel Z:")

        # Raw accel
        self.lbl_rax = row("Raw Accel X:")
        self.lbl_ray = row("Raw Accel Y:")
        self.lbl_raz = row("Raw Accel Z:")

        # Orientation
        self.lbl_pitch = row("Pitch:")
        self.lbl_roll = row("Roll:")
        self.lbl_yaw = row("Yaw:")
        self.lbl_tilt = row("Tilt:")

    def _build_right_tabs(self):
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew", padx=12, pady=12)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)

        self.nb = ttk.Notebook(right)
        self.nb.grid(row=0, column=0, sticky="nsew")

        # ---- Tab 1: Kinematics (3 plots: pos, vel, acc) ----
        kin = ttk.Frame(self.nb)
        kin.rowconfigure(0, weight=1)
        kin.rowconfigure(1, weight=1)
        kin.rowconfigure(2, weight=1)
        kin.columnconfigure(0, weight=1)

        # Position plot (x,y,z)
        self.fig_pos = Figure(figsize=(5, 2.8), dpi=100)
        self.ax_pos = self.fig_pos.add_subplot(111)
        self.ax_pos.set_title("Position (X, Y, Z)")
        self.ax_pos.set_xlabel("Time (s)")
        self.ax_pos.set_ylabel("Position")
        self.line_px, = self.ax_pos.plot([], [], lw=1.2, label="px")
        self.line_py, = self.ax_pos.plot([], [], lw=1.2, label="py")
        self.line_pz, = self.ax_pos.plot([], [], lw=1.2, label="pz")
        self.ax_pos.legend(loc="upper right")
        self.canvas_pos = FigureCanvasTkAgg(self.fig_pos, master=kin)
        self.canvas_pos.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        # Velocity plot (x,y,z)
        self.fig_vel = Figure(figsize=(5, 2.8), dpi=100)
        self.ax_vel = self.fig_vel.add_subplot(111)
        self.ax_vel.set_title("Velocity (X, Y, Z)")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity")
        self.line_vx, = self.ax_vel.plot([], [], lw=1.2, label="vx")
        self.line_vy, = self.ax_vel.plot([], [], lw=1.2, label="vy")
        self.line_vz, = self.ax_vel.plot([], [], lw=1.2, label="vz")
        self.ax_vel.legend(loc="upper right")
        self.canvas_vel = FigureCanvasTkAgg(self.fig_vel, master=kin)
        self.canvas_vel.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=6, pady=6)

        # Acceleration plot (x,y,z)
        self.fig_acc = Figure(figsize=(5, 2.8), dpi=100)
        self.ax_acc = self.fig_acc.add_subplot(111)
        self.ax_acc.set_title("Acceleration (X, Y, Z)")
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_ylabel("Acceleration")
        self.line_ax, = self.ax_acc.plot([], [], lw=1.2, label="ax")
        self.line_ay, = self.ax_acc.plot([], [], lw=1.2, label="ay")
        self.line_az, = self.ax_acc.plot([], [], lw=1.2, label="az")
        self.ax_acc.legend(loc="upper right")
        self.canvas_acc = FigureCanvasTkAgg(self.fig_acc, master=kin)
        self.canvas_acc.get_tk_widget().grid(row=2, column=0, sticky="nsew", padx=6, pady=6)

        self.nb.add(kin, text="Kalman")

        # ---- Tab 2: Orientation & Raw (2 plots) ----
        ori = ttk.Frame(self.nb)
        ori.rowconfigure(0, weight=1)
        ori.rowconfigure(1, weight=1)
        ori.columnconfigure(0, weight=1)

        # Orientation: pitch/roll/yaw
        self.fig_ori = Figure(figsize=(5, 3.2), dpi=100)
        self.ax_ori = self.fig_ori.add_subplot(111)
        self.ax_ori.set_title("Orientation (Pitch, Roll, Yaw)")
        self.ax_ori.set_xlabel("Time (s)")
        self.ax_ori.set_ylabel("deg")
        self.line_pitch, = self.ax_ori.plot([], [], lw=1.2, label="pitch")
        self.line_roll,  = self.ax_ori.plot([], [], lw=1.2, label="roll")
        self.line_yaw,   = self.ax_ori.plot([], [], lw=1.2, label="yaw")
        self.ax_ori.legend(loc="upper right")
        self.canvas_ori = FigureCanvasTkAgg(self.fig_ori, master=ori)
        self.canvas_ori.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=6, pady=6)


        # Raw accel: x/y/z
        self.fig_raw = Figure(figsize=(5, 3.2), dpi=100)
        self.ax_raw = self.fig_raw.add_subplot(111)
        self.ax_raw.set_title("Raw Accelerometer (X, Y, Z)")
        self.ax_raw.set_xlabel("Time (s)")
        self.ax_raw.set_ylabel("Raw accel")
        self.line_rax, = self.ax_raw.plot([], [], lw=1.2, label="raw ax")
        self.line_ray, = self.ax_raw.plot([], [], lw=1.2, label="raw ay")
        self.line_raz, = self.ax_raw.plot([], [], lw=1.2, label="raw az")
        self.ax_raw.legend(loc="upper right")
        self.canvas_raw = FigureCanvasTkAgg(self.fig_raw, master=ori)
        self.canvas_raw.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=6, pady=6)

        self.nb.add(ori, text="Orientation & Raw")

        # ---- Tab 3: Matrices (K_mat and P_k_mat) ----
        mat = ttk.Frame(self.nb)
        mat.rowconfigure(0, weight=1)
        mat.rowconfigure(1, weight=1)
        mat.columnconfigure(0, weight=1)

        # K_mat heatmap (9x4)
        self.fig_kmat = Figure(figsize=(5, 3.2), dpi=100)
        self.ax_kmat = self.fig_kmat.add_subplot(111)
        self.ax_kmat.set_title("K Matrix (9x4)")
        self.im_kmat = self.ax_kmat.imshow([[0]*4 for _ in range(9)], cmap='coolwarm', aspect='auto')
        self.fig_kmat.colorbar(self.im_kmat, ax=self.ax_kmat)
        self.ax_kmat.set_xlabel("Column")
        self.ax_kmat.set_ylabel("Row")
        self.canvas_kmat = FigureCanvasTkAgg(self.fig_kmat, master=mat)
        self.canvas_kmat.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        # P_k_mat heatmap (9x9)
        self.fig_pkmat = Figure(figsize=(5, 3.2), dpi=100)
        self.ax_pkmat = self.fig_pkmat.add_subplot(111)
        self.ax_pkmat.set_title("P_k Matrix (9x9)")
        self.im_pkmat = self.ax_pkmat.imshow([[0]*9 for _ in range(9)], cmap='coolwarm', aspect='auto')
        self.fig_pkmat.colorbar(self.im_pkmat, ax=self.ax_pkmat)
        self.ax_pkmat.set_xlabel("Column")
        self.ax_pkmat.set_ylabel("Row")
        self.canvas_pkmat = FigureCanvasTkAgg(self.fig_pkmat, master=mat)
        self.canvas_pkmat.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=6, pady=6)

        self.nb.add(mat, text="Matrices")

    # ---------- POLLING / UPDATE ----------
    def _snapshot(self):
        """Thread-safe-ish snapshot of current rocket data."""
        if self.lock:
            self.lock.acquire()
        try:
            r = self.rocket
            # Position
            px, py, pz = float(r.position.px), float(r.position.py), float(r.position.pz)
            # Velocity
            vx, vy, vz = float(r.velocity.vx), float(r.velocity.vy), float(r.velocity.vz)
            # Accel (filtered)
            ax, ay, az = float(r.acceleration.ax), float(r.acceleration.ay), float(r.acceleration.az)
            # Raw accel
            rax, ray, raz = float(r.raw_accel.ax), float(r.raw_accel.ay), float(r.raw_accel.az)
            # Orientation
            pitch = float(r.orientation.pitch)
            roll  = float(r.orientation.roll)
            yaw   = float(r.orientation.yaw)
            tilt  = float(r.orientation.tilt)
            # Matrices - deep copy the data
            k_mat_data = [[float(r.K_mat._data[i][j]) for j in range(r.K_mat._y)] for i in range(r.K_mat._x)]
            pk_mat_data = [[float(r.P_k_mat._data[i][j]) for j in range(r.P_k_mat._y)] for i in range(r.P_k_mat._x)]
        finally:
            if self.lock:
                self.lock.release()

        return {
            "px": px, "py": py, "pz": pz,
            "vx": vx, "vy": vy, "vz": vz,
            "ax": ax, "ay": ay, "az": az,
            "rax": rax, "ray": ray, "raz": raz,
            "pitch": pitch, "roll": roll, "yaw": yaw, "tilt": tilt,
            "k_mat": k_mat_data,
            "pk_mat": pk_mat_data,
        }

    def _poll_and_update(self):
        now = time.time() - self._t0
        snap = self._snapshot()

        # Update readouts
        self.lbl_px.config(text=f"{snap['px']:.3f}")
        self.lbl_py.config(text=f"{snap['py']:.3f}")
        self.lbl_pz.config(text=f"{snap['pz']:.3f}")

        self.lbl_vx.config(text=f"{snap['vx']:.3f}")
        self.lbl_vy.config(text=f"{snap['vy']:.3f}")
        self.lbl_vz.config(text=f"{snap['vz']:.3f}")

        self.lbl_ax.config(text=f"{snap['ax']:.3f}")
        self.lbl_ay.config(text=f"{snap['ay']:.3f}")
        self.lbl_az.config(text=f"{snap['az']:.3f}")

        self.lbl_rax.config(text=f"{snap['rax']:.3f}")
        self.lbl_ray.config(text=f"{snap['ray']:.3f}")
        self.lbl_raz.config(text=f"{snap['raz']:.3f}")

        self.lbl_pitch.config(text=f"{snap['pitch']:.2f}")
        self.lbl_roll.config(text=f"{snap['roll']:.2f}")
        self.lbl_yaw.config(text=f"{snap['yaw']:.2f}")
        self.lbl_tilt.config(text=f"{snap['tilt']:.2f}")

        # Push to histories
        self.t_hist.append(now)

        self.px_hist.append(snap["px"])
        self.py_hist.append(snap["py"])
        self.pz_hist.append(snap["pz"])

        self.vx_hist.append(snap["vx"])
        self.vy_hist.append(snap["vy"])
        self.vz_hist.append(snap["vz"])

        self.ax_hist.append(snap["ax"])
        self.ay_hist.append(snap["ay"])
        self.az_hist.append(snap["az"])

        self.pitch_hist.append(snap["pitch"])
        self.roll_hist.append(snap["roll"])
        self.yaw_hist.append(snap["yaw"])

        self.rax_hist.append(snap["rax"])
        self.ray_hist.append(snap["ray"])
        self.raz_hist.append(snap["raz"])

        # Update plots
        self._update_plot_multi(self.ax_pos,
                                [(self.line_px, self.px_hist),
                                 (self.line_py, self.py_hist),
                                 (self.line_pz, self.pz_hist)],
                                self.t_hist)
        self.canvas_pos.draw_idle()

        self._update_plot_multi(self.ax_vel,
                                [(self.line_vx, self.vx_hist),
                                 (self.line_vy, self.vy_hist),
                                 (self.line_vz, self.vz_hist)],
                                self.t_hist)
        self.canvas_vel.draw_idle()

        self._update_plot_multi(self.ax_acc,
                                [(self.line_ax, self.ax_hist),
                                 (self.line_ay, self.ay_hist),
                                 (self.line_az, self.az_hist)],
                                self.t_hist)
        self.canvas_acc.draw_idle()

        self._update_plot_multi(self.ax_ori,
                                [(self.line_pitch, self.pitch_hist),
                                 (self.line_roll, self.roll_hist),
                                 (self.line_yaw, self.yaw_hist)],
                                self.t_hist)
        self.canvas_ori.draw_idle()

        self._update_plot_multi(self.ax_raw,
                                [(self.line_rax, self.rax_hist),
                                 (self.line_ray, self.ray_hist),
                                 (self.line_raz, self.raz_hist)],
                                self.t_hist)
        self.canvas_raw.draw_idle()

        # Update matrix heatmaps
        self._update_matrix_heatmap(self.im_kmat, snap["k_mat"])
        self.canvas_kmat.draw_idle()

        self._update_matrix_heatmap(self.im_pkmat, snap["pk_mat"])
        self.canvas_pkmat.draw_idle()

        self.after(self.poll_ms, self._poll_and_update)

    @staticmethod
    def _update_plot_multi(ax, line_series, t):
        if len(t) < 2:
            return
        all_y = []
        for line, ybuf in line_series:
            line.set_data(t, ybuf)
            all_y.extend(ybuf)
        tmin, tmax = t[0], t[-1]
        ax.set_xlim(max(0, tmax - (t[-1] - t[0])), tmax)
        if all_y:
            ymin, ymax = min(all_y), max(all_y)
            if abs(ymax - ymin) < 1e-9:
                ymin -= 1.0
                ymax += 1.0
            pad = 0.05 * (ymax - ymin)
            ax.set_ylim(ymin - pad, ymax + pad)

    def _update_matrix_heatmap(self, image, matrix_data):
        """Update a matrix heatmap with new data."""
        image.set_data(matrix_data)
        # Auto-scale color limits based on data range
        all_vals = [val for row in matrix_data for val in row]
        if all_vals:
            vmin, vmax = min(all_vals), max(all_vals)
            if abs(vmax - vmin) < 1e-9:
                vmin -= 0.1
                vmax += 0.1
            image.set_clim(vmin, vmax)

        # Get the axes from the image
        ax = image.axes

        # Clear existing text annotations
        for txt in ax.texts:
            txt.remove()

        # Add text annotations for each cell
        rows = len(matrix_data)
        cols = len(matrix_data[0]) if rows > 0 else 0
        for i in range(rows):
            for j in range(cols):
                value = matrix_data[i][j]
                # Format the value - use scientific notation for very small/large values
                if abs(value) < 0.01 or abs(value) >= 1000:
                    text_str = f"{value:.2e}"
                else:
                    text_str = f"{value:.3f}"
                ax.text(j, i, text_str, ha="center", va="center",
                       color="white" if abs(value) > (vmin + vmax) / 2 else "black",
                       fontsize=7)

    def _on_close(self):
        self.destroy()



# ----------------------------
# Main (demo)
# ----------------------------
if __name__ == "__main__":
    lock = threading.Lock()  # optional but recommended

    inp_thd = threading.Thread(target=inputprocess, daemon=True)
    inp_thd.start()


    app = RocketDashboard(data, lock=lock, poll_ms=100, history_sec=30.0)
    
    app.mainloop()
