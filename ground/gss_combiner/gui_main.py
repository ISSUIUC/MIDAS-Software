import tkinter as tk
from tkinter import ttk
import multiprocessing
import subprocess
from pathlib import Path
import sys
import os
import serial
from serial.tools.list_ports import comports
import time
import json

import threading
import sys
import queue


def get_ports():
    return [port.device for port in comports()]

def is_port_taken(port):
    """
    Checks if a serial port is currently in use.

    Args:
        port (str): The name of the serial port to check (e.g., 'COM3' or '/dev/ttyUSB0').

    Returns:
        bool: True if the port is taken, False otherwise.
    """
    try:
        ser = serial.Serial(port)
        return False, ser  # Port is free
    except serial.SerialException as e:
        return True, None


class FeatherSubprocess:
    MAXIMUM_STDOUT_LINES = 300
    def __init__(self, port):
        self.__port: str = port
        self.__serial = None
        self.meta: str = ""
        self.stat: str = "NONE"
        self.type: str = "UNKNOWN"
        self.__is_active = False
        self.proc = None
        self.pipe_conn = None
        self.__ip = ""
        self.should_log = False
        self.stage_sel = ""

        self.has_errored = False

        self.main_stdout = []
        self.__terminal_outputs = []

        print(f"Initializing new device on {self.__port}")
        self.check_type()
    
    def get_stat(self):
        return self.__ip, self.should_log

    def set_terminal_output(self, outpt):
        self.__terminal_outputs.append(outpt)

    def set_ip(self, ip):
        self.__ip = ip

    def add_to_stdout(self, msg):
        self.main_stdout.append(msg)

        if len(self.main_stdout) > FeatherSubprocess.MAXIMUM_STDOUT_LINES:
            self.main_stdout = self.main_stdout[1:]

        for outpt in self.__terminal_outputs:
            outpt.config(state="normal")
            msg_str: str = str(msg)
            if msg_str.startswith("[F]"):
                outpt.insert("end", f"{msg_str}\n", "raw_out")
            else:
                outpt.insert("end", f"{msg_str}\n")
            outpt.config(state="disabled")
            outpt.see("end")

    def check_type(self):
        print("Check type invoked on ", self.__port)
        if self.__is_active:
            return # This will be taken over by another process already
        
        port_taken, self.__serial = is_port_taken(self.__port)
        if port_taken:
            self.stat = "NONE"
            self.type = "UNKNOWN"
            print("Sad!")
        else:
            self.stat = "IDENTIFYING..."
            self.type = "UNKNOWN"

            self.__serial.write("IDENT\n".encode())

            time.sleep(0.5)
            data = self.__serial.read_all().decode().splitlines()
            for line in data:
                print(f"[{self.__port}] {line}")
                if line.startswith("IDENT_RESPONSE:"):
                    ident_value = line[15:]
                    
                    if ident_value == "FEATHER_M0":
                        self.type = "FEATHER M0"
                        self.stat = "OFFLINE"
                        self.__serial.close()
                        self.__serial = None
                        return
                    
                    if ident_value == "FEATHER_DUO":
                        self.type = "FEATHER DUO"
                        self.stat = "OFFLINE"
                        self.__serial.close()
                        self.__serial = None
                        return
            
            self.type = "UNKNOWN"
            self.stat = "NONE"
            self.__serial.close()
            self.__serial = None

    def is_online(self):
        return self.stat.lower() == "online"
    
    def clean_visual(self):
        self.set_ip("")
        self.stat = "OFFLINE"
        self.proc = None
        self.main_stdout = []


    def cleanup(self):
        print(f"[{self.__port}] FeatherSubprocess.cleanup invoked!")
        if self.pipe_conn:
            self.pipe_conn.send("kill\n")
            
        self.clean_visual()

        if self.pipe_conn:
            self.pipe_conn.close()
        self.pipe_conn = None



    def get_port(self):
        return self.__port
    
    def get_serial(self):
        return self.__serial

    def reset(self):
        if self.__serial:
            self.__serial.close()


    def to_dict(self):
        return {"name": self.type, "port": self.__port, "status": self.stat, "server": self.__ip, "meta": self.meta}


devices: list[FeatherSubprocess] = []

def get_device(port):
    # get the device
    for _device in devices:
        if _device.get_port() == port:
            return _device
    return None

def run_standalone_worker(pipe_conn, ip, port, stage_sel, do_log):
    # Add real logic here
    script_path = os.path.join(os.path.dirname(__file__), "standalone.py")
    print("Begin Subprocess:")
    log_t = "" if do_log else "--no-log"
    print(f"{sys.executable} {script_path} --ip {ip} --port {port} --{stage_sel} {log_t}")

    args = [
        sys.executable, script_path,
        "--ip", ip,
        "--port", port,
        f"--{stage_sel}"
    ]

    if not do_log:
        args.append("--no-log")

    proc = subprocess.Popen(args,  stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1)

    # subprocess.run([sys.executable, script_path, "--ip", ip, "--port", port, f"--{stage_sel}"])

    stdin_q = queue.Queue()

    def read_stdout():
        for line in proc.stdout:
            stdin_q.put(line)

    thd = threading.Thread(target=read_stdout, daemon=True).start()

    while True:
        if pipe_conn.poll():
            msg = pipe_conn.recv()
            if msg == "kill\n":
                break
            proc.stdin.write(msg)
            proc.stdin.flush()
        
        if not stdin_q.empty():
            line = stdin_q.get()
            pipe_conn.send(line.strip())

        time.sleep(0.01)
    
    # End loop and clean up
    proc.kill()
    pipe_conn.close()
    print(f"[{port}] Cleaning up process")


class DeviceApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MIDAS Base")
        self.geometry("800x550")
        self.selected_device = None
        self.windows = []
        self.create_widgets()

    def update_devices(self):
        global devices
        ports = get_ports()
        existing_ports = [d.get_port() for d in devices]

        # Remove old ports that aren't connected
        for d in devices:
            if d.get_port() not in ports:
                print("[!] Deleting ", d.get_port())
                d.cleanup()

                for window in self.windows:
                    _device, _window = window
                    if d.get_port() == _device:
                        _window.destroy()

        devices = [d for d in devices if d.get_port() in ports]

        for p in ports:
            # Check if this port is already in devices:
            if p not in existing_ports:
                # Create a new one!
                devices.append(FeatherSubprocess(p))
        
        self.update_device_list()
        self.after(200, self.update_devices)

    def update_stdouts(self):
        global devices
        for device in devices:
            if device.pipe_conn is None or device.proc is None:
                continue
            
            try:
                while device.pipe_conn.poll():
                    msg = device.pipe_conn.recv()
                    # print("MSG RECV: ", msg)
                    if msg.startswith("REPORT_OK:"):
                        ip = msg[10:]
                        device.stat = "ONLINE"
                        device.set_ip(ip)

                    if msg.startswith("REPORT_ERR"):
                        print(f"[!] Process {device.get_port()} flagged an error and exited.")
                        device.cleanup()
                        print(f"[!] Process {device.get_port()} Triggering a device cleanup:")
                        continue
                    device.add_to_stdout(msg)
            except:
                print(f"[{device.get_port()}] Detected an unexpected pipe closure, but process isn't cleaned up!")
                device.meta = "ERR: UNEXPECTED TERM"
                device.has_errored = True
                device.pipe_conn = None
                device.cleanup()

                for window in self.windows:
                    _device, _window = window
                    if device.get_port() == _device:
                        _window.destroy()


        self.after(50, self.update_stdouts)

    def update_device_list(self):
        # Clear treeview
        for item in self.tree.get_children():
            self.tree.delete(item)

        # Re-insert updated device info
        for _device in devices:
            device = _device.to_dict()
            tags = ("disabled",) if device["status"].upper() in ("NONE", "IDENTIFYING...") else ()
            if _device.is_online():
                tags = tags + ("connected",)
            if _device.has_errored:
                tags = tags + ("errored",)
            new_item = self.tree.insert(
                "", "end",
                values=(device["port"], device["name"], device["status"], device["server"], device["meta"]),
                tags=tags
            )

            if _device.get_port() == self.selected_device:
                self.tree.selection_add(new_item)

                if _device.is_online():
                    self.inspect_btn.config(state="normal")
                    self.connect_btn.config(text="Disconnect")
                else:
                    self.inspect_btn.config(state="disabled")
                    self.connect_btn.config(text="Connect")

        # Update stats
        self.total_label.config(text=f"Total Devices: {len(devices)}")
        online_count = sum(1 for d in devices if d.to_dict()["status"].lower() == "online")
        self.online_label.config(text=f"Online: {online_count}")

        


    def create_widgets(self):
        # Main layout
        main_frame = ttk.Frame(self)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Left side: device list
        list_frame = ttk.Frame(main_frame)
        list_frame.pack(side="left", fill="both", expand=True)

        title = ttk.Label(list_frame, text="COM List", font=("Helvetica", 14))
        title.pack(pady=5)

        columns = ("Device Port", "Type", "Status", "Streaming to", "Meta")
        self.tree = ttk.Treeview(list_frame, columns=columns, show="headings")
        self.tree.tag_configure("disabled", foreground="gray")
        
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=120)

        for _device in devices:
            device = _device.to_dict()

            tags = ("disabled",) if device["status"].upper() == "NONE" or device["status"].upper() == "IDENTIFYING..." else ()
            self.tree.insert("", "end", values=(device["port"], device["name"], device["status"], device["server"], device["meta"]), tags=tags)

        self.tree.bind("<<TreeviewSelect>>", self.on_select)
        self.tree.pack(fill="both", expand=True)

        self.tree.tag_configure("connected", background="#d2ffd2")
        self.tree.tag_configure("errored", background="#ffd2d2")

        # Right side: control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side="right", fill="y", padx=10, pady=5)

        control_title = ttk.Label(control_frame, text="Input", font=("Helvetica", 14))
        control_title.pack(pady=5)

        self.device_label = ttk.Label(control_frame, text="Select a device")
        self.device_label.pack(pady=5)

        self.stage_sel = tk.StringVar(value="sustainer")  # Default selected value

        radio_label = ttk.Label(control_frame, text="Stage Select:")
        radio_label.pack(pady=5)

        self.radio1 = ttk.Radiobutton(control_frame, text="Sustainer", variable=self.stage_sel, value="sustainer")
        self.radio1.pack()

        self.radio2 = ttk.Radiobutton(control_frame, text="Booster", variable=self.stage_sel, value="booster")
        self.radio2.pack()

        self.radio3 = ttk.Radiobutton(control_frame, text="Duo", variable=self.stage_sel, value="duo")
        self.radio3.pack()

        self.do_log = tk.BooleanVar(value=True)

        self.do_log_checkbox = ttk.Checkbutton(control_frame, text="Generate Log File", variable=self.do_log)
        self.do_log_checkbox.pack(pady=(0, 5))


        self.radio1.config(state="disabled")
        self.radio2.config(state="disabled")
        self.radio3.config(state="disabled")
        self.do_log_checkbox.config(state="disabled")
        # Separator
        ttk.Separator(control_frame, orient="horizontal").pack(fill="x", pady=15)

        stats_title = ttk.Label(control_frame, text="Network", font=("Helvetica", 12, "underline"))
        stats_title.pack(pady=(0, 5))

        label = ttk.Label(control_frame, text="GSS IP:")
        label.pack(pady=5)

        self.ip_entry = tk.Entry(control_frame)
        self.ip_entry.pack(pady=5)

        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.perform_action, state="disabled")
        self.connect_btn.pack(pady=2)

        self.inspect_btn = ttk.Button(control_frame, text="Console", command=self.inspect_window, state="disabled")
        self.inspect_btn.pack(pady=2)


        stats_title = ttk.Label(control_frame, text="System", font=("Helvetica", 12, "underline"))
        stats_title.pack(pady=(20, 5))

        self.total_label = ttk.Label(control_frame, text=f"Total Devices: {len(devices)}")
        self.total_label.pack(anchor="w")

        online_count = sum(1 for d in devices if d.to_dict()["status"].lower() == "online")
        self.online_label = ttk.Label(control_frame, text=f"Online: {online_count}")
        self.online_label.pack(anchor="w")


        def deselect(event=None):
            self.selected_device = None
            self.tree.selection_remove(self.tree.selection())
            self.device_label.config(text="Select a device")
            self.radio1.config(state="disabled")
            self.radio2.config(state="disabled")
            self.radio3.config(state="disabled")
            self.inspect_btn.config(state="disabled")
            self.connect_btn.config(state="disabled")
            self.do_log_checkbox.config(state="disabled")
        self.bind("<Escape>", deselect)

    def on_select(self, event):
        selected = self.tree.selection()
        if selected:
            item = selected[0]
            tags = self.tree.item(item, "tags")

            if "disabled" in tags:
                # Prevent selection visually
                self.tree.selection_remove(item)
                self.selected_device = None
                self.device_label.config(text="Select a device")
                self.connect_btn.config(state="disabled")
                self.inspect_btn.config(state="disabled")
                self.do_log_checkbox.config(state="disabled")
                return

            values = self.tree.item(item, "values")
            is_same_select = values[0] == self.selected_device
            self.selected_device = values[0]
            self.device_label.config(text=f"Selected: {self.selected_device}")
            self.connect_btn.config(state="normal")
            self.do_log_checkbox.config(state="normal")

            _device = get_device(self.selected_device)

            if not is_same_select:
                if _device.type == "FEATHER M0":
                    self.radio1.config(state="normal")
                    self.radio2.config(state="normal")
                    self.radio3.config(state="disabled")
                    self.stage_sel.set("sustainer")

                if _device.type == "FEATHER DUO":
                    self.radio1.config(state="disabled")
                    self.radio2.config(state="disabled")
                    self.radio3.config(state="normal")
                    self.stage_sel.set("duo")

                if _device.is_online():
                    dev_ip, dev_sl = _device.get_stat()
                    self.do_log.set(dev_sl)
                    self.ip_entry.delete(0, tk.END) # Clear existing content
                    self.ip_entry.insert(0, dev_ip)
                    self.stage_sel.set(_device.stage_sel)

    def perform_action(self):
        global devices
        if self.selected_device:
            target_device = get_device(self.selected_device)

            if target_device.is_online():
                # disconnect & close windows
                target_device.cleanup()

                target_device.proc = None
                target_device.meta = ""

                for window in self.windows:
                    _device, _window = window
                    if self.selected_device == _device:
                        _window.destroy()
                return

            if target_device.pipe_conn is not None or target_device.proc is not None:
                print("Cleaned up device.")
                target_device.cleanup()
                target_device.pipe_conn = None
                target_device.proc = None

            ip = self.ip_entry.get()
            print(f"Running device {self.selected_device}")
            print(f"Connecting to... {self.ip_entry.get()}")

            # self.open_terminal_window(self.selected_device)

            should_log = self.do_log.get()

            target_device.should_log = should_log

            target_device.has_errored = False
            target_device.reset()
            target_device.stat = "STARTUP..."
            # target_device.meta = f"{self.stage_sel.get().upper()} (LOG: {"YES" if should_log else "NO"})"
            target_device.stage_sel = self.stage_sel.get()
            target_device.pipe_conn, child_conn = multiprocessing.Pipe()
            target_device.proc = multiprocessing.Process(target=run_standalone_worker, args=(child_conn, ip, self.selected_device, self.stage_sel.get(), should_log))
            target_device.proc.start()


    def inspect_window(self):
        if self.selected_device:
            print("Opening terminal window")
            self.open_terminal_window(self.selected_device)
            # Add real logic here

    def show_json_window(self):
        window = tk.Toplevel(self)

        title = "Data"
        json_data = {"hello": "world", "bruh": "moment"}
        window.title(title)
        window.geometry("600x400")

        label = ttk.Label(window, text=title, font=("Helvetica", 14, "bold"))
        label.pack(pady=5)

        # Text widget with scrollbar
        text_frame = ttk.Frame(window)
        text_frame.pack(fill="both", expand=True, padx=10, pady=5)

        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side="right", fill="y")

        text = tk.Text(text_frame, wrap="none", yscrollcommand=scrollbar.set, bg="#1e1e1e", fg="#d4d4d4", insertbackground="white")
        text.pack(fill="both", expand=True)
        scrollbar.config(command=text.yview)

        # Pretty-print the JSON
        pretty_json = json.dumps(json_data, indent=4)
        text.insert("1.0", pretty_json)
        text.config(state="disabled")  # Make it read-only

        # Optional: allow closing with Esc
        window.bind("<Escape>", lambda e: window.destroy())

    def open_terminal_window(self, device):
        global devices
        term_win = tk.Toplevel(self)
        term_win.title("Network console: " + device)
        term_win.geometry("500x300")

        self.windows.append((device, term_win))

        # get the device
        target_device = get_device(device)

        # Output area (read-only text box with scrollbar)
        output_frame = ttk.Frame(term_win)
        output_frame.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(output_frame)
        scrollbar.pack(side="right", fill="y")

        output = tk.Text(output_frame, wrap="word", bg="black", fg="white", insertbackground="lime")
        target_device.set_terminal_output(output)
        output.pack(fill="both", expand=True)
        output.insert("end", "<BEGINNING OF INPUT STREAM>\n")

        for stdout_line in target_device.main_stdout:
            output.insert("end", f"{stdout_line}\n")

        output.insert("end", f"<NEW SESSION>\n", "user_in")

        output.config(state="disabled", yscrollcommand=scrollbar.set)
        scrollbar.config(command=output.yview)

        # Input field
        input_frame = ttk.Frame(term_win)
        input_frame.pack(fill="x", padx=5, pady=5)

        input_var = tk.StringVar()
        input_entry = tk.Entry(input_frame, textvariable=input_var, bg="black", fg="white", insertbackground="white")
        input_entry.pack(side="left", fill="x", expand=True)

        output.tag_configure("user_in", foreground="green2")
        output.tag_configure("raw_out", foreground="SkyBlue1")

        def send_command(event=None):
            command = input_var.get().strip()
            if command:
                output.config(state="normal")
                output.insert("end", f">> {command}\n", "user_in")
                output.config(state="disabled")
                output.see("end")
                target_device.pipe_conn.send(command + "\n")
                input_var.set("")

        input_entry.bind("<Return>", send_command)

        submit_btn = ttk.Button(input_frame, text="Send", command=send_command)
        submit_btn.pack(side="right", padx=5)

        input_entry.focus_set()


if __name__ == "__main__":
    app = DeviceApp()
    app.after(1000, app.update_devices)
    app.after(50, app.update_stdouts)
    app.mainloop()
