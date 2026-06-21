import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class FloatControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MateROV Float Controller")
        self.root.geometry("940x820")

        self.serial_port = None
        self.running = False
        self.recv_queue = queue.Queue()
        self.acks = 0
        self.packets = 0

        self.profile_data = []
        self.profile_active = False
        self.current_profile_id = 0
        self.test_pending = False

        self.pressure_var = tk.StringVar(value="---")
        self.depth_var = tk.StringVar(value="---")
        self.motor_var = tk.StringVar(value="---")
        self.packet_var = tk.StringVar(value="0")
        self.profile_status_var = tk.StringVar(value="Idle")

        self._action_widgets = []

        self._build_ui()
        self._set_controls_state(False)
        self.refresh_ports()
        self._poll_queue()

    def _build_ui(self):
        # ── Connection ──────────────────────────────────────────────────────
        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.pack(fill="x", padx=8, pady=4)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, padx=4, pady=6, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, padx=(12, 4))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn, textvariable=self.baud_var,
                     values=["9600", "19200", "38400", "57600", "115200"], width=8).grid(row=0, column=4)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=12)

        self.test_btn = ttk.Button(conn, text="Test Connection", command=self.test_connection)
        self.test_btn.grid(row=0, column=6, padx=4)
        self._action_widgets.append(self.test_btn)

        self.status_label = ttk.Label(conn, text="Disconnected", foreground="red",
                                      font=("Consolas", 9, "bold"))
        self.status_label.grid(row=0, column=7, padx=12)

        # ── Middle: left + right columns ────────────────────────────────────
        mid = ttk.Frame(self.root)
        mid.pack(fill="x", padx=8, pady=4)
        mid.columnconfigure(0, weight=1)
        mid.columnconfigure(1, weight=2)

        # ── LEFT: live readout + manual controls ────────────────────────────
        left = ttk.Frame(mid)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        readout = ttk.LabelFrame(left, text="Live Readout")
        readout.pack(fill="x", pady=(0, 6))

        for row_i, (label, var, unit) in enumerate([
            ("Pressure", self.pressure_var, "mbar"),
            ("Depth",    self.depth_var,    "m"),
            ("Motor",    self.motor_var,    ""),
            ("Packets",  self.packet_var,   ""),
        ]):
            ttk.Label(readout, text=f"{label}:", width=10, anchor="w").grid(
                row=row_i, column=0, padx=8, pady=3, sticky="w")
            lbl = ttk.Label(readout, textvariable=var, font=("Consolas", 10), width=10, anchor="w")
            lbl.grid(row=row_i, column=1, sticky="w")
            if label == "Motor":
                self.motor_display_label = lbl
            if unit:
                ttk.Label(readout, text=unit).grid(row=row_i, column=2, padx=4, sticky="w")

        motor_f = ttk.LabelFrame(left, text="Motor Controls")
        motor_f.pack(fill="x", pady=(0, 6))
        btn_row = ttk.Frame(motor_f)
        btn_row.pack(pady=8, padx=8)

        for label, cmd in [("FORWARD", "FORWARD"), ("STOP", "STOP"), ("REVERSE", "REVERSE")]:
            b = ttk.Button(btn_row, text=label, width=9, command=lambda c=cmd: self.send(c))
            b.pack(side="left", padx=4)
            self._action_widgets.append(b)

        pres_f = ttk.LabelFrame(left, text="Pressure Sensor")
        pres_f.pack(fill="x", pady=(0, 6))
        pres_row = ttk.Frame(pres_f)
        pres_row.pack(pady=8, padx=8, fill="x")

        b = ttk.Button(pres_row, text="Get Pressure", command=lambda: self.send("GET_PRESSURE"))
        b.pack(side="left", padx=(0, 10))
        self._action_widgets.append(b)

        self.stream_var = tk.BooleanVar(value=False)
        chk = ttk.Checkbutton(pres_row, text="Stream Sensor Data",
                               variable=self.stream_var, command=self.toggle_stream)
        chk.pack(side="left")
        self._action_widgets.append(chk)

        custom_f = ttk.LabelFrame(left, text="Custom Command")
        custom_f.pack(fill="x")
        custom_row = ttk.Frame(custom_f)
        custom_row.pack(pady=8, padx=8, fill="x")

        self.custom_cmd_var = tk.StringVar()
        self.custom_entry = ttk.Entry(custom_row, textvariable=self.custom_cmd_var)
        self.custom_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        self.custom_entry.bind("<Return>", lambda e: self.send_custom())
        self._action_widgets.append(self.custom_entry)

        b = ttk.Button(custom_row, text="Send", command=self.send_custom)
        b.pack(side="left")
        self._action_widgets.append(b)

        # ── RIGHT: profile status + profiles ────────────────────────────────
        right = ttk.Frame(mid)
        right.grid(row=0, column=1, sticky="nsew")

        status_f = ttk.LabelFrame(right, text="Profile Status")
        status_f.pack(fill="x", pady=(0, 6))
        status_row = ttk.Frame(status_f)
        status_row.pack(fill="x", padx=8, pady=6)

        ttk.Label(status_row, text="Status:").pack(side="left")
        self.profile_status_label = ttk.Label(
            status_row, textvariable=self.profile_status_var,
            font=("Consolas", 9, "bold"), foreground="gray", width=10)
        self.profile_status_label.pack(side="left", padx=8)

        b = ttk.Button(status_row, text="ABORT PROFILE",
                       command=lambda: self.send("PROFILE_ABORT"))
        b.pack(side="right")
        self._action_widgets.append(b)

        p1_f = ttk.LabelFrame(right, text="Profile 1  (e.g. sink 2.5 m, hold, rise to 0.4 m)")
        p1_f.pack(fill="x", pady=(0, 6))
        self.p1_d1, self.p1_t1, self.p1_d2, self.p1_t2 = self._build_profile_fields(p1_f, 1)

        p2_f = ttk.LabelFrame(right, text="Profile 2")
        p2_f.pack(fill="x")
        self.p2_d1, self.p2_t1, self.p2_d2, self.p2_t2 = self._build_profile_fields(p2_f, 2)

        # ── Log ─────────────────────────────────────────────────────────────
        log_f = ttk.LabelFrame(self.root, text="Log")
        log_f.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        toolbar = ttk.Frame(log_f)
        toolbar.pack(fill="x", padx=4, pady=(4, 0))
        ttk.Button(toolbar, text="Clear Log", command=self.clear_log).pack(side="right")

        self.log = scrolledtext.ScrolledText(log_f, height=10, state="disabled",
                                              font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_profile_fields(self, parent, pid):
        d1_var = tk.StringVar(value="2.5" if pid == 1 else "1.0")
        t1_var = tk.StringVar(value="30"  if pid == 1 else "20")
        d2_var = tk.StringVar(value="0.4" if pid == 1 else "3.0")
        t2_var = tk.StringVar(value="30"  if pid == 1 else "20")

        g = ttk.Frame(parent)
        g.pack(fill="x", padx=8, pady=6)

        ttk.Label(g, text="Sink to (m):").grid(row=0, column=0, padx=4, pady=3, sticky="w")
        ttk.Entry(g, textvariable=d1_var, width=7).grid(row=0, column=1, padx=4)
        ttk.Label(g, text="Hold (s):").grid(row=0, column=2, padx=4, sticky="w")
        ttk.Entry(g, textvariable=t1_var, width=6).grid(row=0, column=3, padx=4)

        ttk.Label(g, text="Rise to (m):").grid(row=1, column=0, padx=4, pady=3, sticky="w")
        ttk.Entry(g, textvariable=d2_var, width=7).grid(row=1, column=1, padx=4)
        ttk.Label(g, text="Hold (s):").grid(row=1, column=2, padx=4, sticky="w")
        ttk.Entry(g, textvariable=t2_var, width=6).grid(row=1, column=3, padx=4)

        b = ttk.Button(g, text=f"Launch Profile {pid}",
                       command=lambda: self.launch_profile(pid))
        b.grid(row=0, column=4, rowspan=2, padx=12, sticky="ns")
        self._action_widgets.append(b)

        return d1_var, t1_var, d2_var, t2_var

    # ── Helpers ─────────────────────────────────────────────────────────────
    def _set_controls_state(self, enabled: bool):
        state = ["!disabled"] if enabled else ["disabled"]
        for w in self._action_widgets:
            try:
                w.state(state)
            except Exception:
                pass

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.running = False
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
            self._set_controls_state(False)
        else:
            try:
                self.serial_port = serial.Serial(
                    self.port_var.get(), int(self.baud_var.get()), timeout=0.1)
                self.running = True
                threading.Thread(target=self._reader_thread, daemon=True).start()
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
                self._set_controls_state(True)
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _reader_thread(self):
        buf = ""
        while self.running:
            try:
                data = self.serial_port.read(256).decode("utf-8", errors="ignore")
                if data:
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.recv_queue.put(line)
            except Exception:
                break

    def _poll_queue(self):
        try:
            while True:
                line = self.recv_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle_line(self, line):
        if line.startswith("SENSOR|"):
            parts = line.split("|")
            if len(parts) >= 5:
                pressure, depth, motor, pkts = parts[1], parts[2], parts[3], parts[4]
                self.pressure_var.set(pressure)
                self.depth_var.set(depth)
                self.motor_var.set(motor)
                try:
                    self.packets = int(pkts)
                    self.packet_var.set(str(self.packets))
                except Exception:
                    pass
                color = {"FORWARD": "green", "REVERSE": "blue", "STOP": "gray"}.get(
                    motor.upper(), "black")
                self.motor_display_label.config(foreground=color)
                self.log_msg(f"[SENSOR] {pressure} mbar | {depth} m | motor={motor}")

        elif line.startswith("ACK|"):
            parts = line.split("|")
            cmd = parts[1] if len(parts) > 1 else "?"
            if len(parts) > 2:
                try:
                    self.acks = int(parts[2])
                except Exception:
                    pass
            if cmd == "PING" and self.test_pending:
                self.test_pending = False
                messagebox.showinfo("Test Successful", "Communication with float is working!")
            elif cmd == "BUSY":
                self.log_msg("[ACK] Float is busy running a profile.")
            else:
                self.log_msg(f"[ACK] {cmd}")

        elif line.startswith("PROFILE|"):
            msg = line[len("PROFILE|"):]
            if msg.startswith("PROFILE_START"):
                self.profile_data = []
                self.profile_active = True
                self.profile_status_var.set("Running")
                self.profile_status_label.config(foreground="green")
                self.log_msg("[PROFILE] Mission Started")
            elif msg.startswith("PROFILE_DATA:"):
                if self.profile_active:
                    parts = msg.split(":")
                    if len(parts) == 4:
                        t = int(parts[1]) / 1000.0
                        d = float(parts[2])
                        self.profile_data.append((t, d))
            elif msg.startswith("PROFILE_COMPLETE"):
                self.profile_active = False
                self.profile_status_var.set("Complete")
                self.profile_status_label.config(foreground="blue")
                self.log_msg(f"[PROFILE] Mission Ended: {msg}")
                if len(self.profile_data) > 1:
                    self.show_graph(self.profile_data, self.current_profile_id)
            elif msg.startswith("PROFILE_ABORTED"):
                self.profile_active = False
                self.profile_status_var.set("Aborted")
                self.profile_status_label.config(foreground="red")
                self.log_msg(f"[PROFILE] Mission Ended: {msg}")
                if len(self.profile_data) > 1:
                    self.show_graph(self.profile_data, self.current_profile_id)
            else:
                self.log_msg(f"[PROFILE] {msg}")
        else:
            self.log_msg(f"[RAW] {line}")

    def log_msg(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    def send(self, cmd):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Connect to the topside ESP32 first.")
            return
        try:
            self.serial_port.write((cmd + "\n").encode("utf-8"))
            self.log_msg(f"[> GUI] {cmd}")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def send_custom(self):
        cmd = self.custom_cmd_var.get().strip()
        if cmd:
            self.send(cmd)
            self.custom_cmd_var.set("")

    def toggle_stream(self):
        self.send("STREAM_ON" if self.stream_var.get() else "STREAM_OFF")

    def test_connection(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Connect to the topside ESP32 first.")
            return
        self.send("PING")
        self.test_pending = True
        self.root.after(2000, self.check_test_result)

    def check_test_result(self):
        if self.test_pending:
            self.test_pending = False
            messagebox.showerror("Test Failed", "No response from float. Check connection and power.")

    def launch_profile(self, profile_id):
        if profile_id == 1:
            d1, t1, d2, t2 = self.p1_d1.get(), self.p1_t1.get(), self.p1_d2.get(), self.p1_t2.get()
        else:
            d1, t1, d2, t2 = self.p2_d1.get(), self.p2_t1.get(), self.p2_d2.get(), self.p2_t2.get()

        try:
            d1 = float(d1)
            t1 = int(t1)
            d2 = float(d2)
            t2 = int(t2)
        except ValueError:
            messagebox.showerror("Invalid Input", "Depths must be numbers, times must be integers.")
            return

        cmd = f"PROFILE:{d1}:{t1}:{d2}:{t2}"
        self.current_profile_id = profile_id
        self.send(cmd)

    def show_graph(self, data, profile_id):
        win = tk.Toplevel(self.root)
        win.title(f"Dive Profile {profile_id} – Depth Over Time")
        win.geometry("600x400")

        times = [x[0] for x in data]
        depths = [x[1] for x in data]

        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(times, depths, marker='.', linestyle='-')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Depth (m)")
        ax.set_title(f"Profile {profile_id} – Depth Over Time")
        ax.invert_yaxis()
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)


if __name__ == "__main__":
    root = tk.Tk()
    app = FloatControlGUI(root)
    root.mainloop()
