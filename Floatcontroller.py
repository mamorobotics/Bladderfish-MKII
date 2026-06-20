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
        self.root.geometry("850x750")

        self.serial_port = None
        self.running = False
        self.recv_queue = queue.Queue()
        self.acks = 0
        self.packets = 0
        
        self.profile_data = []
        self.profile_active = False
        self.current_profile_id = 0
        self.test_pending = False

        self._build_ui()
        self.refresh_ports()
        self._poll_queue()

    def _build_ui(self):
        # Connection
        top = ttk.LabelFrame(self.root, text="Connection")
        top.pack(fill="x", padx=8, pady=4)

        ttk.Label(top, text="Port:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=24)
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(top, text="Test Connection", command=self.test_connection).grid(row=0, column=3, padx=4)

        ttk.Label(top, text="Baud:").grid(row=0, column=4, padx=4)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200"], width=8).grid(row=0, column=5)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=6, padx=8)

        self.status_label = ttk.Label(top, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=7, padx=8)

        # Motor & Pressure
        ctrl_frame = ttk.Frame(self.root)
        ctrl_frame.pack(fill="x", padx=8, pady=4)

        motor_frame = ttk.LabelFrame(ctrl_frame, text="Motor & Pressure")
        motor_frame.pack(fill="x", pady=4)
        ttk.Button(motor_frame, text="FORWARD", command=lambda: self.send("FORWARD")).pack(side="left", padx=4, pady=8)
        ttk.Button(motor_frame, text="REVERSE", command=lambda: self.send("REVERSE")).pack(side="left", padx=4, pady=8)
        ttk.Button(motor_frame, text="STOP", command=lambda: self.send("STOP")).pack(side="left", padx=4, pady=8)
        
        ttk.Separator(motor_frame, orient="vertical").pack(side="left", fill="y", padx=10)
        
        ttk.Button(motor_frame, text="Get Pressure", command=lambda: self.send("GET_PRESSURE")).pack(side="left", padx=4)
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(motor_frame, text="Stream", variable=self.stream_var, command=self.toggle_stream).pack(side="left", padx=4)

        # Profiles
        profiles_frame = ttk.Frame(self.root)
        profiles_frame.pack(fill="x", padx=8, pady=4)

        # Profile 1
        prof1_frame = ttk.LabelFrame(profiles_frame, text="Profile 1 (e.g., 2.5m -> 0.4m)")
        prof1_frame.pack(fill="x", pady=4)
        
        ttk.Label(prof1_frame, text="Depth 1 (m):").grid(row=0, column=0, padx=4, pady=6)
        self.p1_d1 = tk.StringVar(value="2.5")
        ttk.Entry(prof1_frame, textvariable=self.p1_d1, width=6).grid(row=0, column=1)

        ttk.Label(prof1_frame, text="Time 1 (s):").grid(row=0, column=2, padx=4)
        self.p1_t1 = tk.StringVar(value="30")
        ttk.Entry(prof1_frame, textvariable=self.p1_t1, width=6).grid(row=0, column=3)

        ttk.Label(prof1_frame, text="Depth 2 (m):").grid(row=0, column=4, padx=4)
        self.p1_d2 = tk.StringVar(value="0.4")
        ttk.Entry(prof1_frame, textvariable=self.p1_d2, width=6).grid(row=0, column=5)

        ttk.Label(prof1_frame, text="Time 2 (s):").grid(row=0, column=6, padx=4)
        self.p1_t2 = tk.StringVar(value="30")
        ttk.Entry(prof1_frame, textvariable=self.p1_t2, width=6).grid(row=0, column=7)

        ttk.Button(prof1_frame, text="Launch Profile 1", command=lambda: self.launch_profile(1)).grid(row=0, column=8, padx=10)

        # Profile 2
        prof2_frame = ttk.LabelFrame(profiles_frame, text="Profile 2")
        prof2_frame.pack(fill="x", pady=4)
        
        ttk.Label(prof2_frame, text="Depth 1 (m):").grid(row=0, column=0, padx=4, pady=6)
        self.p2_d1 = tk.StringVar(value="1.0")
        ttk.Entry(prof2_frame, textvariable=self.p2_d1, width=6).grid(row=0, column=1)

        ttk.Label(prof2_frame, text="Time 1 (s):").grid(row=0, column=2, padx=4)
        self.p2_t1 = tk.StringVar(value="20")
        ttk.Entry(prof2_frame, textvariable=self.p2_t1, width=6).grid(row=0, column=3)

        ttk.Label(prof2_frame, text="Depth 2 (m):").grid(row=0, column=4, padx=4)
        self.p2_d2 = tk.StringVar(value="3.0")
        ttk.Entry(prof2_frame, textvariable=self.p2_d2, width=6).grid(row=0, column=5)

        ttk.Label(prof2_frame, text="Time 2 (s):").grid(row=0, column=6, padx=4)
        self.p2_t2 = tk.StringVar(value="20")
        ttk.Entry(prof2_frame, textvariable=self.p2_t2, width=6).grid(row=0, column=7)

        ttk.Button(prof2_frame, text="Launch Profile 2", command=lambda: self.launch_profile(2)).grid(row=0, column=8, padx=10)
        
        ttk.Button(profiles_frame, text="ABORT PROFILE", command=lambda: self.send("PROFILE_ABORT")).pack(pady=5)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=15, state="disabled", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

    # ── Helpers ────────────────────────────────────────────────────────────
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.running = False
            try: self.serial_port.close()
            except: pass
            self.serial_port = None
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
        else:
            try:
                self.serial_port = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.1)
                self.running = True
                threading.Thread(target=self._reader_thread, daemon=True).start()
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
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
                try: self.packets = int(pkts)
                except: pass
                self.log_msg(f"[SENSOR] {pressure} mbar | {depth} m | motor={motor}")
        elif line.startswith("ACK|"):
            parts = line.split("|")
            cmd = parts[1] if len(parts) > 1 else "?"
            if len(parts) > 2:
                try: self.acks = int(parts[2])
                except: pass
            
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
                self.log_msg("[PROFILE] Mission Started")
            elif msg.startswith("PROFILE_DATA:"):
                if self.profile_active:
                    parts = msg.split(":")
                    if len(parts) == 4:
                        t = int(parts[1]) / 1000.0
                        d = float(parts[2])
                        self.profile_data.append((t, d))
            elif msg.startswith("PROFILE_COMPLETE") or msg.startswith("PROFILE_ABORTED"):
                self.profile_active = False
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

    def send(self, cmd):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Connect to the topside ESP32 first.")
            return
        try:
            self.serial_port.write((cmd + "\n").encode("utf-8"))
            self.log_msg(f"[> GUI] {cmd}")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

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
            d1 = float(d1); t1 = int(t1)
            d2 = float(d2); t2 = int(t2)
        except ValueError:
            messagebox.showerror("Invalid Input", "Depths must be numbers, times must be integers.")
            return

        cmd = f"PROFILE:{d1}:{t1}:{d2}:{t2}"
        self.current_profile_id = profile_id
        self.send(cmd)

    def show_graph(self, data, profile_id):
        win = tk.Toplevel(self.root)
        win.title(f"Dive Profile {profile_id} - Depth Over Time")
        win.geometry("600x400")

        times = [x[0] for x in data]
        depths = [x[1] for x in data]

        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(times, depths, marker='.', linestyle='-')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Depth (m)")
        ax.set_title(f"Profile {profile_id} - Depth Over Time")
        ax.invert_yaxis() # Depth increases downwards
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

if __name__ == "__main__":
    root = tk.Tk()
    app = FloatControlGUI(root)
    root.mainloop()