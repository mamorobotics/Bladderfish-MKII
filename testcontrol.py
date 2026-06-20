import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import queue


class FloatControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MateROV Float Controller")
        self.root.geometry("820x720")

        self.serial_port = None
        self.running = False
        self.recv_queue = queue.Queue()
        self.acks = 0
        self.packets = 0

        self._build_ui()
        self.refresh_ports()
        self._poll_queue()

    # ── UI layout ──────────────────────────────────────────────────────────
    def _build_ui(self):
        # Connection
        top = ttk.LabelFrame(self.root, text="Connection")
        top.pack(fill="x", padx=8, pady=4)

        ttk.Label(top, text="Port:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(top, text="Baud:").grid(row=0, column=3, padx=4)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self.baud_var,
                     values=["9600", "19200", "38400", "57600", "115200"],
                     width=8).grid(row=0, column=4)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=8)

        self.status_label = ttk.Label(top, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=8)

        # Motor control
        motor_frame = ttk.LabelFrame(self.root, text="Motor Control")
        motor_frame.pack(fill="x", padx=8, pady=4)
        ttk.Button(motor_frame, text="FORWARD",
                   command=lambda: self.send("FORWARD")).pack(side="left", padx=8, pady=8)
        ttk.Button(motor_frame, text="REVERSE",
                   command=lambda: self.send("REVERSE")).pack(side="left", padx=8, pady=8)
        ttk.Button(motor_frame, text="STOP",
                   command=lambda: self.send("STOP")).pack(side="left", padx=8, pady=8)

        # Pressure
        press_frame = ttk.LabelFrame(self.root, text="Pressure")
        press_frame.pack(fill="x", padx=8, pady=4)
        ttk.Button(press_frame, text="Get Pressure (one-shot)",
                   command=lambda: self.send("GET_PRESSURE")).pack(side="left", padx=8, pady=8)
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(press_frame, text="Stream",
                        variable=self.stream_var,
                        command=self.toggle_stream).pack(side="left", padx=8)

        # Depth profile
        prof_frame = ttk.LabelFrame(self.root, text="Depth Profile Mission")
        prof_frame.pack(fill="x", padx=8, pady=4)

        ttk.Label(prof_frame, text="Target Depth (ft):").grid(row=0, column=0, padx=4, pady=6, sticky="w")
        self.depth_var = tk.StringVar(value="5.0")
        ttk.Entry(prof_frame, textvariable=self.depth_var, width=8).grid(row=0, column=1)

        ttk.Label(prof_frame, text="Hold Time (s):").grid(row=0, column=2, padx=4)
        self.hold_var = tk.StringVar(value="10")
        ttk.Entry(prof_frame, textvariable=self.hold_var, width=8).grid(row=0, column=3)

        ttk.Button(prof_frame, text="Start Profile",
                   command=self.start_profile).grid(row=0, column=4, padx=8)
        ttk.Button(prof_frame, text="Abort",
                   command=lambda: self.send("PROFILE_ABORT")).grid(row=0, column=5, padx=4)

        # Stats
        stats_frame = ttk.LabelFrame(self.root, text="Statistics")
        stats_frame.pack(fill="x", padx=8, pady=4)
        self.stat_label = ttk.Label(stats_frame, text="Packets: 0 | ACKs: 0")
        self.stat_label.pack(anchor="w", padx=8, pady=4)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self.log = scrolledtext.ScrolledText(log_frame, height=20,
                                             state="disabled",
                                             font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

        # Manual command
        cmd_frame = ttk.Frame(self.root)
        cmd_frame.pack(fill="x", padx=8, pady=4)
        ttk.Label(cmd_frame, text="Manual:").pack(side="left")
        self.cmd_var = tk.StringVar()
        entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        entry.pack(side="left", fill="x", expand=True, padx=4)
        entry.bind("<Return>", lambda e: self.send_manual())
        ttk.Button(cmd_frame, text="Send", command=self.send_manual).pack(side="left")

    # ── Serial helpers ────────────────────────────────────────────────────
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
                self.serial_port = serial.Serial(self.port_var.get(),
                                                 int(self.baud_var.get()),
                                                 timeout=0.1)
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

    # ── Message parsing ───────────────────────────────────────────────────
    def _handle_line(self, line):
        if line.startswith("SENSOR|"):
            parts = line.split("|")
            if len(parts) >= 5:
                pressure, depth, motor, pkts = parts[1], parts[2], parts[3], parts[4]
                try: self.packets = int(pkts)
                except: pass
                self.log_msg(f"[SENSOR] {pressure} mbar | {depth} ft | motor={motor}")
        elif line.startswith("ACK|"):
            parts = line.split("|")
            cmd = parts[1] if len(parts) > 1 else "?"
            if len(parts) > 2:
                try: self.acks = int(parts[2])
                except: pass
            self.log_msg(f"[ACK] {cmd}")
        elif line.startswith("PROFILE|"):
            self.log_msg(f"[PROFILE] {line[len('PROFILE|'):]}")
        elif line.startswith("MSG|"):
            self.log_msg(f"[MSG] {line[len('MSG|'):]}")
        elif line.startswith("SEND_OK:"):
            self.log_msg(f"[> SENT] {line[len('SEND_OK: '):]}")
        elif line.startswith("SEND_FAIL:"):
            self.log_msg(f"[SEND FAIL] {line[len('SEND_FAIL: '):]}")
        else:
            self.log_msg(f"[RAW] {line}")
        self.stat_label.config(text=f"Packets: {self.packets} | ACKs: {self.acks}")

    def log_msg(self, msg):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    # ── Send helpers ──────────────────────────────────────────────────────
    def send(self, cmd):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Connect to the topside ESP32 first.")
            return
        try:
            self.serial_port.write((cmd + "\n").encode("utf-8"))
            self.log_msg(f"[> GUI] {cmd}")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def send_manual(self):
        c = self.cmd_var.get().strip()
        if c:
            self.send(c)
            self.cmd_var.set("")

    def toggle_stream(self):
        self.send("STREAM_ON" if self.stream_var.get() else "STREAM_OFF")

    def start_profile(self):
        try:
            d = float(self.depth_var.get())
            t = int(self.hold_var.get())
        except ValueError:
            messagebox.showerror("Invalid Input",
                                 "Depth must be a number; hold time an integer.")
            return
        if not (0 < d <= 100):
            messagebox.showerror("Invalid Depth", "Depth must be between 0 and 100 ft.")
            return
        if not (0 <= t <= 300):
            messagebox.showerror("Invalid Time", "Hold time must be 0-300 s.")
            return
        self.send(f"PROFILE:{d}:{t}")


if __name__ == "__main__":
    root = tk.Tk()
    app = FloatControlGUI(root)
    root.mainloop()