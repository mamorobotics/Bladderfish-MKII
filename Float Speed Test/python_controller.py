#!/usr/bin/env python3
"""
BLADDERFISH FLOAT CALIBRATION CONTROLLER
========================================
GUI interface for calibrating float motor speeds.

WORKFLOW:
1. Connect float running Float_Speed_Test_Shallow.ino
2. Click "Start Test" - float automatically runs calibration
3. Float tests 7 motor speeds (80-200 PWM)
4. GUI receives ~5000 data points and analyzes
5. Finds optimal speeds (targets 0.5 m/s)
6. Auto-saves to calibration_config.json

OUTPUT:
- calibration_config.json with optimal descent/ascent PWM values
- Can export CSV or JSON for analysis
- Graphs showing descent/ascent rates

NEXT STEP:
Update Float.ino DESCENT_PWM and ASCENT_PWM variables with results
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import serial
import json
from datetime import datetime
import csv
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# ──────────────────────────────────────────────────────────────────────────────
# Data Structures
# ──────────────────────────────────────────────────────────────────────────────

class DataPoint:
    def __init__(self, time_ms, depth_m, pressure_mbar, motor_pwm):
        self.time_ms = time_ms
        self.depth_m = depth_m
        self.pressure_mbar = pressure_mbar
        self.motor_pwm = motor_pwm

class CalibrationSession:
    def __init__(self):
        self.data_points = []
        self.results = {
            'descent_rates': {},
            'ascent_rates': {},
            'recommended_descent_pwm': 0,
            'recommended_ascent_pwm': 0,
        }

    def add_point(self, time_ms, depth_m, pressure_mbar, motor_pwm):
        self.data_points.append(DataPoint(time_ms, depth_m, pressure_mbar, motor_pwm))

    def calculate_rate(self, pwm):
        rates = []
        i = 0
        while i < len(self.data_points) - 1:
            if self.data_points[i].motor_pwm == pwm:
                start_depth = self.data_points[i].depth_m
                start_time = self.data_points[i].time_ms
                while i < len(self.data_points) and self.data_points[i].motor_pwm == pwm:
                    i += 1
                end_depth = self.data_points[i - 1].depth_m if i > 0 else start_depth
                end_time = self.data_points[i - 1].time_ms if i > 0 else start_time
                depth_change = abs(end_depth - start_depth)
                time_change_s = (end_time - start_time) / 1000.0
                if time_change_s > 0:
                    rates.append(depth_change / time_change_s)
            else:
                i += 1
        return sum(rates) / len(rates) if rates else 0.0

    def analyze_results(self):
        speeds_tested = sorted(set(p.motor_pwm for p in self.data_points if p.motor_pwm > 0))
        for speed in speeds_tested:
            self.results['descent_rates'][speed] = self.calculate_rate(speed)
        for speed in speeds_tested:
            self.results['ascent_rates'][speed] = self.calculate_rate(speed)

        target_rate = 0.5
        if self.results['descent_rates']:
            best = min(self.results['descent_rates'].items(), key=lambda x: abs(x[1] - target_rate))
            self.results['recommended_descent_pwm'] = best[0]
        if self.results['ascent_rates']:
            best = min(self.results['ascent_rates'].items(), key=lambda x: abs(x[1] - target_rate))
            self.results['recommended_ascent_pwm'] = best[0]

    def save_config(self):
        config_file = Path(__file__).parent / "calibration_config.json"
        config = {
            "version": "1.0",
            "timestamp": datetime.now().isoformat(),
            "calibration_status": "calibrated",
            "motor": {
                "descent_pwm": self.results['recommended_descent_pwm'],
                "ascent_pwm": self.results['recommended_ascent_pwm'],
                "fine_control_pwm": 75,
                "rapid_descent_pwm": 240
            },
            "test_results": {
                "descent_rates": self.results['descent_rates'],
                "ascent_rates": self.results['ascent_rates'],
                "total_data_points": len(self.data_points)
            }
        }
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        return config_file

    def export_csv(self, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time (ms)', 'Depth (m)', 'Pressure (mbar)', 'Motor PWM'])
            for point in self.data_points:
                writer.writerow([point.time_ms, f"{point.depth_m:.4f}", f"{point.pressure_mbar:.2f}", point.motor_pwm])

# ──────────────────────────────────────────────────────────────────────────────
# Serial Communication
# ──────────────────────────────────────────────────────────────────────────────

class SerialReceiver(threading.Thread):
    def __init__(self, port, baudrate=115200):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.message_callback = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            while self.running:
                if self.serial_conn.in_waiting:
                    try:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line and self.message_callback:
                            self.message_callback(line)
                    except:
                        pass
                else:
                    threading.Event().wait(0.01)
        except Exception as e:
            print(f"Serial error: {e}")
        finally:
            if self.serial_conn:
                self.serial_conn.close()

    def send(self, message):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write((message + '\n').encode('utf-8'))

    def stop(self):
        self.running = False

# ──────────────────────────────────────────────────────────────────────────────
# Main GUI Application
# ──────────────────────────────────────────────────────────────────────────────

class CalibrationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Bladderfish Float Calibration - 2m Shallow Test")
        self.root.geometry("1400x900")

        self.session = CalibrationSession()
        self.serial_receiver = None
        self.test_running = False
        self.received_data_count = 0

        self.setup_ui()
        self.list_ports()

    def setup_ui(self):
        # Control Panel (Left)
        control_frame = ttk.LabelFrame(self.root, text="Calibration Control", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)

        # Port Selection
        ttk.Label(control_frame, text="Serial Port:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=20, state='readonly')
        self.port_combo.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Button(control_frame, text="Refresh", command=self.list_ports).grid(row=0, column=2, padx=5)

        # Status
        ttk.Label(control_frame, text="Status:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=1, sticky=tk.W)

        # Connect Button
        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=10)

        # Separator
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=3, sticky=tk.EW, pady=10)

        # Test Controls
        ttk.Label(control_frame, text="Calibration Test:", font=("", 10, "bold")).grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=5)

        self.start_btn = ttk.Button(control_frame, text="Start Calibration Test", command=self.start_test, state=tk.DISABLED)
        self.start_btn.grid(row=5, column=0, columnspan=3, sticky=tk.EW, pady=5)

        self.stop_btn = ttk.Button(control_frame, text="Stop & Return to Surface", command=self.stop_test, state=tk.DISABLED)
        self.stop_btn.grid(row=6, column=0, columnspan=3, sticky=tk.EW, pady=5)

        # Data Display
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=3, sticky=tk.EW, pady=10)
        ttk.Label(control_frame, text="Data Collection:", font=("", 10, "bold")).grid(row=8, column=0, columnspan=2, sticky=tk.W, pady=5)

        ttk.Label(control_frame, text="Data Points:").grid(row=9, column=0, sticky=tk.W)
        self.points_label = ttk.Label(control_frame, text="0")
        self.points_label.grid(row=9, column=1, sticky=tk.W)

        ttk.Label(control_frame, text="Duration:").grid(row=10, column=0, sticky=tk.W)
        self.duration_label = ttk.Label(control_frame, text="0 s")
        self.duration_label.grid(row=10, column=1, sticky=tk.W)

        # Results
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=11, column=0, columnspan=3, sticky=tk.EW, pady=10)
        ttk.Label(control_frame, text="Results:", font=("", 10, "bold")).grid(row=12, column=0, columnspan=2, sticky=tk.W, pady=5)

        ttk.Label(control_frame, text="Desc PWM:").grid(row=13, column=0, sticky=tk.W)
        self.desc_pwm_label = ttk.Label(control_frame, text="—")
        self.desc_pwm_label.grid(row=13, column=1, sticky=tk.W)

        ttk.Label(control_frame, text="Asc PWM:").grid(row=14, column=0, sticky=tk.W)
        self.asc_pwm_label = ttk.Label(control_frame, text="—")
        self.asc_pwm_label.grid(row=14, column=1, sticky=tk.W)

        # Export Buttons
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).grid(row=15, column=0, columnspan=3, sticky=tk.EW, pady=10)

        ttk.Button(control_frame, text="Export CSV", command=self.export_csv, state=tk.DISABLED).grid(row=16, column=0, columnspan=3, sticky=tk.EW, pady=5)
        self.export_csv_btn = control_frame.winfo_children()[-1]

        ttk.Button(control_frame, text="Generate Report", command=self.generate_report, state=tk.DISABLED).grid(row=17, column=0, columnspan=3, sticky=tk.EW, pady=5)
        self.report_btn = control_frame.winfo_children()[-1]

        # Results Panel (Right)
        right_frame = ttk.Frame(self.root)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Notebook for tabs
        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Log Tab
        log_frame = ttk.Frame(self.notebook)
        self.notebook.add(log_frame, text="Log")
        self.log_text = tk.Text(log_frame, height=30, width=80, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text['yscrollcommand'] = scrollbar.set
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Results Tab
        results_frame = ttk.Frame(self.notebook)
        self.notebook.add(results_frame, text="Results")
        self.results_text = tk.Text(results_frame, height=30, width=80, state=tk.DISABLED, font=("Courier", 9))
        scrollbar2 = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.results_text.yview)
        self.results_text['yscrollcommand'] = scrollbar2.set
        self.results_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar2.pack(side=tk.RIGHT, fill=tk.Y)

        # Graphs Tab (if matplotlib available)
        if MATPLOTLIB_AVAILABLE:
            graph_frame = ttk.Frame(self.notebook)
            self.notebook.add(graph_frame, text="Graphs")
            self.fig = Figure(figsize=(10, 6), dpi=80)
            self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def list_ports(self):
        try:
            import serial.tools.list_ports
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.current(0)
        except Exception as e:
            self.log_message(f"Error listing ports: {e}")

    def toggle_connection(self):
        if self.serial_receiver is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return

        try:
            self.serial_receiver = SerialReceiver(port)
            self.serial_receiver.message_callback = self.handle_message
            self.serial_receiver.start()

            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.start_btn.config(state=tk.NORMAL)
            self.log_message(f"✓ Connected to {port}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {e}")
            self.serial_receiver = None

    def disconnect(self):
        if self.serial_receiver:
            self.serial_receiver.stop()
            self.serial_receiver.join(timeout=2)
            self.serial_receiver = None

        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.log_message("Disconnected")

    def handle_message(self, message):
        self.log_message(message)

        if message.startswith("STATUS:"):
            parts = message.split(":")
            try:
                depth = float(parts[1])
                pressure = float(parts[2])
                pwm = int(parts[3])
                count = int(parts[4])
                self.session.add_point(0, depth, pressure, pwm)
                self.received_data_count = count
                self.update_display()
            except:
                pass

        elif message.startswith("DATA:"):
            parts = message.split(":")
            try:
                idx = int(parts[1])
                time_ms = int(parts[2])
                depth = float(parts[3])
                pressure = float(parts[4])
                pwm = int(parts[5])
                self.session.add_point(time_ms, depth, pressure, pwm)
                self.received_data_count += 1
                if self.received_data_count % 100 == 0:
                    self.update_display()
            except:
                pass

        elif "TRANSMISSION_COMPLETE" in message:
            self.log_message("✓ Data transmission complete, analyzing...")
            self.root.after(1000, self.analyze_test_results)

    def log_message(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.root.update()

    def update_display(self):
        self.points_label.config(text=str(self.received_data_count))
        if len(self.session.data_points) > 0:
            duration = self.session.data_points[-1].time_ms / 1000.0
            self.duration_label.config(text=f"{duration:.1f} s")

    def start_test(self):
        if not self.serial_receiver:
            messagebox.showerror("Error", "Not connected")
            return

        self.session = CalibrationSession()
        self.received_data_count = 0
        self.test_running = True

        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        self.log_message("=" * 60)
        self.log_message("CALIBRATION TEST STARTED")
        self.log_message("Testing 7 motor speeds: 80, 100, 120, 140, 160, 180, 200 PWM")
        self.log_message("=" * 60)

        self.serial_receiver.send("START_TEST")

    def stop_test(self):
        if not self.serial_receiver:
            return

        self.test_running = False
        self.stop_btn.config(state=tk.DISABLED)
        self.log_message("\nEmergency stop sent")
        self.serial_receiver.send("STOP")

    def analyze_test_results(self):
        if len(self.session.data_points) == 0:
            messagebox.showerror("Error", "No data collected")
            return

        self.log_message("\n" + "=" * 60)
        self.log_message("ANALYZING RESULTS")
        self.log_message("=" * 60)

        self.session.analyze_results()

        # Display results
        results_text = "\n╔════════════════════════════════════════════════╗\n"
        results_text += "║       FLOAT CALIBRATION RESULTS (2m test)      ║\n"
        results_text += "╠════════════════════════════════════════════════╣\n"

        results_text += "║ DESCENT RATES (m/s):                           ║\n"
        results_text += "╟────────────────────────────────────────────────╢\n"
        for pwm in sorted(self.session.results['descent_rates'].keys()):
            rate = self.session.results['descent_rates'][pwm]
            marker = " ← OPTIMAL" if pwm == self.session.results['recommended_descent_pwm'] else ""
            results_text += f"║ PWM {pwm:3d}:  {rate:.4f} m/s{marker:20} ║\n"

        results_text += "╠════════════════════════════════════════════════╣\n"
        results_text += "║ ASCENT RATES (m/s):                            ║\n"
        results_text += "╟────────────────────────────────────────────────╢\n"
        for pwm in sorted(self.session.results['ascent_rates'].keys()):
            rate = self.session.results['ascent_rates'][pwm]
            marker = " ← OPTIMAL" if pwm == self.session.results['recommended_ascent_pwm'] else ""
            results_text += f"║ PWM {pwm:3d}:  {rate:.4f} m/s{marker:20} ║\n"

        results_text += "╠════════════════════════════════════════════════╣\n"
        results_text += f"║ UPDATE Float.ino TOP VARIABLES:                ║\n"
        results_text += f"║ int DESCENT_PWM = {self.session.results['recommended_descent_pwm']};                 ║\n"
        results_text += f"║ int ASCENT_PWM = {self.session.results['recommended_ascent_pwm']};                  ║\n"
        results_text += "╚════════════════════════════════════════════════╝\n"

        self.results_text.config(state=tk.NORMAL)
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, results_text)
        self.results_text.config(state=tk.DISABLED)

        # Update labels
        self.desc_pwm_label.config(text=str(self.session.results['recommended_descent_pwm']))
        self.asc_pwm_label.config(text=str(self.session.results['recommended_ascent_pwm']))

        # Enable export buttons
        for btn in [self.export_csv_btn, self.report_btn]:
            if hasattr(btn, 'config'):
                btn.config(state=tk.NORMAL)

        # Save config
        config_file = self.session.save_config()
        self.log_message(f"✓ Calibration saved to {config_file}")

        # Generate graphs
        if MATPLOTLIB_AVAILABLE:
            self.update_graphs()

        self.log_message("Analysis complete!")

    def update_graphs(self):
        self.fig.clear()

        # Depth vs Time
        ax1 = self.fig.add_subplot(2, 2, 1)
        times = [p.time_ms / 1000.0 for p in self.session.data_points]
        depths = [p.depth_m for p in self.session.data_points]
        ax1.plot(times, depths, 'b-', linewidth=1)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Depth (m)")
        ax1.set_title("Depth Profile")
        ax1.grid(True, alpha=0.3)

        # Descent Rate vs PWM
        ax2 = self.fig.add_subplot(2, 2, 2)
        pwms = sorted(self.session.results['descent_rates'].keys())
        rates = [self.session.results['descent_rates'][p] for p in pwms]
        ax2.plot(pwms, rates, 'go-', linewidth=2, markersize=6)
        ax2.axhline(y=0.5, color='r', linestyle='--', label='Target: 0.5 m/s')
        ax2.set_xlabel("Motor PWM")
        ax2.set_ylabel("Descent Rate (m/s)")
        ax2.set_title("Descent Rates")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Ascent Rate vs PWM
        ax3 = self.fig.add_subplot(2, 2, 3)
        pwms = sorted(self.session.results['ascent_rates'].keys())
        rates = [self.session.results['ascent_rates'][p] for p in pwms]
        ax3.plot(pwms, rates, 'ro-', linewidth=2, markersize=6)
        ax3.axhline(y=0.5, color='g', linestyle='--', label='Target: 0.5 m/s')
        ax3.set_xlabel("Motor PWM")
        ax3.set_ylabel("Ascent Rate (m/s)")
        ax3.set_title("Ascent Rates")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Motor Power vs Time
        ax4 = self.fig.add_subplot(2, 2, 4)
        pwms = [p.motor_pwm for p in self.session.data_points]
        ax4.plot(times, pwms, 'k-', linewidth=1)
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Motor PWM")
        ax4.set_title("Motor Control")
        ax4.grid(True, alpha=0.3)

        self.fig.tight_layout()
        self.canvas.draw()

    def export_csv(self):
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        if filename:
            self.session.export_csv(filename)
            self.log_message(f"✓ Exported to {filename}")

    def generate_report(self):
        filename = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])
        if not filename:
            return

        with open(filename, 'w') as f:
            f.write("╔════════════════════════════════════════════════════════════╗\n")
            f.write("║     BLADDERFISH FLOAT CALIBRATION REPORT (2m Shallow)      ║\n")
            f.write("╚════════════════════════════════════════════════════════════╝\n\n")

            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Data Points: {len(self.session.data_points)}\n")
            f.write(f"Test Duration: {self.session.data_points[-1].time_ms/1000:.1f} seconds\n\n")

            f.write("UPDATE Float.ino WITH THESE VALUES:\n")
            f.write("────────────────────────────────────────\n")
            f.write(f"int DESCENT_PWM = {self.session.results['recommended_descent_pwm']};\n")
            f.write(f"int ASCENT_PWM = {self.session.results['recommended_ascent_pwm']};\n\n")

            f.write("DESCENT RATES:\n")
            f.write("-" * 40 + "\n")
            for pwm in sorted(self.session.results['descent_rates'].keys()):
                rate = self.session.results['descent_rates'][pwm]
                f.write(f"PWM {pwm:3d}: {rate:.4f} m/s\n")

            f.write("\nASCENT RATES:\n")
            f.write("-" * 40 + "\n")
            for pwm in sorted(self.session.results['ascent_rates'].keys()):
                rate = self.session.results['ascent_rates'][pwm]
                f.write(f"PWM {pwm:3d}: {rate:.4f} m/s\n")

        self.log_message(f"✓ Report saved to {filename}")

if __name__ == "__main__":
    root = tk.Tk()
    app = CalibrationGUI(root)
    root.mainloop()
