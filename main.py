"""
Coil winder control GUI.
Requires: pip install pyserial
"""
import math
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUDRATE = 9600
MAX_RPM = 300                  # must match MAX_RPM in the Arduino sketch
CARRIAGE_STEPS_PER_MM = 400    # 3200 microsteps / 8 mm (TR8x8), must match the sketch
DEFAULT_RPM = 60
SLIDER_DELAY_MS = 150          # wait this long after the slider stops moving before sending
READY_TIMEOUT_MS = 3000


def set_widget_state(widget, enabled):
    try:
        widget.state(["!disabled"] if enabled else ["disabled"])
    except (AttributeError, tk.TclError):
        pass


class MotorPanel(ttk.LabelFrame):
    """Manual controls for one motor: speed, direction, start/stop."""

    def __init__(self, master, title, on_change):
        super().__init__(master, text=title, padding=10)
        self.on_change = on_change
        self.running = False
        self._after_id = None

        self.speed = tk.IntVar(value=DEFAULT_RPM)
        self.direction = tk.IntVar(value=1)
        self.status = tk.StringVar()

        ttk.Label(self, text="Speed (rpm)").grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Scale(self, from_=0, to=MAX_RPM, variable=self.speed, length=220,
                  command=self._on_slider).grid(row=1, column=0, columnspan=2, sticky="ew")

        spin = ttk.Spinbox(self, from_=0, to=MAX_RPM, increment=1, width=6,
                           textvariable=self.speed, command=self._changed)
        spin.grid(row=1, column=2, padx=(8, 0))
        spin.bind("<Return>", lambda e: self._changed())
        spin.bind("<FocusOut>", lambda e: self._changed())

        ttk.Radiobutton(self, text="Forward", value=1, variable=self.direction,
                        command=self._changed).grid(row=2, column=0, sticky="w", pady=8)
        ttk.Radiobutton(self, text="Reverse", value=-1, variable=self.direction,
                        command=self._changed).grid(row=2, column=1, sticky="w", pady=8)

        ttk.Button(self, text="Start", command=lambda: self.set_running(True, notify=True)
                   ).grid(row=3, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(self, text="Stop", command=lambda: self.set_running(False, notify=True)
                   ).grid(row=3, column=1, sticky="ew", padx=(4, 0))

        ttk.Label(self, textvariable=self.status, font=("", 10, "bold")
                  ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self._update_status()

    def get_speed(self):
        try:
            value = int(self.speed.get())
        except (tk.TclError, ValueError):
            value = 0
        return max(0, min(MAX_RPM, value))

    def signed_speed(self):
        """Value sent to the Arduino: 0 if stopped, negative if reverse."""
        if not self.running:
            return 0
        return self.get_speed() * self.direction.get()

    def set_running(self, running, notify=False):
        self.running = running
        self._update_status()
        if notify:
            self.on_change()

    def set_enabled(self, enabled):
        for child in self.winfo_children():
            set_widget_state(child, enabled)

    def _on_slider(self, value):
        self.speed.set(round(float(value)))
        if self._after_id:
            self.after_cancel(self._after_id)
        self._after_id = self.after(SLIDER_DELAY_MS, self._slider_done)

    def _slider_done(self):
        self._after_id = None
        self._changed()

    def _changed(self):
        self.speed.set(self.get_speed())   # clamp / clean typed value
        self._update_status()
        self.on_change()

    def _update_status(self):
        if self.running:
            way = "forward" if self.direction.get() > 0 else "reverse"
            self.status.set(f"RUNNING - {self.get_speed()} rpm {way}")
        else:
            self.status.set("STOPPED")


class WindingPanel(ttk.LabelFrame):
    """Winding parameters, computed info, start/pause/resume/abort and progress."""

    FIELDS = [
        ("core_d", "Bobbin core diameter (mm)", "29"),
        ("length", "Bobbin length (mm)", "11.5"),
        ("wire_d", "Wire diameter (mm)", "0.53"),
        ("tpl", "Turns per layer", "20"),
        ("layers", "Number of layers", "1"),
        ("rpm", "Winding speed (rpm)", "60"),
    ]

    def __init__(self, master, on_start, on_pause, on_resume, on_abort):
        super().__init__(master, text="Winding", padding=10)
        self.on_start = on_start
        self.total_turns = 0
        self.total_layers = 0
        self.vars = {}
        self.entries = []

        for row, (key, label, default) in enumerate(self.FIELDS):
            ttk.Label(self, text=label).grid(row=row, column=0, sticky="w")
            var = tk.StringVar(value=default)
            var.trace_add("write", lambda *_: self.update_info())
            entry = ttk.Entry(self, textvariable=var, width=8)
            entry.grid(row=row, column=1, sticky="w", padx=5, pady=1)
            self.vars[key] = var
            self.entries.append(entry)

        r = len(self.FIELDS)
        self.direction = tk.IntVar(value=1)
        dir_frame = ttk.Frame(self)
        dir_frame.grid(row=r, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Label(dir_frame, text="First layer carriage direction:").pack(side="left")
        for text, value in (("Forward", 1), ("Reverse", -1)):
            rb = ttk.Radiobutton(dir_frame, text=text, value=value, variable=self.direction)
            rb.pack(side="left", padx=3)
            self.entries.append(rb)

        self.info = tk.StringVar()
        ttk.Label(self, textvariable=self.info, justify="left"
                  ).grid(row=0, column=2, rowspan=r + 1, sticky="nw", padx=(25, 0))

        buttons = ttk.Frame(self)
        buttons.grid(row=r + 1, column=0, columnspan=3, sticky="w", pady=(8, 4))
        self.start_btn = ttk.Button(buttons, text="Start winding", command=self._start)
        self.pause_btn = ttk.Button(buttons, text="Pause", command=on_pause)
        self.resume_btn = ttk.Button(buttons, text="Resume", command=on_resume)
        self.abort_btn = ttk.Button(buttons, text="Abort", command=on_abort)
        for b in (self.start_btn, self.pause_btn, self.resume_btn, self.abort_btn):
            b.pack(side="left", padx=3)

        self.progress = ttk.Progressbar(self, length=500, mode="determinate")
        self.progress.grid(row=r + 2, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        self.progress_text = tk.StringVar(value="Idle")
        ttk.Label(self, textvariable=self.progress_text
                  ).grid(row=r + 3, column=0, columnspan=3, sticky="w")

        self.set_active(False)
        self.update_info()

    def _num(self, key, cast=float):
        return cast(self.vars[key].get().strip().replace(",", "."))

    def read_params(self):
        try:
            p = {
                "core_d": self._num("core_d"),
                "length": self._num("length"),
                "wire_d": self._num("wire_d"),
                "tpl": self._num("tpl", int),
                "layers": self._num("layers", int),
                "rpm": self._num("rpm"),
            }
        except ValueError:
            raise ValueError("Invalid value: check all fields.")
        if p["core_d"] <= 0 or p["length"] <= 0:
            raise ValueError("Bobbin dimensions must be > 0.")
        if not 0.05 <= p["wire_d"] <= 3:
            raise ValueError("Wire diameter: 0.05 to 3 mm.")
        if not 1 <= p["tpl"] <= 1000:
            raise ValueError("Turns per layer: 1 to 1000.")
        if not 1 <= p["layers"] <= 200:
            raise ValueError("Layers: 1 to 200.")
        if not 0 < p["rpm"] <= MAX_RPM:
            raise ValueError(f"Speed: 0 to {MAX_RPM} rpm.")
        p["direction"] = self.direction.get()
        return p

    def update_info(self):
        try:
            p = self.read_params()
        except ValueError as e:
            self.info.set(str(e))
            return
        width = p["tpl"] * p["wire_d"]
        max_tpl = int(p["length"] / p["wire_d"] + 1e-9)
        total = p["tpl"] * p["layers"]
        outer = p["core_d"] + 2 * p["layers"] * p["wire_d"]
        wire_m = sum(p["tpl"] * math.pi * (p["core_d"] + p["wire_d"] * (2 * i + 1))
                     for i in range(p["layers"])) / 1000
        minutes = total / p["rpm"]
        carriage_speed = p["rpm"] * p["wire_d"] / 60
        steps = p["wire_d"] * CARRIAGE_STEPS_PER_MM

        lines = [
            f"Layer width: {width:.2f} mm (max {max_tpl} turns/layer)",
            f"Total turns: {total}",
            f"Outer diameter: ~{outer:.1f} mm",
            f"Wire length: ~{wire_m:.1f} m",
            f"Duration: ~{minutes:.1f} min",
            f"Carriage speed: {carriage_speed:.2f} mm/s",
        ]
        if width > p["length"] + 1e-9:
            lines.append("WARNING: layer wider than the bobbin!")
        if abs(steps - round(steps)) > 1e-6:
            lines.append(f"Note: pitch rounded to {round(steps) / CARRIAGE_STEPS_PER_MM:.4f} mm")
        self.info.set("\n".join(lines))

    def _start(self):
        try:
            p = self.read_params()
        except ValueError as e:
            messagebox.showerror("Invalid parameters", str(e))
            return
        if p["tpl"] * p["wire_d"] > p["length"] + 1e-9:
            if not messagebox.askyesno("Layer too wide",
                                       "The layer is wider than the bobbin. Start anyway?"):
                return
        self.on_start(p)

    def prepare(self, total_turns, total_layers):
        self.total_turns = total_turns
        self.total_layers = total_layers
        self.progress["maximum"] = total_turns
        self.progress["value"] = 0
        self.progress_text.set("Starting...")

    def set_progress(self, turn, layer):
        self.progress["value"] = turn
        self.progress_text.set(f"Turn {turn} / {self.total_turns}  -  "
                               f"Layer {layer} / {self.total_layers}")

    def set_text(self, text):
        self.progress_text.set(text)

    def set_active(self, active):
        for w in self.entries:
            set_widget_state(w, not active)
        set_widget_state(self.start_btn, not active)
        for b in (self.pause_btn, self.resume_btn, self.abort_btn):
            set_widget_state(b, active)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Coil Winder Control")
        self.resizable(False, False)

        self.ser = None
        self.ready = False
        self.winding = False
        self.connection_id = 0
        self.last_cmd = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()

        # --- Connection bar ---
        bar = ttk.Frame(self, padding=10)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(bar, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.pack(side="left", padx=5)
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)
        self.conn_status = tk.StringVar(value="Disconnected")
        ttk.Label(bar, textvariable=self.conn_status).pack(side="left", padx=10)

        # --- Manual control ---
        self.motor1 = MotorPanel(self, "Motor 1 - Spool (manual)", self.send_state)
        self.motor1.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        self.motor2 = MotorPanel(self, "Motor 2 - Carriage (manual)", self.send_state)
        self.motor2.grid(row=1, column=1, padx=10, pady=5, sticky="nsew")

        # --- Winding ---
        self.winding_panel = WindingPanel(self, self.start_winding,
                                          lambda: self.send_line("P"),
                                          lambda: self.send_line("R"),
                                          lambda: self.send_line("X"))
        self.winding_panel.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # --- Emergency stop ---
        tk.Button(self, text="STOP ALL", bg="#c62828", fg="white", font=("", 12, "bold"),
                  command=self.stop_all).grid(row=3, column=0, columnspan=2,
                                              sticky="ew", padx=10, pady=5)

        # --- Log ---
        self.log = tk.Text(self, height=8, width=80, state="disabled")
        self.log.grid(row=4, column=0, columnspan=2, padx=10, pady=(0, 10))

        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self.poll_serial)

    # ---------- Connection ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            self.ser = None
            messagebox.showerror("Connection error", str(e))
            return

        self.ready = False
        self.last_cmd = None
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self.reader_loop, args=(self.ser,), daemon=True)
        self.reader_thread.start()

        self.connect_btn.config(text="Disconnect")
        self.conn_status.set("Waiting for Arduino...")
        self.log_line(f"Opened {port}")

        # The Uno resets when the port opens: wait for "READY", with a fallback timeout
        self.connection_id += 1
        cid = self.connection_id
        self.after(READY_TIMEOUT_MS, lambda: self.ready_timeout(cid))

    def ready_timeout(self, cid):
        if cid == self.connection_id and self.ser and not self.ready:
            self.log_line("No READY received, continuing anyway")
            self.on_ready()

    def on_ready(self):
        # Arduino (re)started: everything is stopped, so sync the GUI
        self.ready = True
        self.last_cmd = None
        self.set_winding(False)
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.winding_panel.set_text("Idle")
        self.conn_status.set(f"Connected ({self.port_var.get()})")
        self.log_line("Arduino ready")

    def disconnect(self):
        if self.ser is None:
            return
        try:
            if self.ready:
                self.ser.write(b"X\n")         # abort winding / stop motors before leaving
                self.ser.flush()
        except (serial.SerialException, OSError):
            pass
        self.reader_stop.set()
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        if self.reader_thread:
            self.reader_thread.join(timeout=0.5)

        self.ser = None
        self.ready = False
        self.set_winding(False)
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.connect_btn.config(text="Connect")
        self.conn_status.set("Disconnected")
        self.log_line("Disconnected")

    # ---------- Serial I/O ----------
    def reader_loop(self, ser):
        """Background thread: puts received lines into a queue."""
        while not self.reader_stop.is_set():
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                if not self.reader_stop.is_set():
                    self.rx_queue.put(("error", str(e)))
                return
            if raw:
                self.rx_queue.put(("line", raw.decode(errors="ignore").strip()))

    def poll_serial(self):
        """GUI thread: handles what the reader thread received."""
        while not self.rx_queue.empty():
            kind, data = self.rx_queue.get_nowait()
            if kind == "error":
                self.log_line(f"Serial error: {data}")
                self.disconnect()
            else:
                self.handle_line(data)
        self.after(50, self.poll_serial)

    def handle_line(self, line):
        if not line or line == "OK":
            return
        if line == "READY":
            self.on_ready()
        elif line == "WINDING":
            self.set_winding(True)
            self.log_line("Winding started")
        elif line.startswith("T;"):
            try:
                _, turn, layer = line.split(";")
                self.winding_panel.set_progress(int(turn), int(layer))
            except ValueError:
                self.log_line(f"< {line}")
        elif line == "DONE":
            self.set_winding(False)
            self.winding_panel.set_text("Done - motors holding the wire (press Stop to release)")
            self.log_line("Winding complete")
        elif line == "STOPPED":
            self.set_winding(False)
            self.winding_panel.set_text("Aborted - motors holding the wire (press Stop to release)")
            self.log_line("Winding aborted")
        elif line in ("PAUSED", "RESUMED", "ABORTING"):
            self.winding_panel.set_text(line.capitalize())
            self.log_line(line.capitalize())
        else:
            self.log_line(f"< {line}")

    def send_line(self, cmd):
        if not (self.ser and self.ready):
            return False
        try:
            self.ser.write((cmd + "\n").encode())
            self.log_line(f"> {cmd}")
            return True
        except (serial.SerialException, OSError) as e:
            self.log_line(f"Write error: {e}")
            self.disconnect()
            return False

    def send_state(self):
        """Manual mode: sends both motor speeds."""
        if self.winding:
            return
        cmd = f"S;{self.motor1.signed_speed()};{self.motor2.signed_speed()}"
        if cmd != self.last_cmd and self.send_line(cmd):
            self.last_cmd = cmd

    # ---------- Winding ----------
    def start_winding(self, p):
        if not (self.ser and self.ready):
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        if self.motor1.running or self.motor2.running:
            messagebox.showwarning("Motors running", "Stop both motors before starting the winding.")
            return
        self.winding_panel.prepare(p["tpl"] * p["layers"], p["layers"])
        self.send_line(f"W;{p['wire_d']:g};{p['tpl']};{p['layers']};{p['rpm']:g};{p['direction']}")

    def set_winding(self, active):
        self.winding = active
        self.last_cmd = None
        self.winding_panel.set_active(active)
        self.motor1.set_enabled(not active)
        self.motor2.set_enabled(not active)

    # ---------- Actions ----------
    def stop_all(self):
        if self.winding:
            self.send_line("X")
            return
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.last_cmd = None
        self.send_state()

    def log_line(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()