"""
Coil winder control GUI.
Requires: pip install pyserial
"""
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUDRATE = 9600
MAX_RPM = 300          # must match MAX_RPM in the Arduino sketch
DEFAULT_RPM = 60
SLIDER_DELAY_MS = 150  # wait this long after the slider stops moving before sending
READY_TIMEOUT_MS = 3000


class MotorPanel(ttk.LabelFrame):
    """Controls for one motor: speed, direction, start/stop."""

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


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Coil Winder Control")
        self.resizable(False, False)

        self.ser = None
        self.ready = False
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

        # --- Motor panels ---
        self.motor1 = MotorPanel(self, "Motor 1 - Spool", self.send_state)
        self.motor1.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        self.motor2 = MotorPanel(self, "Motor 2 - Carriage", self.send_state)
        self.motor2.grid(row=1, column=1, padx=10, pady=5, sticky="nsew")

        # --- Emergency stop ---
        tk.Button(self, text="STOP ALL", bg="#c62828", fg="white", font=("", 12, "bold"),
                  command=self.stop_all).grid(row=2, column=0, columnspan=2,
                                              sticky="ew", padx=10, pady=5)

        # --- Log ---
        self.log = tk.Text(self, height=8, width=70, state="disabled")
        self.log.grid(row=3, column=0, columnspan=2, padx=10, pady=(0, 10))

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
        # Arduino (re)started: its motors are stopped, so sync the GUI
        self.ready = True
        self.last_cmd = None
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.conn_status.set(f"Connected ({self.port_var.get()})")
        self.log_line("Arduino ready")

    def disconnect(self):
        if self.ser is None:
            return
        try:
            if self.ready:
                self.ser.write(b"S;0;0\n")   # stop motors before leaving
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
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.connect_btn.config(text="Connect")
        self.conn_status.set("Disconnected")
        self.log_line("Disconnected")

    # ---------- Serial I/O ----------
    def reader_loop(self, ser):
        """Runs in a background thread: puts received lines into a queue."""
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
        """Runs in the GUI thread: handles what the reader thread received."""
        while not self.rx_queue.empty():
            kind, data = self.rx_queue.get_nowait()
            if kind == "line":
                if data == "READY":
                    self.on_ready()
                elif data and data != "OK":
                    self.log_line(f"< {data}")
            elif kind == "error":
                self.log_line(f"Serial error: {data}")
                self.disconnect()
        self.after(50, self.poll_serial)

    def send_state(self):
        if not (self.ser and self.ready):
            return
        cmd = f"S;{self.motor1.signed_speed()};{self.motor2.signed_speed()}"
        if cmd == self.last_cmd:
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.last_cmd = cmd
            self.log_line(f"> {cmd}")
        except (serial.SerialException, OSError) as e:
            self.log_line(f"Write error: {e}")
            self.disconnect()

    # ---------- Actions ----------
    def stop_all(self):
        self.motor1.set_running(False)
        self.motor2.set_running(False)
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