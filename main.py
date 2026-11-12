"""
Coil winder & current source - control software.

Tabs
  Bobineuse : manual control of the two steppers + automatic winding   (firmware bobin_v0)
  Courant   : current regulation, PID tuning, live chart, CSV logging  (firmware current)
  Firmware  : compile (optional, arduino-cli) and upload the sketches to the Arduino

Requires: pip install pyserial
Portable .exe: run build_exe.bat
"""
import bisect
import csv
import json
import math
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog

import serial
import serial.tools.list_ports

# ============================================================================ Settings
APP_NAME = "Coil Winder Control"
BAUDRATE = 115200                # must match Serial.begin() in both sketches
MAX_RPM = 300                    # must match MAX_RPM in bobin_v0.ino
CARRIAGE_STEPS_PER_MM = 400      # 3200 microsteps / 8 mm (TR8x8), must match bobin_v0.ino
DEFAULT_RPM = 60
CURRENT_MAX_MA = 1000            # must match MAX_SETPOINT_MA in current.ino
PWM_MAX = 255

SLIDER_DELAY_MS = 150            # wait this long after a slider stops moving before sending
READY_TIMEOUT_MS = 3000          # the board resets when the port opens: wait for READY...
IDENTIFY_TIMEOUT_MS = 1000       # ...then ask "?" and wait this long before giving up
UI_POLL_MS = 20                  # how often the GUI thread handles serial data
CHART_REFRESH_MS = 100           # chart redraw period (10 fps)
CHART_MAX_SAMPLES = 100_000      # samples kept in memory (chart + CSV export)
LOG_MAX_LINES = 1500

SETTINGS_FILE = "coil_winder_settings.json"

# Firmwares known by the application. The id is what the sketch prints in "READY;<id>".
FIRMWARES = {
    "WINDER": {"label": "Bobineuse (moteurs) - bobin_v0", "sketch": "bobin_v0", "libs": ()},
    "CURRENT": {"label": "Générateur de courant - current", "sketch": "current",
                "libs": ("Adafruit INA219",)},
}

# Boards supported by the built-in uploader (ATmega328P, STK500v1 bootloader)
BOARDS = {
    "Arduino Uno": {"fqbn": "arduino:avr:uno", "baud": 115200, "max_size": 32256},
    "Arduino Nano": {"fqbn": "arduino:avr:nano:cpu=atmega328", "baud": 115200, "max_size": 30720},
    "Arduino Nano (ancien bootloader)": {"fqbn": "arduino:avr:nano:cpu=atmega328old",
                                         "baud": 57600, "max_size": 30720},
}
ATMEGA328P_SIGNATURE = b"\x1e\x95\x0f"

COLOR_OK = "#2e7d32"
COLOR_WARN = "#b26a00"
COLOR_ERROR = "#c62828"
COLOR_IDLE = "#555555"

# ============================================================================ Paths
FROZEN = getattr(sys, "frozen", False)
APP_DIR = os.path.dirname(sys.executable if FROZEN else os.path.abspath(__file__))
BUNDLE_DIR = getattr(sys, "_MEIPASS", APP_DIR)     # PyInstaller extracts bundled files here


def find_resource(*parts):
    """Looks next to the app first (user can drop updated files), then in the bundle."""
    for base in (APP_DIR, BUNDLE_DIR):
        path = os.path.join(base, *parts)
        if os.path.exists(path):
            return path
    return None


class Settings:
    """Small JSON settings store, next to the app (portable) or in the home folder."""

    def __init__(self):
        self.path = os.path.join(APP_DIR, SETTINGS_FILE)
        self.data = {}
        for path in (self.path, self._fallback_path()):
            try:
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
            except (OSError, ValueError):
                continue
            if isinstance(data, dict):
                self.data, self.path = data, path
                break

    @staticmethod
    def _fallback_path():
        return os.path.join(os.path.expanduser("~"), "." + SETTINGS_FILE)

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def save(self):
        for path in (self.path, self._fallback_path()):
            try:
                tmp = path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(self.data, f, indent=2, ensure_ascii=False)
                os.replace(tmp, path)
                self.path = path
                return
            except OSError:
                continue


# ============================================================================ Helpers
def set_widget_state(widget, enabled):
    try:
        widget.state(["!disabled"] if enabled else ["disabled"])
    except (AttributeError, tk.TclError):
        try:
            widget.configure(state="normal" if enabled else "disabled")
        except tk.TclError:
            pass


def set_tree_state(container, enabled):
    for child in container.winfo_children():
        set_widget_state(child, enabled)
        set_tree_state(child, enabled)


def parse_number(text, cast=float):
    return cast(float(str(text).strip().replace(",", ".")))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def nice_ceil(value):
    """Smallest 'nice' number (1, 2, 2.5, 5 x 10^n) >= value."""
    if value <= 0:
        return 1.0
    exp = 10 ** math.floor(math.log10(value))
    for m in (1, 2, 2.5, 5, 10):
        if value <= m * exp + 1e-12:
            return m * exp
    return 10 * exp


def format_duration(seconds):
    seconds = int(round(max(0, seconds)))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


def list_serial_ports():
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    return [(p.device, p.description or "") for p in ports]


class Debouncer:
    """Calls callback once the trigger() calls stop for delay_ms (for sliders)."""

    def __init__(self, widget, delay_ms, callback):
        self.widget, self.delay_ms, self.callback = widget, delay_ms, callback
        self._after_id = None

    def trigger(self):
        if self._after_id:
            self.widget.after_cancel(self._after_id)
        self._after_id = self.widget.after(self.delay_ms, self._fire)

    def _fire(self):
        self._after_id = None
        self.callback()


# ============================================================================ Serial link
class SerialLink:
    """Serial port + background reader thread.

    Received lines are delivered in batches to on_lines(list_of_str) in the GUI thread
    (through post), errors to on_error(message). Lines from a previous connection are dropped.
    """

    def __init__(self, post, on_lines, on_error):
        self._post = post
        self._on_lines = on_lines
        self._on_error = on_error
        self.ser = None
        self._stop = None
        self._thread = None
        self._generation = 0

    @property
    def is_open(self):
        return self.ser is not None

    @property
    def port(self):
        return self.ser.port if self.ser else None

    def open(self, port):
        self.close()
        ser = serial.Serial(port, BAUDRATE, timeout=0.1, write_timeout=1)
        self.ser = ser
        self._generation += 1
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True,
                                        args=(ser, self._stop, self._generation))
        self._thread.start()

    def close(self):
        if self.ser is None:
            return
        self._stop.set()
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        if self._thread:
            self._thread.join(timeout=0.5)
        self.ser = None
        self._thread = None
        self._generation += 1

    def write_line(self, text):
        self.ser.write((text + "\n").encode("ascii", errors="replace"))

    def _reader(self, ser, stop, generation):
        """Background thread: reads whatever is available (much faster than readline)."""
        pending = b""
        while not stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                if not stop.is_set():
                    self._post(self._deliver_error, generation, str(e))
                return
            if not chunk:
                continue
            pending += chunk
            if b"\n" not in chunk:
                if len(pending) > 4096:          # garbage without line endings (wrong baud rate)
                    pending = b""
                continue
            *lines, pending = pending.split(b"\n")
            self._post(self._deliver, generation,
                       [raw.decode("ascii", errors="ignore").strip() for raw in lines])

    def _deliver(self, generation, lines):
        if generation == self._generation and self.ser is not None:
            self._on_lines(lines)

    def _deliver_error(self, generation, message):
        if generation == self._generation and self.ser is not None:
            self._on_error(message)


# ============================================================================ Firmware upload
class UploadError(Exception):
    pass


def parse_intel_hex(text):
    """Intel HEX -> flash image (bytes, starting at address 0, holes filled with 0xFF)."""
    image = bytearray()
    base = 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            if not line.startswith(":"):
                raise ValueError
            rec = bytes.fromhex(line[1:])
        except ValueError:
            raise UploadError(f"Fichier HEX invalide (ligne {n})") from None
        if len(rec) < 5 or len(rec) != rec[0] + 5 or sum(rec) & 0xFF:
            raise UploadError(f"Fichier HEX corrompu (ligne {n})")
        count, address, rtype, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if rtype == 0:
            start = base + address
            if len(image) < start + count:
                image.extend(b"\xff" * (start + count - len(image)))
            image[start:start + count] = data
        elif rtype == 1:
            break
        elif rtype == 2:
            base = int.from_bytes(data, "big") << 4
        elif rtype == 4:
            base = int.from_bytes(data, "big") << 16
    if not image:
        raise UploadError("Fichier HEX vide")
    return bytes(image)


class Stk500Uploader:
    """Minimal STK500v1 client: enough for the Uno / Nano bootloaders (Optiboot, ATmegaBOOT).

    Needs nothing but pyserial, so the portable exe can flash a board without avrdude.
    """
    STK_INSYNC, STK_OK, CRC_EOP = 0x14, 0x10, 0x20
    PAGE_SIZE = 128                                  # ATmega328P flash page, in bytes

    def __init__(self, port, baud, log=lambda text: None, progress=lambda fraction: None,
                 serial_factory=serial.Serial):
        self.port, self.baud = port, baud
        self.log, self.progress = log, progress
        self.serial_factory = serial_factory
        self.ser = None

    def upload(self, image, signature=ATMEGA328P_SIGNATURE, verify=True):
        ps = self.PAGE_SIZE
        data = bytes(image) + b"\xff" * (-len(image) % ps)
        pages = range(0, len(data), ps)
        steps, done = len(pages) * (2 if verify else 1), 0

        ser = self.serial_factory()
        ser.port, ser.baudrate = self.port, self.baud
        ser.timeout, ser.write_timeout = 1.0, 2.0
        ser.open()
        self.ser = ser
        try:
            self._sync()
            sig = self._command(b"\x75", 3)
            self.log(f"Signature du microcontrôleur : {sig.hex(' ').upper()}")
            if signature and sig != signature:
                raise UploadError(f"Signature inattendue {sig.hex(' ').upper()} "
                                  f"(ATmega328P attendu : {signature.hex(' ').upper()})")
            self._command(b"\x50")                       # enter programming mode
            self.log(f"Écriture de {len(image)} octets ({len(pages)} pages)...")
            for address in pages:
                self._load_address(address)
                self._command(b"\x64" + ps.to_bytes(2, "big") + b"F" + data[address:address + ps])
                done += 1
                self.progress(done / steps)
            if verify:
                self.log("Vérification...")
                for address in pages:
                    self._load_address(address)
                    if self._command(b"\x74" + ps.to_bytes(2, "big") + b"F", ps) != data[address:address + ps]:
                        raise UploadError(f"Erreur de vérification à l'adresse 0x{address:04X}")
                    done += 1
                    self.progress(done / steps)
            self._command(b"\x51")                       # leave programming mode -> sketch starts
        finally:
            self.ser = None
            ser.close()

    def _reset_board(self):
        """Auto-reset through DTR/RTS, like avrdude's 'arduino' programmer."""
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(0.25)
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def _sync(self):
        timeout = self.ser.timeout
        self.ser.timeout = 0.2
        try:
            for _ in range(3):
                self._reset_board()
                for _ in range(5):
                    self.ser.write(b"\x30\x20")
                    if self.ser.read(2) == b"\x14\x10":
                        time.sleep(0.02)
                        self.ser.reset_input_buffer()     # drop late answers to earlier attempts
                        self.ser.write(b"\x30\x20")
                        if self.ser.read(2) == b"\x14\x10":
                            return
            raise UploadError("Pas de réponse du bootloader. Vérifiez le port, le type de carte "
                              "(essayez « ancien bootloader » pour un Nano) et que le port "
                              "n'est pas utilisé par un autre programme.")
        finally:
            self.ser.timeout = timeout

    def _read(self, n):
        data = self.ser.read(n)
        if len(data) < n:
            raise UploadError("Le bootloader ne répond plus (timeout)")
        return data

    def _command(self, payload, reply_len=0):
        self.ser.write(payload + bytes([self.CRC_EOP]))
        head = self._read(1)
        if head[0] != self.STK_INSYNC:
            raise UploadError(f"Bootloader désynchronisé (reçu 0x{head[0]:02X})")
        data = self._read(reply_len) if reply_len else b""
        if self._read(1)[0] != self.STK_OK:
            raise UploadError("Commande refusée par le bootloader")
        return data

    def _load_address(self, byte_address):
        word = byte_address // 2
        self._command(bytes([0x55, word & 0xFF, word >> 8]))


def find_arduino_cli(custom=None):
    exe = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    candidates = [custom, os.path.join(APP_DIR, exe), os.path.join(APP_DIR, "arduino-cli", exe),
                  shutil.which("arduino-cli")]
    if os.name == "nt":
        ide = os.path.join("Arduino IDE", "resources", "app", "lib", "backend", "resources", exe)
        for base in (os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs"),
                     os.environ.get("ProgramFiles", r"C:\Program Files")):
            candidates.append(os.path.join(base, ide))
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return None


class ArduinoCli:
    """Thin wrapper around arduino-cli (optional: only needed to compile from the sources)."""

    def __init__(self, path, log):
        self.path, self.log = path, log

    def run(self, *args):
        self.log("$ arduino-cli " + " ".join(f'"{a}"' if " " in a else a for a in args))
        try:
            proc = subprocess.Popen([self.path, *args], stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding="utf-8", errors="replace",
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except OSError as e:
            raise UploadError(f"Impossible de lancer arduino-cli : {e}") from None
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                self.log(line)
        if proc.wait() != 0:
            raise UploadError(f"arduino-cli a échoué (code {proc.returncode})")

    def compile(self, sketch_dir, fqbn, out_dir):
        self.run("compile", "--fqbn", fqbn, "--output-dir", out_dir, sketch_dir)
        hex_path = os.path.join(out_dir, os.path.basename(sketch_dir) + ".ino.hex")
        if not os.path.isfile(hex_path):
            raise UploadError("Compilation terminée mais fichier .hex introuvable")
        return hex_path

    def upload(self, hex_path, port, fqbn):
        self.run("upload", "-p", port, "--fqbn", fqbn, "--input-file", hex_path)

    def install_dependencies(self, libraries):
        self.run("core", "update-index")
        self.run("core", "install", "arduino:avr")
        for lib in libraries:
            self.run("lib", "install", lib)


# ============================================================================ Common widgets
class LogView(ttk.Frame):
    def __init__(self, master, height=6, timestamps=True):
        super().__init__(master)
        self.timestamps = timestamps
        self.text = tk.Text(self, height=height, state="disabled", wrap="word",
                            font=("Consolas", 9), relief="flat", background="#fafafa")
        bar = ttk.Scrollbar(self, command=self.text.yview)
        self.text["yscrollcommand"] = bar.set
        self.text.grid(row=0, column=0, sticky="nsew")
        bar.grid(row=0, column=1, sticky="ns")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def write(self, line):
        follow = self.text.yview()[1] >= 0.999
        if self.timestamps:
            line = f"{time.strftime('%H:%M:%S')}  {line}"
        self.text.configure(state="normal")
        self.text.insert("end", line + "\n")
        lines = int(self.text.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.text.delete("1.0", f"{lines - LOG_MAX_LINES}.0")
        self.text.configure(state="disabled")
        if follow:
            self.text.see("end")

    def clear(self):
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")


class PortSelector(ttk.Frame):
    """Serial port combobox (refreshed every time it is opened) + optional connect button/status."""

    def __init__(self, master, initial_port=None, on_toggle=None):
        super().__init__(master)
        self._devices = []
        ttk.Label(self, text="Port :").pack(side="left")
        self.port_var = tk.StringVar()
        self.combo = ttk.Combobox(self, textvariable=self.port_var, width=36, state="readonly",
                                  postcommand=self.refresh)
        self.combo.pack(side="left", padx=5)
        self.button = None
        if on_toggle:
            self.button = ttk.Button(self, text="Connecter", width=13, command=on_toggle)
            self.button.pack(side="left")
        self.status = tk.StringVar(value="Déconnecté")
        self.status_label = ttk.Label(self, textvariable=self.status, font=("", 9, "bold"),
                                      foreground=COLOR_IDLE)
        if on_toggle:
            self.status_label.pack(side="left", padx=10)
        self.refresh(initial_port)

    @property
    def port(self):
        return self.port_var.get().split(" - ")[0].strip()

    def refresh(self, wanted=None):
        wanted = wanted or self.port
        ports = list_serial_ports()
        self._devices = [dev for dev, _ in ports]
        labels = [f"{dev} - {desc}" if desc and desc not in (dev, "n/a") else dev for dev, desc in ports]
        self.combo["values"] = labels
        if wanted in self._devices:
            self.port_var.set(labels[self._devices.index(wanted)])
        elif labels:
            likely = [l for l in labels if any(k in l.lower() for k in ("arduino", "ch340", "usb"))]
            self.port_var.set((likely or labels)[0])
        else:
            self.port_var.set("")

    def set_port(self, device):
        self.refresh(device)

    def set_status(self, text, color=COLOR_IDLE):
        self.status.set(text)
        self.status_label.configure(foreground=color)

    def set_connected(self, connected):
        if self.button:
            self.button.configure(text="Déconnecter" if connected else "Connecter")
        self.combo.configure(state="disabled" if connected else "readonly")


class StripChart(tk.Canvas):
    """Scrolling real-time chart drawn on a Tk canvas (no matplotlib: fast and light exe).

    series: list of dicts {name, color, axis ('left'|'right'), index (position in a sample), unit}
    samples: tuples (t, v1, v2, ...) with t in seconds, increasing.
    """
    ML, MR, MT, MB = 58, 48, 28, 26           # margins (px)
    BG, GRID, AXIS, TEXT = "#ffffff", "#e6e6e6", "#9e9e9e", "#424242"

    def __init__(self, master, series, left_title, right_title, right_max, **kw):
        super().__init__(master, background=self.BG, highlightthickness=1,
                         highlightbackground="#cfcfcf", **kw)
        self.series = series
        self.visible = [tk.BooleanVar(value=True) for _ in series]
        for var in self.visible:
            var.trace_add("write", lambda *_: self.request_redraw())
        self.left_title, self.right_title, self.right_max = left_title, right_title, right_max
        self.times = []
        self.samples = []
        self.window_s = 30.0
        self.paused = False
        self._end_time = None           # frozen right edge while paused
        self._dirty = True
        self._view = None
        self._mouse = None
        self.bind("<Configure>", lambda e: self.request_redraw())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", self._on_leave)
        self.after(CHART_REFRESH_MS, self._tick)

    # ----- data
    def add(self, sample):
        self.times.append(sample[0])
        self.samples.append(sample)
        if len(self.times) > CHART_MAX_SAMPLES * 1.2:      # amortized trimming
            cut = len(self.times) - CHART_MAX_SAMPLES
            del self.times[:cut]
            del self.samples[:cut]
        if not self.paused:
            self._dirty = True

    def clear(self):
        self.times.clear()
        self.samples.clear()
        self._end_time = None
        self.request_redraw()

    def set_window(self, seconds):
        self.window_s = float(seconds)
        self.request_redraw()

    def set_paused(self, paused):
        self.paused = paused
        self._end_time = self.times[-1] if paused and self.times else None
        self.request_redraw()

    def request_redraw(self):
        self._dirty = True

    def visible_range(self):
        """(first index, end index, right-edge time) of the samples in the window."""
        if not self.times:
            return 0, 0, 0.0
        end = self._end_time if (self.paused and self._end_time is not None) else self.times[-1]
        i0 = bisect.bisect_left(self.times, end - self.window_s)
        i1 = bisect.bisect_right(self.times, end)
        return i0, i1, end

    def visible_values(self, index):
        i0, i1, _ = self.visible_range()
        return [s[index] for s in self.samples[i0:i1]]

    # ----- drawing
    def _tick(self):
        if self._dirty:
            self._dirty = False
            self._redraw()
        self.after(CHART_REFRESH_MS, self._tick)

    def _redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        x0, y0, x1, y1 = self.ML, self.MT, w - self.MR, h - self.MB
        if x1 - x0 < 50 or y1 - y0 < 40:
            return
        i0, i1, t_end = self.visible_range()
        t_start = t_end - self.window_s
        window = self.samples[i0:i1]
        shown = [s for s, v in zip(self.series, self.visible) if v.get()]

        left_max = 0.0
        for s in shown:
            if s["axis"] == "left" and window:
                left_max = max(left_max, max(smp[s["index"]] for smp in window))
        left_max = nice_ceil(max(left_max * 1.1, 10.0))

        # grid + axes
        for k in range(6):
            y = y1 - (y1 - y0) * k / 5
            self.create_line(x0, y, x1, y, fill=self.GRID)
            self.create_text(x0 - 6, y, text=f"{left_max * k / 5:g}", anchor="e", fill=self.TEXT,
                             font=("", 8))
            self.create_text(x1 + 6, y, text=f"{self.right_max * k / 5:.0f}", anchor="w",
                             fill=self.TEXT, font=("", 8))
        step = next((s for s in (1, 2, 5, 10, 15, 30, 60, 120, 300, 600)
                     if self.window_s / s <= 8), 600)
        t = math.ceil(t_start / step) * step
        while t <= t_end + 1e-9:
            x = x0 + (t - t_start) / self.window_s * (x1 - x0)
            self.create_line(x, y0, x, y1, fill=self.GRID)
            self.create_text(x, y1 + 4, text=f"{t - t_end:.0f} s", anchor="n", fill=self.TEXT,
                             font=("", 8))
            t += step
        self.create_rectangle(x0, y0, x1, y1, outline=self.AXIS)
        self.create_text(x0 - 6, y0 - 4, text=self.left_title, anchor="se", fill=self.TEXT,
                         font=("", 8, "bold"))
        self.create_text(x1 + 6, y0 - 4, text=self.right_title, anchor="sw", fill=self.TEXT,
                         font=("", 8, "bold"))

        # curves (decimated to about one point per pixel)
        stride = max(1, len(window) // max(1, x1 - x0))
        points = window[::stride]
        if window and points[-1] is not window[-1]:
            points.append(window[-1])
        sx = (x1 - x0) / self.window_s
        for s in shown:
            top = left_max if s["axis"] == "left" else self.right_max
            sy = (y1 - y0) / top
            idx = s["index"]
            coords = []
            for smp in points:
                coords.append(x0 + (smp[0] - t_start) * sx)
                coords.append(y1 - clamp(smp[idx], 0, top) * sy)
            if len(coords) >= 4:
                self.create_line(*coords, fill=s["color"], width=s.get("width", 1.5))

        # legend with the latest values
        x = x0
        last = window[-1] if window else None
        for s in shown:
            value = "" if last is None else f" {last[s['index']]:.{s.get('digits', 1)}f}"
            item = self.create_text(x, 4, text=f"\u25A0 {s['name']}{value}", anchor="nw",
                                    fill=s["color"], font=("", 9, "bold"))
            x = self.bbox(item)[2] + 14

        self._view = (x0, y0, x1, y1, t_start, t_end, i0, i1)
        self._draw_cursor()

    def _on_motion(self, event):
        self._mouse = (event.x, event.y)
        self._draw_cursor()

    def _on_leave(self, _event):
        self._mouse = None
        self.delete("cursor")

    def _draw_cursor(self):
        """Vertical cursor showing the values under the mouse (also works while paused)."""
        self.delete("cursor")
        if not (self._mouse and self._view):
            return
        x0, y0, x1, y1, t_start, t_end, i0, i1 = self._view
        mx, my = self._mouse
        if not (x0 <= mx <= x1 and y0 <= my <= y1) or i1 <= i0:
            return
        t = t_start + (mx - x0) / (x1 - x0) * self.window_s
        i = bisect.bisect_left(self.times, t, i0, i1)
        if i > i0 and (i == i1 or t - self.times[i - 1] < self.times[i] - t):
            i -= 1
        smp = self.samples[i]
        cx = x0 + (smp[0] - t_start) / self.window_s * (x1 - x0)
        self.create_line(cx, y0, cx, y1, fill="#616161", dash=(3, 3), tags="cursor")
        lines = [f"t = {smp[0] - t_end:+.2f} s"]
        lines += [f"{s['name']} : {smp[s['index']]:.{s.get('digits', 1)}f} {s.get('unit', '')}"
                  for s, v in zip(self.series, self.visible) if v.get()]
        anchor, tx = ("nw", cx + 8) if cx < (x0 + x1) / 2 else ("ne", cx - 8)
        text = self.create_text(tx, y0 + 8, text="\n".join(lines), anchor=anchor, fill=self.TEXT,
                                font=("Consolas", 9), tags="cursor")
        bx = self.bbox(text)
        box = self.create_rectangle(bx[0] - 4, bx[1] - 3, bx[2] + 4, bx[3] + 3, fill="#ffffff",
                                    outline="#bdbdbd", tags="cursor")
        self.tag_lower(box, text)


# ============================================================================ Device tab base
class DeviceTab(ttk.Frame):
    """A notebook tab driving one Arduino over its own serial connection.

    Handles connect / disconnect, the READY / identification handshake and wrong-firmware
    detection. Subclasses implement the hooks at the bottom.
    """
    FIRMWARE_ID = ""

    def __init__(self, notebook, app):
        super().__init__(notebook, padding=8)
        self.app = app
        self.ready = False
        self.identity = None
        self._handshake = 0
        self._closing = False
        self.log_view = None
        self.link = SerialLink(app.post, self._on_lines, self._on_serial_error)
        self.conn_bar = PortSelector(self, app.settings.get(self._port_key), self.toggle_connection)

    @property
    def _port_key(self):
        return f"port_{self.FIRMWARE_ID.lower()}"

    # ----- connection
    def toggle_connection(self):
        if self.link.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.conn_bar.port
        if not port:
            messagebox.showwarning("Aucun port", "Sélectionnez d'abord un port série.")
            return
        if not self.app.release_port(port, exclude=self):
            return
        try:
            self.link.open(port)
        except serial.SerialException as e:
            messagebox.showerror("Erreur de connexion", str(e))
            return
        self.ready = False
        self.identity = None
        self.conn_bar.set_connected(True)
        self.conn_bar.set_status("Attente de l'Arduino...", COLOR_WARN)
        self.log(f"Port {port} ouvert")
        self.app.settings.set(self._port_key, port)
        # The Uno resets when the port opens: wait for READY, then ask "?", then give up waiting
        self._handshake += 1
        hid = self._handshake
        self.after(READY_TIMEOUT_MS, lambda: self._ask_identity(hid))

    def _handshake_pending(self, hid):
        return hid == self._handshake and self.link.is_open and not self.ready and not self.identity

    def _ask_identity(self, hid):
        if self._handshake_pending(hid):
            self._write("?")
            self.after(IDENTIFY_TIMEOUT_MS, lambda: self._identify_timeout(hid))

    def _identify_timeout(self, hid):
        if self._handshake_pending(hid):
            self.log("Pas de READY reçu, on continue quand même")
            self._became_ready(None)

    def disconnect(self, reason="Déconnecté", graceful=True):
        if not self.link.is_open:
            return
        self._closing = True
        try:
            if graceful and self.ready:
                self.before_disconnect()
            self.link.close()
        finally:
            self._closing = False
        self._handshake += 1
        self.ready = False
        self.identity = None
        self.conn_bar.set_connected(False)
        self.conn_bar.set_status("Déconnecté")
        self.log(reason)
        self.on_disconnected()

    def _on_serial_error(self, message):
        self.log(f"Erreur série : {message}")
        self.disconnect("Connexion perdue", graceful=False)

    def _on_lines(self, lines):
        for line in lines:
            if not line:
                continue
            if line.startswith("READY") or (line.startswith("ID;") and not self.ready):
                parts = line.split(";")
                self._became_ready(parts[1] if len(parts) > 1 else None)
            elif line.startswith("ID;"):
                continue
            else:
                self.handle_line(line)

    def _became_ready(self, identity):
        self.identity = identity
        if identity and identity != self.FIRMWARE_ID:
            self.ready = False
            self.conn_bar.set_status(f"Mauvais firmware : {identity}", COLOR_ERROR)
            self.log(f"Firmware détecté : {identity} (attendu : {self.FIRMWARE_ID})")
            self.on_disconnected()
            self.after_idle(self._offer_firmware_upload, identity)
            return
        self.ready = True
        self.conn_bar.set_status(f"Connecté ({self.link.port})", COLOR_OK)
        self.log("Arduino prêt" + (f" (firmware {identity})" if identity else ""))
        self.on_ready()

    def _offer_firmware_upload(self, identity):
        port = self.link.port
        if port and messagebox.askyesno(
                "Mauvais firmware",
                f"L'Arduino sur {port} exécute le firmware « {identity} » au lieu de "
                f"« {self.FIRMWARE_ID} ».\n\nOuvrir l'onglet Firmware pour téléverser le bon programme ?"):
            self.app.show_firmware_tab(self.FIRMWARE_ID, port)

    # ----- I/O
    def _write(self, cmd):
        try:
            self.link.write_line(cmd)
            return True
        except (serial.SerialException, OSError) as e:
            self.log(f"Erreur d'écriture : {e}")
            if not self._closing:
                self.disconnect("Connexion perdue", graceful=False)
            return False

    def send_line(self, cmd, log=True):
        if not (self.link.is_open and self.ready):
            return False
        if not self._write(cmd):
            return False
        if log:
            self.log(f"> {cmd}")
        return True

    def log(self, text):
        if self.log_view:
            self.log_view.write(text)

    def save_settings(self):
        if self.conn_bar.port:
            self.app.settings.set(self._port_key, self.conn_bar.port)

    # ----- hooks
    def on_ready(self):
        pass

    def on_disconnected(self):
        pass

    def before_disconnect(self):
        pass

    def handle_line(self, line):
        self.log(f"< {line}")

    def emergency_stop(self):
        pass


# ============================================================================ Winder tab
class MotorPanel(ttk.LabelFrame):
    """Manual controls for one motor: speed, direction, start/stop."""

    def __init__(self, master, title, on_change):
        super().__init__(master, text=title, padding=10)
        self.on_change = on_change
        self.running = False
        self._debounce = Debouncer(self, SLIDER_DELAY_MS, self._changed)

        self.speed = tk.IntVar(value=DEFAULT_RPM)
        self.direction = tk.IntVar(value=1)
        self.status = tk.StringVar()

        ttk.Label(self, text="Vitesse (tr/min)").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Scale(self, from_=0, to=MAX_RPM, variable=self.speed, length=220,
                  command=self._on_slider).grid(row=1, column=0, columnspan=2, sticky="ew")
        spin = ttk.Spinbox(self, from_=0, to=MAX_RPM, increment=1, width=6,
                           textvariable=self.speed, command=self._changed)
        spin.grid(row=1, column=2, padx=(8, 0))
        spin.bind("<Return>", lambda e: self._changed())
        spin.bind("<FocusOut>", lambda e: self._changed())

        ttk.Radiobutton(self, text="Avant", value=1, variable=self.direction,
                        command=self._changed).grid(row=2, column=0, sticky="w", pady=8)
        ttk.Radiobutton(self, text="Arrière", value=-1, variable=self.direction,
                        command=self._changed).grid(row=2, column=1, sticky="w", pady=8)

        ttk.Button(self, text="Marche", command=lambda: self.set_running(True, notify=True)
                   ).grid(row=3, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(self, text="Arrêt", command=lambda: self.set_running(False, notify=True)
                   ).grid(row=3, column=1, sticky="ew", padx=(4, 0))

        ttk.Label(self, textvariable=self.status, font=("", 10, "bold")
                  ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self._update_status()

    def get_speed(self):
        try:
            value = int(self.speed.get())
        except (tk.TclError, ValueError):
            value = 0
        return clamp(value, 0, MAX_RPM)

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
        self._debounce.trigger()

    def _changed(self):
        self.speed.set(self.get_speed())   # clamp / clean typed value
        self._update_status()
        self.on_change()

    def _update_status(self):
        if self.running:
            way = "avant" if self.direction.get() > 0 else "arrière"
            self.status.set(f"EN MARCHE - {self.get_speed()} tr/min {way}")
        else:
            self.status.set("ARRÊTÉ")


class WindingPanel(ttk.LabelFrame):
    """Winding parameters, presets, computed info, start/pause/resume/abort, progress and ETA."""

    FIELDS = [
        ("core_d", "Diamètre du mandrin (mm)", "29"),
        ("length", "Longueur de la bobine (mm)", "11.5"),
        ("wire_d", "Diamètre du fil (mm)", "0.53"),
        ("tpl", "Spires par couche", "20"),
        ("layers", "Nombre de couches", "1"),
        ("rpm", "Vitesse de bobinage (tr/min)", "60"),
    ]

    def __init__(self, master, settings, on_start, on_pause, on_resume, on_abort):
        super().__init__(master, text="Bobinage automatique", padding=10)
        self.settings = settings
        self.on_start = on_start
        self.total_turns = 0
        self.total_layers = 0
        self.vars = {}
        self.entries = []
        self._started_at = None
        self._history = deque()          # (time, turn) over the last seconds, for the ETA

        # Presets
        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        ttk.Label(bar, text="Préréglage :").pack(side="left")
        self.preset_var = tk.StringVar()
        self.preset_box = ttk.Combobox(bar, textvariable=self.preset_var, width=24, state="readonly")
        self.preset_box.pack(side="left", padx=5)
        self.preset_box.bind("<<ComboboxSelected>>", lambda e: self._load_preset())
        save_btn = ttk.Button(bar, text="Enregistrer...", command=self._save_preset)
        save_btn.pack(side="left", padx=2)
        del_btn = ttk.Button(bar, text="Supprimer", command=self._delete_preset)
        del_btn.pack(side="left", padx=2)
        self.entries += [self.preset_box, save_btn, del_btn]

        saved = settings.get("winding_params", {})
        for row, (key, label, default) in enumerate(self.FIELDS, start=1):
            ttk.Label(self, text=label).grid(row=row, column=0, sticky="w")
            var = tk.StringVar(value=str(saved.get(key, default)))
            var.trace_add("write", lambda *_: self.update_info())
            entry = ttk.Entry(self, textvariable=var, width=8)
            entry.grid(row=row, column=1, sticky="w", padx=5, pady=1)
            self.vars[key] = var
            self.entries.append(entry)

        r = len(self.FIELDS) + 1
        self.direction = tk.IntVar(value=saved.get("direction", 1))
        dir_frame = ttk.Frame(self)
        dir_frame.grid(row=r, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Label(dir_frame, text="Sens du chariot (1re couche) :").pack(side="left")
        for text, value in (("Avant", 1), ("Arrière", -1)):
            rb = ttk.Radiobutton(dir_frame, text=text, value=value, variable=self.direction)
            rb.pack(side="left", padx=3)
            self.entries.append(rb)

        self.info = tk.StringVar()
        ttk.Label(self, textvariable=self.info, justify="left"
                  ).grid(row=1, column=2, rowspan=r, sticky="nw", padx=(25, 0))

        buttons = ttk.Frame(self)
        buttons.grid(row=r + 1, column=0, columnspan=3, sticky="w", pady=(8, 4))
        self.start_btn = ttk.Button(buttons, text="Démarrer le bobinage", command=self._start)
        self.pause_btn = ttk.Button(buttons, text="Pause", command=on_pause)
        self.resume_btn = ttk.Button(buttons, text="Reprendre", command=on_resume)
        self.abort_btn = ttk.Button(buttons, text="Annuler", command=on_abort)
        for b in (self.start_btn, self.pause_btn, self.resume_btn, self.abort_btn):
            b.pack(side="left", padx=3)

        self.progress = ttk.Progressbar(self, length=500, mode="determinate")
        self.progress.grid(row=r + 2, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        self.progress_text = tk.StringVar(value="Inactif")
        ttk.Label(self, textvariable=self.progress_text
                  ).grid(row=r + 3, column=0, columnspan=3, sticky="w")
        self.columnconfigure(2, weight=1)

        self._refresh_presets()
        self.set_active(False)
        self.update_info()

    # ----- parameters
    def _num(self, key, cast=float):
        return parse_number(self.vars[key].get(), cast)

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
            raise ValueError("Valeur invalide : vérifiez tous les champs.") from None
        if p["core_d"] <= 0 or p["length"] <= 0:
            raise ValueError("Les dimensions de la bobine doivent être > 0.")
        if not 0.05 <= p["wire_d"] <= 3:
            raise ValueError("Diamètre du fil : 0,05 à 3 mm.")
        if not 1 <= p["tpl"] <= 1000:
            raise ValueError("Spires par couche : 1 à 1000.")
        if not 1 <= p["layers"] <= 200:
            raise ValueError("Couches : 1 à 200.")
        if not 0 < p["rpm"] <= MAX_RPM:
            raise ValueError(f"Vitesse : 0 à {MAX_RPM} tr/min.")
        p["direction"] = self.direction.get()
        return p

    def get_values(self):
        values = {key: var.get() for key, var in self.vars.items()}
        values["direction"] = self.direction.get()
        return values

    def set_values(self, values):
        for key, var in self.vars.items():
            if key in values:
                var.set(str(values[key]))
        if values.get("direction") in (1, -1):
            self.direction.set(values["direction"])

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
            f"Largeur de couche : {width:.2f} mm (max {max_tpl} spires/couche)",
            f"Spires totales : {total}",
            f"Diamètre extérieur : ~{outer:.1f} mm",
            f"Longueur de fil : ~{wire_m:.1f} m",
            f"Durée : ~{format_duration(minutes * 60)}",
            f"Vitesse du chariot : {carriage_speed:.2f} mm/s",
        ]
        if width > p["length"] + 1e-9:
            lines.append("ATTENTION : couche plus large que la bobine !")
        if abs(steps - round(steps)) > 1e-6:
            lines.append(f"Note : pas arrondi à {round(steps) / CARRIAGE_STEPS_PER_MM:.4f} mm")
        self.info.set("\n".join(lines))

    # ----- presets
    def _presets(self):
        return self.settings.get("winding_presets", {})

    def _refresh_presets(self):
        self.preset_box["values"] = sorted(self._presets(), key=str.lower)

    def _load_preset(self):
        values = self._presets().get(self.preset_var.get())
        if values:
            self.set_values(values)

    def _save_preset(self):
        name = simpledialog.askstring("Enregistrer le préréglage", "Nom du préréglage :",
                                      initialvalue=self.preset_var.get(), parent=self)
        if not name or not name.strip():
            return
        name = name.strip()
        presets = dict(self._presets())
        if name in presets and not messagebox.askyesno("Préréglage existant",
                                                       f"Remplacer « {name} » ?", parent=self):
            return
        presets[name] = self.get_values()
        self.settings.set("winding_presets", presets)
        self.settings.save()
        self._refresh_presets()
        self.preset_var.set(name)

    def _delete_preset(self):
        name = self.preset_var.get()
        presets = dict(self._presets())
        if name in presets and messagebox.askyesno("Supprimer", f"Supprimer « {name} » ?", parent=self):
            del presets[name]
            self.settings.set("winding_presets", presets)
            self.settings.save()
            self.preset_var.set("")
            self._refresh_presets()

    # ----- run
    def _start(self):
        try:
            p = self.read_params()
        except ValueError as e:
            messagebox.showerror("Paramètres invalides", str(e))
            return
        if p["tpl"] * p["wire_d"] > p["length"] + 1e-9:
            if not messagebox.askyesno("Couche trop large",
                                       "La couche est plus large que la bobine. Démarrer quand même ?"):
                return
        self.on_start(p)

    def prepare(self, total_turns, total_layers):
        self.total_turns = total_turns
        self.total_layers = total_layers
        self.progress["maximum"] = total_turns
        self.progress["value"] = 0
        self.progress_text.set("Démarrage...")
        self._started_at = time.monotonic()
        self._history.clear()

    def reset_rate(self):
        """After a pause the speed estimate must start again."""
        self._history.clear()

    def set_progress(self, turn, layer):
        now = time.monotonic()
        self.progress["value"] = turn
        self._history.append((now, turn))
        while len(self._history) > 2 and now - self._history[0][0] > 10:
            self._history.popleft()
        text = (f"Spire {turn} / {self.total_turns}  -  Couche {layer} / {self.total_layers}")
        if self._started_at is not None:
            text += f"  -  écoulé {format_duration(now - self._started_at)}"
        if len(self._history) >= 2:
            (ta, na), (tb, nb) = self._history[0], self._history[-1]
            if tb > ta and nb > na:
                text += f"  -  reste ~{format_duration((self.total_turns - turn) * (tb - ta) / (nb - na))}"
        self.progress_text.set(text)

    def set_text(self, text):
        self.progress_text.set(text)

    def set_active(self, active):
        for w in self.entries:
            set_widget_state(w, not active)
        set_widget_state(self.start_btn, not active)
        for b in (self.pause_btn, self.resume_btn, self.abort_btn):
            set_widget_state(b, active)


class WinderTab(DeviceTab):
    FIRMWARE_ID = "WINDER"

    def __init__(self, notebook, app):
        super().__init__(notebook, app)
        self.winding = False
        self.last_cmd = None

        self.conn_bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        self.motor1 = MotorPanel(self, "Moteur 1 - Bobine (manuel)", self.send_state)
        self.motor1.grid(row=1, column=0, padx=(0, 5), pady=5, sticky="nsew")
        self.motor2 = MotorPanel(self, "Moteur 2 - Chariot (manuel)", self.send_state)
        self.motor2.grid(row=1, column=1, padx=(5, 0), pady=5, sticky="nsew")

        self.winding_panel = WindingPanel(self, app.settings, self.start_winding,
                                          lambda: self.send_line("P"),
                                          lambda: self.send_line("R"),
                                          lambda: self.send_line("X"))
        self.winding_panel.grid(row=2, column=0, columnspan=2, pady=5, sticky="ew")

        self.log_view = LogView(self, height=8)
        self.log_view.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(5, 0))
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(3, weight=1)

    # ----- hooks
    def on_ready(self):
        # Arduino (re)started: everything is stopped, so sync the GUI
        self.last_cmd = None
        self.set_winding(False)
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.winding_panel.set_text("Inactif")

    def on_disconnected(self):
        self.set_winding(False)
        self.motor1.set_running(False)
        self.motor2.set_running(False)

    def before_disconnect(self):
        self.send_line("X")            # abort winding / stop motors before leaving
        try:
            self.link.ser.flush()
        except (serial.SerialException, OSError):
            pass

    def emergency_stop(self):
        self.stop_all()

    def save_settings(self):
        super().save_settings()
        self.app.settings.set("winding_params", self.winding_panel.get_values())

    def handle_line(self, line):
        panel = self.winding_panel
        if line == "OK":
            return
        if line == "WINDING":
            self.set_winding(True)
            self.log("Bobinage démarré")
        elif line.startswith("T;"):
            try:
                _, turn, layer = line.split(";")
                panel.set_progress(int(turn), int(layer))
            except ValueError:
                self.log(f"< {line}")
        elif line == "DONE":
            self.set_winding(False)
            panel.set_text("Terminé - les moteurs maintiennent le fil (Arrêt pour relâcher)")
            self.log("Bobinage terminé")
            self.bell()
        elif line == "STOPPED":
            self.set_winding(False)
            panel.set_text("Annulé - les moteurs maintiennent le fil (Arrêt pour relâcher)")
            self.log("Bobinage annulé")
        elif line in ("PAUSED", "RESUMED", "ABORTING"):
            text = {"PAUSED": "En pause", "RESUMED": "Repris", "ABORTING": "Annulation..."}[line]
            panel.reset_rate()
            panel.set_text(text)
            self.log(text)
        else:
            self.log(f"< {line}")

    # ----- commands
    def send_state(self):
        """Manual mode: sends both motor speeds."""
        if self.winding:
            return
        cmd = f"S;{self.motor1.signed_speed()};{self.motor2.signed_speed()}"
        if cmd != self.last_cmd and self.send_line(cmd):
            self.last_cmd = cmd

    def start_winding(self, p):
        if not (self.link.is_open and self.ready):
            messagebox.showwarning("Non connecté", "Connectez d'abord l'Arduino de la bobineuse.")
            return
        if self.motor1.running or self.motor2.running:
            messagebox.showwarning("Moteurs en marche", "Arrêtez les deux moteurs avant de bobiner.")
            return
        self.winding_panel.prepare(p["tpl"] * p["layers"], p["layers"])
        self.send_line(f"W;{p['wire_d']:g};{p['tpl']};{p['layers']};{p['rpm']:g};{p['direction']}")

    def set_winding(self, active):
        self.winding = active
        self.last_cmd = None
        self.winding_panel.set_active(active)
        self.motor1.set_enabled(not active)
        self.motor2.set_enabled(not active)

    def stop_all(self):
        if self.winding:
            self.send_line("X")
            return
        self.motor1.set_running(False)
        self.motor2.set_running(False)
        self.last_cmd = None
        self.send_state()


# ============================================================================ Current tab
class CurrentTab(DeviceTab):
    FIRMWARE_ID = "CURRENT"
    MODE_OFF, MODE_REGULATION, MODE_OPEN_LOOP = 0, 1, 2

    # sample tuple: (t, setpoint, raw, filtered, pwm, voltage)
    SERIES = [
        {"name": "Consigne", "color": "#2e7d32", "axis": "left", "index": 1, "unit": "mA", "width": 2},
        {"name": "Courant", "color": "#90caf9", "axis": "left", "index": 2, "unit": "mA", "width": 1},
        {"name": "Filtré", "color": "#1565c0", "axis": "left", "index": 3, "unit": "mA", "width": 2},
        {"name": "PWM", "color": "#ef6c00", "axis": "right", "index": 4, "unit": "", "digits": 0},
    ]
    WINDOWS = {"10 s": 10, "30 s": 30, "1 min": 60, "2 min": 120, "5 min": 300, "10 min": 600}
    PID_FIELDS = [   # key, label, default
        ("kp", "Kp (pts PWM / mA)", "0"),
        ("ki", "Ki (pts / mA·s)", "0.25"),
        ("kd", "Kd (pts·s / mA)", "0"),
        ("period", "Période (ms)", "200"),
        ("alpha", "Filtre α (0,01-1)", "0.1"),
        ("limit", "Limite (mA, 0 = off)", "1000"),
    ]

    def __init__(self, notebook, app):
        super().__init__(notebook, app)
        self.mode = tk.IntVar(value=self.MODE_OFF)
        self.setpoint = tk.DoubleVar(value=40.0)
        self.pwm = tk.IntVar(value=0)
        self.fault = None
        self._last_output_cmd = None
        self._sensor_fault = False
        self._fault_dialog_open = False
        self._t0 = None
        self._wall0 = None
        self._latest = None
        self._new_data = False
        self._rates = deque(maxlen=30)
        self._rec_file = None
        self._rec_writer = None
        self._rec_last_flush = 0.0
        self._sp_debounce = Debouncer(self, SLIDER_DELAY_MS, self._setpoint_changed)
        self._pwm_debounce = Debouncer(self, SLIDER_DELAY_MS, self._pwm_changed)

        self.conn_bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        left = ttk.Frame(self)
        left.grid(row=1, column=0, sticky="nsw", padx=(0, 8))
        right = ttk.Frame(self)
        right.grid(row=1, column=1, sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

        self._build_output(left).pack(fill="x")
        self._build_pid(left).pack(fill="x", pady=8)
        self._build_measures(left).pack(fill="x")
        self._build_chart(right)

        self._update_output_status()
        self.after(250, self._refresh_readouts)

    # ----- layout
    def _build_output(self, parent):
        f = ttk.LabelFrame(parent, text="Sortie", padding=8)
        modes = ttk.Frame(f)
        modes.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        for text, value in (("Arrêt", self.MODE_OFF), ("Régulation", self.MODE_REGULATION),
                            ("PWM manuel", self.MODE_OPEN_LOOP)):
            ttk.Radiobutton(modes, text=text, value=value, variable=self.mode,
                            command=self._mode_changed).pack(side="left", padx=(0, 10))

        ttk.Label(f, text="Consigne (mA)").grid(row=1, column=0, sticky="w")
        ttk.Scale(f, from_=0, to=CURRENT_MAX_MA, variable=self.setpoint, length=190,
                  command=lambda v: (self.setpoint.set(round(float(v))), self._sp_debounce.trigger())
                  ).grid(row=1, column=1, sticky="ew", padx=5)
        self.sp_spin = ttk.Spinbox(f, from_=0, to=CURRENT_MAX_MA, increment=1, width=7,
                                   textvariable=self.setpoint, command=self._setpoint_changed)
        self.sp_spin.grid(row=1, column=2)
        self.sp_spin.bind("<Return>", lambda e: self._setpoint_changed())
        self.sp_spin.bind("<FocusOut>", lambda e: self._setpoint_changed())

        ttk.Label(f, text="PWM manuel").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Scale(f, from_=0, to=PWM_MAX, variable=self.pwm, length=190,
                  command=lambda v: (self.pwm.set(round(float(v))), self._pwm_debounce.trigger())
                  ).grid(row=2, column=1, sticky="ew", padx=5, pady=(4, 0))
        self.pwm_spin = ttk.Spinbox(f, from_=0, to=PWM_MAX, increment=1, width=7,
                                    textvariable=self.pwm, command=self._pwm_changed)
        self.pwm_spin.grid(row=2, column=2, pady=(4, 0))
        self.pwm_spin.bind("<Return>", lambda e: self._pwm_changed())
        self.pwm_spin.bind("<FocusOut>", lambda e: self._pwm_changed())

        tk.Button(f, text="COUPER LA SORTIE", bg=COLOR_ERROR, fg="white", activebackground="#8e0000",
                  activeforeground="white", font=("", 10, "bold"), command=self.output_off
                  ).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 4))
        self.out_status = ttk.Label(f, font=("", 10, "bold"))
        self.out_status.grid(row=4, column=0, columnspan=3, sticky="w")
        f.columnconfigure(1, weight=1)
        return f

    def _build_pid(self, parent):
        f = ttk.LabelFrame(parent, text="Régulateur PID", padding=8)
        self.pid_vars = {}
        for i, (key, label, default) in enumerate(self.PID_FIELDS):
            row, col = i % 3, (i // 3) * 2
            ttk.Label(f, text=label).grid(row=row, column=col, sticky="w", padx=(0 if col == 0 else 12, 4))
            var = tk.StringVar(value=default)
            entry = ttk.Entry(f, textvariable=var, width=8)
            entry.grid(row=row, column=col + 1, sticky="w", pady=1)
            entry.bind("<Return>", lambda e: self.send_pid())
            self.pid_vars[key] = var
        self.fast_pwm = tk.BooleanVar(value=False)
        self.boot_regulation = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="PWM rapide (31 kHz)", variable=self.fast_pwm
                        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Checkbutton(f, text="Régulation au démarrage", variable=self.boot_regulation
                        ).grid(row=3, column=2, columnspan=2, sticky="w", padx=(12, 0), pady=(4, 0))
        buttons = ttk.Frame(f)
        buttons.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        for text, command in (("Envoyer", self.send_pid), ("Lire", lambda: self.send_line("G")),
                              ("Sauver EEPROM", self.save_eeprom), ("Défauts", self.restore_defaults),
                              ("Reset intégrale", lambda: self.send_line("Z"))):
            ttk.Button(buttons, text=text, command=command).pack(side="left", padx=(0, 4))
        ttk.Label(f, text="Entrée = envoyer. Sans « Sauver EEPROM », perdu au reset.",
                  foreground="#757575", font=("", 8)
                  ).grid(row=5, column=0, columnspan=4, sticky="w", pady=(6, 0))
        return f

    def _build_measures(self, parent):
        f = ttk.LabelFrame(parent, text="Mesures", padding=8)
        self.big_value = tk.StringVar(value="-- mA")
        ttk.Label(f, text="Courant filtré").grid(row=0, column=0, sticky="w")
        ttk.Label(f, textvariable=self.big_value, font=("", 22, "bold"), foreground="#1565c0"
                  ).grid(row=1, column=0, columnspan=2, sticky="w")
        self.readouts = {}
        rows = [("raw", "Courant brut"), ("sp", "Consigne"), ("err", "Erreur"), ("pwm", "PWM"),
                ("volt", "Tension bus"), ("res", "V / I"), ("mean", "Moyenne (fenêtre)"),
                ("std", "Ondulation σ (fenêtre)"), ("rate", "Échantillonnage")]
        for i, (key, label) in enumerate(rows, start=2):
            ttk.Label(f, text=label).grid(row=i, column=0, sticky="w")
            var = tk.StringVar(value="--")
            ttk.Label(f, textvariable=var, font=("Consolas", 10)).grid(row=i, column=1, sticky="e", padx=(20, 0))
            self.readouts[key] = var
        f.columnconfigure(1, weight=1)
        return f

    def _build_chart(self, parent):
        settings = self.app.settings
        bar = ttk.Frame(parent)
        bar.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(bar, text="Fenêtre :").pack(side="left")
        self.window_var = tk.StringVar(value=settings.get("chart_window", "30 s"))
        if self.window_var.get() not in self.WINDOWS:
            self.window_var.set("30 s")
        box = ttk.Combobox(bar, textvariable=self.window_var, values=list(self.WINDOWS), width=7,
                           state="readonly")
        box.pack(side="left", padx=(4, 12))
        box.bind("<<ComboboxSelected>>", lambda e: self.chart.set_window(self.WINDOWS[self.window_var.get()]))

        self.chart = StripChart(parent, self.SERIES, "mA", "PWM", PWM_MAX, width=720, height=400)
        self.chart.grid(row=1, column=0, sticky="nsew")
        self.chart.set_window(self.WINDOWS[self.window_var.get()])
        hidden = settings.get("chart_hidden", [])
        for s, var in zip(self.SERIES, self.chart.visible):
            var.set(s["name"] not in hidden)
            style = f"{s['name']}.TCheckbutton"
            ttk.Style(self).configure(style, foreground=s["color"])
            ttk.Checkbutton(bar, text=s["name"], variable=var, style=style).pack(side="left", padx=2)

        self.rec_btn = ttk.Button(bar, text="\u25CF Enregistrer...", command=self.toggle_recording)
        self.rec_btn.pack(side="right")
        ttk.Button(bar, text="Exporter CSV...", command=self.export_csv).pack(side="right", padx=4)
        ttk.Button(bar, text="Effacer", command=self.chart.clear).pack(side="right")
        self.pause_btn = ttk.Button(bar, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side="right", padx=4)

        self.log_view = LogView(parent, height=6)
        self.log_view.grid(row=2, column=0, sticky="nsew", pady=(6, 0))
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

    # ----- hooks
    def on_ready(self):
        self._last_output_cmd = None
        self.fault = None
        self._sensor_fault = False
        self.send_line("G")

    def on_disconnected(self):
        self.mode.set(self.MODE_OFF)
        self._update_output_status()
        self.stop_recording()

    def before_disconnect(self):
        self.send_line("X")            # safety: no current left flowing without supervision
        try:
            self.link.ser.flush()
        except (serial.SerialException, OSError):
            pass

    def emergency_stop(self):
        self.output_off()

    def save_settings(self):
        super().save_settings()
        self.app.settings.set("chart_window", self.window_var.get())
        self.app.settings.set("chart_hidden", [s["name"] for s, v in zip(self.SERIES, self.chart.visible)
                                               if not v.get()])

    def handle_line(self, line):
        if line.startswith("Consigne:"):
            self._on_telemetry(line)
        elif line == "OK":
            return
        elif line.startswith("P;"):
            self._apply_settings_report(line.split(";"))
        elif line.startswith("FAULT;OVERCURRENT"):
            value = line.split(";")[2] if line.count(";") >= 2 else "?"
            self.mode.set(self.MODE_OFF)
            self.fault = f"SURINTENSITÉ ({value} mA) - sortie coupée"
            self._update_output_status()
            self.log(f"Protection : surintensité {value} mA, sortie coupée")
            if not self._fault_dialog_open:
                self._fault_dialog_open = True
                self.after_idle(self._show_fault_dialog, value)
        elif line == "FAULT;SENSOR":
            if not self._sensor_fault:
                self._sensor_fault = True
                self.mode.set(self.MODE_OFF)
                self.fault = "Capteur INA219 introuvable (0x45) - vérifiez le câblage I2C"
                self._update_output_status()
                self.log(self.fault)
        elif line.startswith("ERR"):
            self.log(f"< {line}")
            self.send_line("G", log=False)       # re-sync the GUI with the real state
        else:
            self.log(f"< {line}")

    def _show_fault_dialog(self, value):
        messagebox.showwarning("Surintensité",
                               f"Courant mesuré {value} mA au-dessus de la limite : sortie coupée.")
        self._fault_dialog_open = False

    # ----- telemetry
    def _on_telemetry(self, line):
        values = {}
        for part in line.split(","):
            key, sep, value = part.partition(":")
            if not sep:
                return
            try:
                values[key.strip()] = float(value)
            except ValueError:
                return
        now = time.monotonic()
        if self._t0 is None:
            self._t0, self._wall0 = now, time.time()
        filtered = values.get("Filtre", values.get("Courant filtre", 0.0))
        sample = (now - self._t0, values.get("Consigne", 0.0), values.get("Courant", 0.0), filtered,
                  values.get("PWM", 0.0), values.get("Tension", float("nan")))
        self.chart.add(sample)
        self._latest = sample
        self._new_data = True
        self._rates.append(now)
        if self._sensor_fault:
            self._sensor_fault = False
            self.fault = None
            self._update_output_status()
        if self._rec_writer:
            self._rec_writer.writerow(self._csv_row(sample))
            if now - self._rec_last_flush > 1.0:
                self._rec_last_flush = now
                self._rec_file.flush()

    def _refresh_readouts(self):
        """Labels are refreshed at a few Hz only, whatever the telemetry rate."""
        if self._new_data and self._latest:
            self._new_data = False
            _, sp, raw, filt, pwm, volt = self._latest
            r = self.readouts
            self.big_value.set(f"{filt:.1f} mA")
            r["raw"].set(f"{raw:.1f} mA")
            regulating = self.mode.get() == self.MODE_REGULATION
            r["sp"].set(f"{sp:.1f} mA" if regulating else "--")
            r["err"].set(f"{sp - filt:+.1f} mA" if regulating else "--")
            r["pwm"].set(f"{pwm:.0f}  ({pwm / PWM_MAX * 100:.0f} %)")
            r["volt"].set("--" if math.isnan(volt) else f"{volt:.3f} V")
            r["res"].set(f"{volt / filt * 1000:.1f} Ω" if filt > 1 and not math.isnan(volt) else "--")
            values = self.chart.visible_values(2)
            if len(values) >= 2:
                mean = sum(values) / len(values)
                std = math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))
                r["mean"].set(f"{mean:.2f} mA")
                r["std"].set(f"{std:.2f} mA")
            if len(self._rates) >= 2 and self._rates[-1] > self._rates[0]:
                r["rate"].set(f"{(len(self._rates) - 1) / (self._rates[-1] - self._rates[0]):.1f} Hz")
        self.after(250, self._refresh_readouts)

    def _apply_settings_report(self, parts):
        try:
            mode, sp, pwm = int(parts[1]), float(parts[2]), int(parts[3])
            kp, ki, kd = (float(x) for x in parts[4:7])
            period, alpha, limit = int(parts[7]), float(parts[8]), float(parts[9])
            fast, boot = parts[10] == "1", parts[11] == "1"
        except (IndexError, ValueError):
            self.log(f"Réponse P invalide : {';'.join(parts)}")
            return
        self.mode.set(mode)
        self.setpoint.set(sp)
        self.pwm.set(pwm)
        for key, value in (("kp", kp), ("ki", ki), ("kd", kd), ("period", period),
                           ("alpha", alpha), ("limit", limit)):
            self.pid_vars[key].set(f"{value:g}")
        self.fast_pwm.set(fast)
        self.boot_regulation.set(boot)
        self._last_output_cmd = None
        self._update_output_status()

    # ----- output control
    def _get_setpoint(self):
        try:
            value = parse_number(self.sp_spin.get())
        except ValueError:
            value = 0.0
        value = round(clamp(value, 0, CURRENT_MAX_MA), 2)
        self.setpoint.set(value)
        return value

    def _get_pwm(self):
        try:
            value = parse_number(self.pwm_spin.get(), int)
        except ValueError:
            value = 0
        value = clamp(value, 0, PWM_MAX)
        self.pwm.set(value)
        return value

    def _send_output(self, cmd):
        if cmd != self._last_output_cmd and self.send_line(cmd):
            self._last_output_cmd = cmd

    def _mode_changed(self):
        if not (self.link.is_open and self.ready):
            self.mode.set(self.MODE_OFF)
            self._update_output_status()
            messagebox.showwarning("Non connecté", "Connectez d'abord l'Arduino du générateur de courant.")
            return
        self.fault = None
        mode = self.mode.get()
        if mode == self.MODE_REGULATION and self._get_setpoint() <= 0:
            self.mode.set(self.MODE_OFF)
            mode = self.MODE_OFF
        if mode == self.MODE_OFF:
            self._send_output("X")
        elif mode == self.MODE_REGULATION:
            self._send_output(f"C;{self._get_setpoint():g}")
        else:
            self._send_output(f"O;{self._get_pwm()}")
        self._update_output_status()

    def _setpoint_changed(self):
        sp = self._get_setpoint()
        if self.mode.get() == self.MODE_REGULATION:
            if sp <= 0:
                self.mode.set(self.MODE_OFF)
                self._send_output("X")
            else:
                self._send_output(f"C;{sp:g}")
        self._update_output_status()

    def _pwm_changed(self):
        pwm = self._get_pwm()
        if self.mode.get() == self.MODE_OPEN_LOOP:
            self._send_output(f"O;{pwm}")
        self._update_output_status()

    def output_off(self):
        self.mode.set(self.MODE_OFF)
        self._last_output_cmd = None
        self._send_output("X")
        self._update_output_status()

    def _update_output_status(self):
        mode = self.mode.get()
        if self.fault:
            text, color = self.fault, COLOR_ERROR
        elif mode == self.MODE_REGULATION:
            text, color = f"Régulation à {self.setpoint.get():g} mA", COLOR_OK
        elif mode == self.MODE_OPEN_LOOP:
            pwm = self.pwm.get()
            text, color = f"PWM manuel : {pwm} ({pwm / PWM_MAX * 100:.0f} %)", COLOR_WARN
        else:
            text, color = "Sortie coupée", COLOR_IDLE
        self.out_status.configure(text=text, foreground=color)

    # ----- PID
    def _read_pid_form(self):
        v = {}
        try:
            for key in ("kp", "ki", "kd", "alpha", "limit"):
                v[key] = parse_number(self.pid_vars[key].get())
            v["period"] = parse_number(self.pid_vars["period"].get(), int)
        except ValueError:
            raise ValueError("Valeur invalide : vérifiez tous les champs du PID.") from None
        if not all(0 <= v[k] <= 1000 for k in ("kp", "ki", "kd")):
            raise ValueError("Gains : 0 à 1000.")
        if not 10 <= v["period"] <= 2000:
            raise ValueError("Période : 10 à 2000 ms.")
        if not 0.01 <= v["alpha"] <= 1:
            raise ValueError("Filtre α : 0,01 à 1 (1 = pas de filtrage).")
        if v["limit"] < 0:
            raise ValueError("La limite doit être >= 0.")
        return v

    def _send_pid_form(self):
        if not (self.link.is_open and self.ready):
            messagebox.showwarning("Non connecté", "Connectez d'abord l'Arduino du générateur de courant.")
            return False
        try:
            v = self._read_pid_form()
        except ValueError as e:
            messagebox.showerror("Paramètres invalides", str(e))
            return False
        for cmd in (f"K;{v['kp']:g};{v['ki']:g};{v['kd']:g}", f"T;{v['period']}", f"A;{v['alpha']:g}",
                    f"L;{v['limit']:g}", f"F;{int(self.fast_pwm.get())}",
                    f"B;{int(self.boot_regulation.get())}"):
            if not self.send_line(cmd):
                return False
        return True

    def send_pid(self):
        if self._send_pid_form():
            self.send_line("G", log=False)

    def save_eeprom(self):
        if self._send_pid_form() and self.send_line("E"):
            self.send_line("G", log=False)
            self.log("Réglages sauvegardés dans l'EEPROM")

    def restore_defaults(self):
        if not (self.link.is_open and self.ready):
            return
        if messagebox.askyesno("Réglages par défaut",
                               "Recharger les réglages par défaut dans l'Arduino ?\n"
                               "(non sauvegardés tant que vous ne cliquez pas « Sauver EEPROM »)"):
            self.send_line("D")
            self.send_line("G", log=False)

    # ----- chart / recording
    def toggle_pause(self):
        paused = not self.chart.paused
        self.chart.set_paused(paused)
        self.pause_btn.configure(text="Reprendre" if paused else "Pause")

    def _csv_row(self, sample):
        t, sp, raw, filt, pwm, volt = sample
        stamp = datetime.fromtimestamp(self._wall0 + t).isoformat(timespec="milliseconds")
        return [stamp, f"{t:.3f}", f"{sp:.2f}", f"{raw:.2f}", f"{filt:.2f}", f"{pwm:.0f}",
                "" if math.isnan(volt) else f"{volt:.3f}"]

    CSV_HEADER = ["horodatage", "t_s", "consigne_mA", "courant_mA", "courant_filtre_mA", "pwm",
                  "tension_V"]

    def _ask_csv_path(self, prefix):
        return filedialog.asksaveasfilename(
            parent=self, defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.csv")

    def export_csv(self):
        if not self.chart.samples:
            messagebox.showinfo("Exporter", "Aucune donnée à exporter.")
            return
        path = self._ask_csv_path("courant")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f, delimiter=";")
                writer.writerow(self.CSV_HEADER)
                writer.writerows(self._csv_row(s) for s in self.chart.samples)
        except OSError as e:
            messagebox.showerror("Exporter", str(e))
            return
        self.log(f"{len(self.chart.samples)} points exportés dans {path}")

    def toggle_recording(self):
        if self._rec_file:
            self.stop_recording()
            return
        path = self._ask_csv_path("enregistrement")
        if not path:
            return
        try:
            self._rec_file = open(path, "w", newline="", encoding="utf-8")
        except OSError as e:
            messagebox.showerror("Enregistrement", str(e))
            return
        if self._t0 is None:
            self._t0, self._wall0 = time.monotonic(), time.time()
        self._rec_writer = csv.writer(self._rec_file, delimiter=";")
        self._rec_writer.writerow(self.CSV_HEADER)
        self.rec_btn.configure(text="\u25A0 Arrêter l'enregistrement")
        self.log(f"Enregistrement dans {path}")

    def stop_recording(self):
        if not self._rec_file:
            return
        try:
            self._rec_file.close()
        except OSError:
            pass
        self.log(f"Enregistrement terminé ({self._rec_file.name})")
        self._rec_file = None
        self._rec_writer = None
        self.rec_btn.configure(text="\u25CF Enregistrer...")


# ============================================================================ Firmware tab
class FirmwareTab(ttk.Frame):
    """Compiles (optional) and uploads a sketch, then reconnects the matching tab."""

    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app = app
        s = app.settings
        self.busy = False
        self.choice = tk.StringVar(value=s.get("fw_choice", "WINDER"))
        self.custom_path = tk.StringVar(value=s.get("fw_custom", ""))
        self.board = tk.StringVar(value=s.get("fw_board", "Arduino Uno"))
        if self.board.get() not in BOARDS:
            self.board.set("Arduino Uno")
        self.source = tk.StringVar(value="compile")
        self.method = tk.StringVar(value=s.get("fw_method", "Auto"))
        self.auto_connect = tk.BooleanVar(value=s.get("fw_autoconnect", True))
        self.cli_custom = s.get("arduino_cli")
        self.cli_path = find_arduino_cli(self.cli_custom)

        # --- what
        what = ttk.LabelFrame(self, text="Programme à téléverser", padding=8)
        what.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        for fw_id, fw in FIRMWARES.items():
            ttk.Radiobutton(what, text=fw["label"], value=fw_id, variable=self.choice,
                            command=self._update_options).pack(anchor="w")
        custom = ttk.Frame(what)
        custom.pack(anchor="w", fill="x", pady=(2, 0))
        ttk.Radiobutton(custom, text="Autre fichier (.ino / .hex) :", value="CUSTOM",
                        variable=self.choice, command=self._update_options).pack(side="left")
        ttk.Entry(custom, textvariable=self.custom_path, width=30).pack(side="left", padx=4, fill="x", expand=True)
        ttk.Button(custom, text="Parcourir...", command=self._browse_file).pack(side="left")
        self.files_info = ttk.Label(what, foreground="#616161", justify="left", font=("", 8))
        self.files_info.pack(anchor="w", pady=(8, 0))

        # --- where
        where = ttk.LabelFrame(self, text="Carte", padding=8)
        where.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        ttk.Label(where, text="Type :").grid(row=0, column=0, sticky="w")
        ttk.Combobox(where, textvariable=self.board, values=list(BOARDS), state="readonly", width=32
                     ).grid(row=0, column=1, sticky="w", pady=2)
        self.port_sel = PortSelector(where, s.get("fw_port"))
        self.port_sel.grid(row=1, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Checkbutton(where, text="Se connecter automatiquement après le téléversement",
                        variable=self.auto_connect).grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))

        # --- how
        how = ttk.LabelFrame(self, text="Compilation et téléversement", padding=8)
        how.grid(row=1, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Label(how, text="arduino-cli :").grid(row=0, column=0, sticky="w")
        self.cli_label = ttk.Label(how, font=("", 8))
        self.cli_label.grid(row=0, column=1, columnspan=3, sticky="w")
        ttk.Button(how, text="Choisir...", command=self._browse_cli).grid(row=0, column=4, padx=4)
        self.deps_btn = ttk.Button(how, text="Installer les dépendances", command=self.install_dependencies)
        self.deps_btn.grid(row=0, column=5)

        ttk.Label(how, text="Source :").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.rb_compile = ttk.Radiobutton(how, text="Compiler les sources (.ino)", value="compile",
                                          variable=self.source)
        self.rb_compile.grid(row=1, column=1, sticky="w", pady=(6, 0))
        self.rb_hex = ttk.Radiobutton(how, text="Binaire précompilé (.hex)", value="hex", variable=self.source)
        self.rb_hex.grid(row=1, column=2, sticky="w", padx=10, pady=(6, 0))

        ttk.Label(how, text="Téléverseur :").grid(row=2, column=0, sticky="w", pady=(4, 0))
        methods = ttk.Frame(how)
        methods.grid(row=2, column=1, columnspan=5, sticky="w", pady=(4, 0))
        for text, value in (("Auto", "Auto"), ("Intégré (aucune dépendance)", "Intégré"),
                            ("arduino-cli / avrdude", "arduino-cli")):
            ttk.Radiobutton(methods, text=text, value=value, variable=self.method).pack(side="left", padx=(0, 10))
        how.columnconfigure(3, weight=1)

        # --- go
        go = ttk.Frame(self)
        go.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.upload_btn = ttk.Button(go, text="Téléverser", command=self.upload, width=16)
        self.upload_btn.pack(side="left")
        self.progress = ttk.Progressbar(go, mode="determinate", maximum=1.0, length=300)
        self.progress.pack(side="left", padx=10, fill="x", expand=True)
        self.status = ttk.Label(go, text="Prêt", font=("", 9, "bold"))
        self.status.pack(side="left")

        self.log_view = LogView(self, height=14, timestamps=False)
        self.log_view.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(3, weight=1)

        self._update_options(initial=True)

    # ----- options
    def _job_files(self):
        """(sketch dir or None, hex path or None, firmware id or None) for the current choice."""
        choice = self.choice.get()
        if choice in FIRMWARES:
            sketch = FIRMWARES[choice]["sketch"]
            ino = find_resource(sketch, sketch + ".ino")
            return (os.path.dirname(ino) if ino else None,
                    find_resource("firmware", sketch + ".hex"), choice)
        path = self.custom_path.get().strip()
        if path.lower().endswith(".ino") and os.path.isfile(path):
            return os.path.dirname(path), None, None
        if path.lower().endswith(".hex") and os.path.isfile(path):
            return None, path, None
        return None, None, None

    def _update_options(self, initial=False):
        sketch_dir, hex_path, _ = self._job_files()
        can_compile = bool(sketch_dir and self.cli_path)
        set_widget_state(self.rb_compile, can_compile)
        set_widget_state(self.rb_hex, bool(hex_path))
        if initial or (self.source.get() == "compile" and not can_compile) or \
                (self.source.get() == "hex" and not hex_path):
            self.source.set("compile" if can_compile else "hex")
        self.cli_label.configure(text=self.cli_path or "introuvable (installez l'IDE Arduino ou choisissez "
                                                       "arduino-cli.exe) - seuls les .hex sont utilisables",
                                 foreground="#424242" if self.cli_path else COLOR_WARN)
        set_widget_state(self.deps_btn, bool(self.cli_path))
        self.files_info.configure(text=f"Sources : {sketch_dir or 'aucune'}\nBinaire : {hex_path or 'aucun'}")

    def _browse_file(self):
        path = filedialog.askopenfilename(parent=self, filetypes=[("Arduino", "*.ino *.hex"),
                                                                  ("Tous", "*.*")])
        if path:
            self.custom_path.set(path)
            self.choice.set("CUSTOM")
            self._update_options()

    def _browse_cli(self):
        path = filedialog.askopenfilename(parent=self, title="arduino-cli",
                                          filetypes=[("arduino-cli", "arduino-cli*"), ("Tous", "*.*")])
        if path:
            self.cli_custom = path
            self.cli_path = find_arduino_cli(path)
            self._update_options()

    def preselect(self, firmware_id, port):
        if firmware_id in FIRMWARES:
            self.choice.set(firmware_id)
        if port:
            self.port_sel.set_port(port)
        self._update_options()

    def save_settings(self):
        s = self.app.settings
        s.set("fw_choice", self.choice.get())
        s.set("fw_custom", self.custom_path.get())
        s.set("fw_board", self.board.get())
        s.set("fw_method", self.method.get())
        s.set("fw_autoconnect", self.auto_connect.get())
        s.set("fw_port", self.port_sel.port)
        if self.cli_custom:
            s.set("arduino_cli", self.cli_custom)

    # ----- thread plumbing
    def _log(self, text):                       # callable from any thread
        self.app.post(self.log_view.write, text)

    def _progress(self, fraction):              # callable from any thread
        self.app.post(self._set_progress, fraction)

    def _status(self, text):                    # callable from any thread
        self.app.post(self._set_status, text)

    def _set_progress(self, fraction):
        self.progress.configure(value=fraction)

    def _set_status(self, text, color="#424242"):
        self.status.configure(text=text, foreground=color)

    def _set_busy(self, busy, status=None, color="#424242"):
        self.busy = busy
        set_widget_state(self.upload_btn, not busy)
        set_widget_state(self.deps_btn, not busy and bool(self.cli_path))
        if busy:
            self.progress.configure(mode="indeterminate")
            self.progress.start(15)
        else:
            self.progress.stop()
            self.progress.configure(mode="determinate")
        if status:
            self._set_status(status, color)

    # ----- actions
    def upload(self):
        if self.busy:
            return
        self._update_options()
        sketch_dir, hex_path, fw_id = self._job_files()
        compile_it = self.source.get() == "compile"
        if compile_it and not (sketch_dir and self.cli_path):
            messagebox.showerror("Téléverser", "Compilation impossible : sources ou arduino-cli introuvables.")
            return
        if not compile_it and not hex_path:
            messagebox.showerror("Téléverser", "Aucun fichier .hex disponible pour ce choix.")
            return
        port = self.port_sel.port
        if not port:
            messagebox.showwarning("Aucun port", "Sélectionnez le port de l'Arduino.")
            return
        method = self.method.get()
        if method == "Auto":
            method = "arduino-cli" if self.cli_path else "Intégré"
        if method == "arduino-cli" and not self.cli_path:
            messagebox.showerror("Téléverser", "arduino-cli introuvable : choisissez le téléverseur intégré.")
            return
        if not self.app.release_port(port):
            return

        board = BOARDS[self.board.get()]
        job = {"sketch_dir": sketch_dir if compile_it else None, "hex": hex_path, "fw_id": fw_id,
               "port": port, "board": board, "method": method}
        self.log_view.clear()
        self._set_busy(True, "Préparation...")
        threading.Thread(target=self._upload_worker, args=(job,), daemon=True).start()

    def _upload_worker(self, job):
        build_dir = None
        try:
            board = job["board"]
            cli = ArduinoCli(self.cli_path, self._log) if self.cli_path else None
            hex_path = job["hex"]
            if job["sketch_dir"]:
                name = os.path.basename(job["sketch_dir"])
                if not os.path.isfile(os.path.join(job["sketch_dir"], name + ".ino")):
                    raise UploadError("Le fichier .ino doit être dans un dossier du même nom "
                                      "(ex. mon_sketch/mon_sketch.ino).")
                self._status("Compilation...")
                build_dir = tempfile.mkdtemp(prefix="coilwinder_build_")
                hex_path = cli.compile(job["sketch_dir"], board["fqbn"], build_dir)
                self._refresh_precompiled(job, hex_path)

            with open(hex_path, encoding="ascii", errors="replace") as f:
                image = parse_intel_hex(f.read())
            self._log(f"Programme : {len(image)} octets (max {board['max_size']})")
            if len(image) > board["max_size"]:
                raise UploadError("Programme trop gros pour cette carte")

            self._status(f"Téléversement sur {job['port']}...")
            if job["method"] == "arduino-cli":
                cli.upload(hex_path, job["port"], board["fqbn"])
            else:
                self.app.post(self._determinate)
                Stk500Uploader(job["port"], board["baud"], self._log, self._progress).upload(image)
            self.app.post(self._upload_done, True, "Téléversement réussi", job)
        except (UploadError, serial.SerialException, OSError) as e:
            self._log(f"ERREUR : {e}")
            self.app.post(self._upload_done, False, "Échec du téléversement", job)
        finally:
            if build_dir:
                shutil.rmtree(build_dir, ignore_errors=True)

    def _refresh_precompiled(self, job, hex_path):
        """Running from the sources: keep firmware/<sketch>.hex in sync for the portable exe."""
        if FROZEN or not job["fw_id"] or job["board"]["fqbn"] != BOARDS["Arduino Uno"]["fqbn"]:
            return
        target = os.path.join(APP_DIR, "firmware", FIRMWARES[job["fw_id"]]["sketch"] + ".hex")
        try:
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copyfile(hex_path, target)
        except OSError:
            pass

    def _determinate(self):
        self.progress.stop()
        self.progress.configure(mode="determinate", value=0)

    def _upload_done(self, ok, message, job):
        self._set_busy(False, message, COLOR_OK if ok else COLOR_ERROR)
        self.progress.configure(value=1.0 if ok else 0)
        self._update_options()
        if ok:
            self.log_view.write(message)
            if self.auto_connect.get() and job["fw_id"]:
                self.after(500, lambda: self.app.connect_tab(job["fw_id"], job["port"]))
        else:
            messagebox.showerror("Téléversement", "Le téléversement a échoué : voir le journal.")

    def install_dependencies(self):
        if self.busy or not self.cli_path:
            return
        libs = sorted({lib for fw in FIRMWARES.values() for lib in fw["libs"]})
        self.log_view.clear()
        self._set_busy(True, "Installation des dépendances...")

        def worker():
            try:
                ArduinoCli(self.cli_path, self._log).install_dependencies(libs)
                self.app.post(self._set_busy, False, "Dépendances installées", COLOR_OK)
            except UploadError as e:
                self._log(f"ERREUR : {e}")
                self.app.post(self._set_busy, False, "Échec de l'installation", COLOR_ERROR)
        threading.Thread(target=worker, daemon=True).start()


# ============================================================================ Application
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_NAME)
        self.settings = Settings()
        self._ui_queue = queue.Queue()
        self._setup_style()

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=8, pady=(8, 4))
        self.winder = WinderTab(self.notebook, self)
        self.current = CurrentTab(self.notebook, self)
        self.firmware = FirmwareTab(self.notebook, self)
        self.device_tabs = (self.winder, self.current)
        self.notebook.add(self.winder, text="  Bobineuse  ")
        self.notebook.add(self.current, text="  Courant  ")
        self.notebook.add(self.firmware, text="  Firmware  ")

        tk.Button(self, text="ARRÊT D'URGENCE  (Échap)  -  moteurs + courant", bg=COLOR_ERROR, fg="white",
                  activebackground="#8e0000", activeforeground="white", font=("", 12, "bold"),
                  command=self.emergency_stop).grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        self.bind_all("<Escape>", lambda e: self.emergency_stop())
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        size = self.settings.get("window_size")
        if isinstance(size, str) and "x" in size:
            self.geometry(size)
        tab = self.settings.get("tab", 0)
        if isinstance(tab, int) and 0 <= tab < 3:
            self.notebook.select(tab)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(UI_POLL_MS, self._process_ui_queue)

    def _setup_style(self):
        style = ttk.Style(self)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("TNotebook.Tab", font=("", 10, "bold"))

    # ----- thread-safe dispatch to the GUI thread
    def post(self, fn, *args):
        self._ui_queue.put((fn, args))

    def _process_ui_queue(self):
        # Rescheduled first, so the queue keeps flowing even while a callback shows a modal dialog
        self.after(UI_POLL_MS, self._process_ui_queue)
        deadline = time.perf_counter() + 0.05          # keep the GUI responsive under heavy load
        while time.perf_counter() < deadline:
            try:
                fn, args = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            try:
                fn(*args)
            except Exception:
                self.report_callback_exception(*sys.exc_info())

    # ----- coordination between tabs
    def release_port(self, port, exclude=None):
        """Closes the connection another tab holds on port (after confirmation)."""
        for tab in self.device_tabs:
            if tab is not exclude and tab.link.is_open and tab.link.port == port:
                name = self.notebook.tab(tab, "text").strip()
                if not messagebox.askyesno("Port occupé",
                                           f"{port} est utilisé par l'onglet « {name} ».\n"
                                           f"Le déconnecter (les sorties seront arrêtées) ?"):
                    return False
                tab.disconnect(f"Port {port} libéré")
        return True

    def show_firmware_tab(self, firmware_id, port):
        self.firmware.preselect(firmware_id, port)
        self.notebook.select(self.firmware)

    def connect_tab(self, firmware_id, port):
        for tab in self.device_tabs:
            if tab.FIRMWARE_ID == firmware_id:
                tab.conn_bar.set_port(port)
                if not tab.link.is_open:
                    tab.connect()
                self.notebook.select(tab)

    def emergency_stop(self):
        for tab in self.device_tabs:
            tab.emergency_stop()

    def on_close(self):
        if self.firmware.busy and not messagebox.askyesno(
                "Téléversement en cours", "Un téléversement est en cours. Quitter quand même ?"):
            return
        for tab in (*self.device_tabs, self.firmware):
            tab.save_settings()
        if self.state() == "normal" and self.winfo_width() > 200:
            self.settings.set("window_size", f"{self.winfo_width()}x{self.winfo_height()}")
        self.settings.set("tab", self.notebook.index(self.notebook.select()))
        self.settings.save()
        for tab in self.device_tabs:
            tab.disconnect()
        self.current.stop_recording()
        self.destroy()


def enable_high_dpi():
    if os.name == "nt":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass


if __name__ == "__main__":
    enable_high_dpi()
    App().mainloop()
