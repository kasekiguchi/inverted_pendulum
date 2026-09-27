"""Serial link to the firmware (see README.md for the protocol)."""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path

import serial

LOG_FIELDS = ["t_ms", "state", "p", "th", "v", "dth", "u", "exec_us"]
BAUD = 921600


class Device:
    def __init__(self, port: str, baud: int = BAUD):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, baud, 0.1
        # The FIRE's auto-reset circuit reboots the ESP32 when DTR/RTS toggle.
        s.dtr = s.rts = False
        s.open()
        self.ser = s

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def readline(self) -> str | None:
        raw = self.ser.readline()
        return raw.decode(errors="replace").strip() if raw else None

    def cmd(self, line: str, timeout: float = 1.0, echo: bool = True) -> list[str]:
        """Send one command and return its '#' response lines (up to '# OK' / '# ERR')."""
        self.ser.write((line + "\n").encode())
        out = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r = self.readline()
            if not r or r.startswith("D,"):
                continue
            out.append(r)
            if echo:
                print(r)
            if r == "# OK" or r.startswith("# ERR"):
                break
        return out

    def record(self, path: str | Path, duration: float, commands: list[str] = ()) -> int:
        """Log D-lines for `duration` seconds after sending `commands`; returns row count."""
        self.cmd("LOG 1", echo=False)
        for c in commands:
            self.ser.write((c + "\n").encode())
        rows = 0
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(LOG_FIELDS)
            end = time.monotonic() + duration
            while time.monotonic() < end:
                r = self.readline()
                if not r:
                    continue
                if r.startswith("D,"):
                    w.writerow(r.split(",")[1:])
                    rows += 1
                elif not r.startswith("# OK"):
                    print(r)
        self.cmd("LOG 0", echo=False)
        return rows

    def terminal(self):
        """Interactive console. D-lines are hidden."""
        stop = threading.Event()

        def reader():
            while not stop.is_set():
                r = self.readline()
                if r and not r.startswith("D,"):
                    print(r)

        t = threading.Thread(target=reader, daemon=True)
        t.start()
        print("Commands are sent as typed. Ctrl-D / 'exit' to quit.")
        try:
            while True:
                line = input()
                if line.strip().lower() in ("exit", "quit"):
                    break
                self.ser.write((line + "\n").encode())
        except (EOFError, KeyboardInterrupt):
            pass
        finally:
            self.ser.write(b"STOP\n")
            stop.set()
            t.join(0.5)


def load_log(path: str | Path) -> dict:
    import numpy as np

    data = np.genfromtxt(path, delimiter=",", names=True)
    log = {k: np.atleast_1d(data[k]) for k in LOG_FIELDS}
    log["t"] = (log["t_ms"] - log["t_ms"][0]) * 1e-3
    return log
