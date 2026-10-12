"""Live monitor + logger for the robot's mic meter.

Usage: python scripts/mic_log.py [port]      (or PORT=<port>)

Works like `make monitor` (everything the board prints is echoed, and whatever you type is sent to it), and also
asks the board to stream mic levels (`plot on`). Every meter row is saved to data/mic_<stamp>.csv and every line to
data/mic_<stamp>.log. Stop with Ctrl+C. Run `make plot` in a second terminal to watch the rows as a graph.
"""
import os
import re
import sys
import threading
import time
from pathlib import Path

import serial
from serial.tools import list_ports

HEADER = "time_ms,l_db,r_db,l_peak,r_peak,l_dc,r_dc,loop"
ROW = re.compile(r"^\d+,-?\d+(\.\d+)?,-?\d+(\.\d+)?,\d+,\d+,-?\d+,-?\d+,[01]$")
ESPRESSIF_VID = 0x303A  # the S3's native USB port; the UART port (CH343 etc.) shows other vendors


def find_port():
    explicit = (sys.argv[1] if len(sys.argv) > 1 else "") or os.environ.get("PORT", "")
    if explicit:
        return explicit
    for p in list_ports.comports():
        if p.vid == ESPRESSIF_VID:
            return p.device
    return None


def open_port(path):
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = path, 115200, 0.2
    # Keep DTR/RTS low: asserting them resets the chip or drops it into download mode
    for line in ("dtr", "rts"):
        try:
            setattr(ser, line, False)
        except OSError:
            pass
    ser.open()
    return ser


def main():
    root = Path(__file__).resolve().parent.parent
    (root / "data").mkdir(exist_ok=True)
    base = root / "data" / time.strftime("mic_%Y%m%d_%H%M%S")
    log = open(f"{base}.log", "w")
    csv = None
    rows = 0
    state = {"ser": None}

    def forward_stdin():
        for line in sys.stdin:
            ser = state["ser"]
            if ser is not None:
                try:
                    ser.write(line.rstrip("\r\n").encode() + b"\r")
                except (serial.SerialException, OSError):
                    pass

    threading.Thread(target=forward_stdin, daemon=True).start()
    print(f"Logging to {base}.log and {base}.csv (Ctrl+C to stop; type commands here as in the monitor)")

    pending = b""
    try:
        while True:
            port = find_port()
            if port is None:
                print("[mic_log] no ESP32-S3 USB port found; waiting (plug in the board's USB port, not UART)...")
                time.sleep(1)
                continue
            try:
                ser = open_port(port)
            except (serial.SerialException, OSError) as e:
                print(f"[mic_log] cannot open {port}: {e}; retrying")
                time.sleep(1)
                continue

            state["ser"] = ser
            print(f"[mic_log] connected to {port}")
            time.sleep(0.5)
            ser.write(b"plot on\r")  # a reboot turns streaming off, so ask again after every reconnect
            try:
                while True:
                    pending += ser.read(512)
                    while b"\n" in pending:
                        raw, pending = pending.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").rstrip("\r")
                        print(line, flush=True)
                        log.write(line + "\n")
                        log.flush()
                        if ROW.match(line):
                            if csv is None:
                                csv = open(f"{base}.csv", "w")
                                csv.write(HEADER + "\n")
                            csv.write(line + "\n")
                            csv.flush()
                            rows += 1
            except (serial.SerialException, OSError) as e:
                state["ser"] = None
                print(f"[mic_log] port lost ({e}); reconnecting")
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        ser = state["ser"]
        if ser is not None:
            try:
                ser.write(b"plot off\r")
                time.sleep(0.2)
                ser.close()
            except (serial.SerialException, OSError):
                pass
        print()
        if csv is not None:
            print(f"Saved {rows} rows\n  {base}.csv\n  {base}.log")
        else:
            print(f"No meter rows received. Is the board running this firmware?\n  {base}.log")


if __name__ == "__main__":
    main()
