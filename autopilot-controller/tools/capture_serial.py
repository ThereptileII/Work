#!/usr/bin/env python3
"""Capture ESP32 bridge diagnostics on Linux, using only the Python standard library."""

import argparse
from datetime import datetime, timezone
import fcntl
import glob
import math
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time
import tty


def duration(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0:
        raise argparse.ArgumentTypeError("seconds must be finite and >= 0")
    return seconds


def ports():
    # Prefer stable USB identities and avoid listing their tty aliases twice.
    found = {}
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        for port in sorted(glob.glob(pattern)):
            found.setdefault(os.path.realpath(port), port)
    return list(found.values())


def send(fd, command):
    data = memoryview(command)
    deadline = time.monotonic() + 1
    while data:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise OSError("serial command write timed out")
        try:
            count = os.write(fd, data)
        except BlockingIOError:
            continue
        if count == 0:
            raise OSError("serial port stopped accepting commands")
        data = data[count:]


def capture(port, output, seconds, listen_only=False, can_trace=False):
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation protects previous captures from accidental overwrite.
    with output.open("xb", buffering=0) as log:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        original = None
        total = 0
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            original = termios.tcgetattr(fd)
            tty.setraw(fd, termios.TCSANOW)
            settings = termios.tcgetattr(fd)
            settings[2] |= termios.CLOCAL | termios.CREAD
            settings[2] &= ~(termios.CRTSCTS | termios.CSTOPB)
            settings[4] = settings[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            # Deassert the usual ESP32 boot/reset control lines. Some USB drivers
            # still pulse these at open; allow startup time and retain boot output.
            try:
                fcntl.ioctl(fd, termios.TIOCMBIC,
                            struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
            except OSError:
                pass  # Pseudo-terminals and some adapters lack modem controls.
            start = time.monotonic()
            next_status = start + 2
            print(f"Capturing {port} at 115200 baud → {output}\nCtrl+C to stop.",
                  file=sys.stderr)
            while seconds == 0 or time.monotonic() - start < seconds:
                now = time.monotonic()
                if now >= next_status:
                    # Re-enable raw logging after a board restart, too.
                    commands = b"raw on\nstatus\n"
                    if listen_only:
                        commands = b"listen on\n" + commands
                    if can_trace:
                        commands = b"can on\n" + commands
                    send(fd, commands)
                    next_status = now + 5
                if select.select([fd], [], [], 0.1)[0]:
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        continue
                    if not data:
                        raise OSError("serial device disconnected")
                    log.write(data)
                    total += len(data)
                    sys.stdout.write(data.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            # Raw logging is left on; no steering commands are ever sent.
            if original is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, original)
                except OSError:
                    pass
            os.close(fd)
            print(f"\nSaved {total} serial bytes to {output}", file=sys.stderr)
            if total == 0:
                print("No serial output received; check the port, firmware and USB cable.",
                      file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device; auto-selects if exactly one is found")
    parser.add_argument("--list", action="store_true", help="list available USB serial ports")
    parser.add_argument("--seconds", type=duration, default=60,
                        help="capture duration, including startup (default: 60; 0: until Ctrl+C)")
    parser.add_argument("--output", type=Path, help="new log file (default: logs/serial-UTC.log)")
    parser.add_argument("--listen-only", action="store_true",
                        help="disable all SeaTalk transmissions while capturing; leaves this mode enabled")
    parser.add_argument("--can", action="store_true",
                        help="also log CAN frames for autopilot, navigation, wind and device discovery")
    args = parser.parse_args()
    available = ports()
    if args.list:
        print("\n".join(available) or "No USB serial ports found.")
        return 0
    port = args.port
    if not port:
        if len(available) != 1:
            parser.error("specify --port; found: " + (", ".join(available) or "no USB serial ports"))
        port = available[0]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = args.output or Path(__file__).resolve().parents[1] / "logs" / f"serial-{stamp}.log"
    try:
        capture(port, output, args.seconds, args.listen_only, args.can)
    except (OSError, termios.error) as error:
        print(f"Capture failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
