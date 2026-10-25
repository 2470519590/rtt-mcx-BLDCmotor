#!/usr/bin/env python3
"""Read-only AS5600 angle CLI over the board's USB VCOM."""

import argparse
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="COM port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(
            f"Opened {args.port} at {args.baud} baud. "
            "Commands: help, angle, state, quit"
        )
        time.sleep(0.2)
        while ser.in_waiting:
            line = ser.readline()
            if line:
                sys.stdout.write(line.decode("utf-8", errors="replace"))
        while True:
            try:
                command = input("encoder> ").strip()
            except EOFError:
                command = "quit"
            if command in ("quit", "exit"):
                return 0
            if not command:
                continue
            ser.write((command + "\r\n").encode("ascii"))
            deadline = time.monotonic() + 0.8
            received = False
            while time.monotonic() < deadline:
                line = ser.readline()
                if line:
                    sys.stdout.write(line.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
                    received = True
            if not received:
                print(
                    "No reply within 0.8 s. The port opened, but the board may "
                    "not be running this CLI firmware; check the flashed image, "
                    "COM port, and 115200 baud."
                )


if __name__ == "__main__":
    raise SystemExit(main())
