"""Check telemetry and STOP-command ACK via an external USB-UART (no motor start)."""
import argparse
import time
import serial


def inspect_line(line):
    if line.startswith("T,"):
        fields = line.split(",")
        if len(fields) == 12:
            try:
                return ("telemetry", [int(value) for value in fields[1:]])
            except ValueError:
                pass
    if line == "ACK,S,0,pwm_init=0":
        return ("ack", None)
    return ("other", None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="External adapter port, e.g. COM32")
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--confirm-safe", action="store_true",
                        help="Confirm driver 12V disconnected and serial assistant closed")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if not args.confirm_safe:
        parser.error("Disconnect driver 12V, close other serial apps, then use --confirm-safe")
    ser = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=1)
    ser.port, ser.dtr, ser.rts = args.port, False, False
    frames, acks = 0, 0
    try:
        ser.open()
        # Clear stale input so a previous STOP reply cannot pass this test.
        ser.reset_input_buffer()
        ser.write(b"\n")  # terminate a possible partial firmware command
        # Send only STOP; pace bytes as the current firmware polls RX.
        for byte in b"S0\n":
            ser.write(bytes([byte]))
            time.sleep(.005)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            kind, values = inspect_line(line)
            if kind == "telemetry":
                frames += 1
                if frames == 1:
                    print("RX telemetry:", line)
            elif kind == "ack":
                acks += 1
                print("STOP accepted:", line)
            else:
                print("RX:", line)
        print(f"Telemetry frames={frames}; STOP ACKs={acks}")
        if frames and acks:
            print("PASS: both UART directions responded.")
            return 0
        if frames:
            print("FAIL: MCU TX works; inspect adapter TX -> J5-3/P4_2 and firmware RX.")
        else:
            print("FAIL: no valid telemetry; inspect firmware/version, power, baud and J5-4/P4_5 -> adapter RX.")
        return 1
    except (OSError, serial.SerialException) as exc:
        print("Serial error:", exc)
        return 2
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
