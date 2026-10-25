"""Explicit one-time post-flash calibration, with heartbeat and stop cleanup."""
import argparse
import threading
import time
import serial
from collect_nn_dataset import tx, send_checked, read_state
from motor_port import resolve_port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibrate-if-needed', action='store_true')
    args = parser.parse_args()
    port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.5)
    port.port = resolve_port('auto')
    port.dtr = False
    port.rts = False
    lock, done = threading.Lock(), threading.Event()
    def heartbeat():
        while not done.wait(.2):
            try:
                tx(port, 'H', 0, lock)
            except Exception:
                done.set()
    port.open()
    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    try:
        send_checked(port, 'S', 0, lock)
        send_checked(port, 'T', 0, lock)
        state = read_state(port, require_calibrated=False)
        print('INITIAL', ','.join(state), flush=True)
        # ACK proves new command implementation; does not assert model hash.
        send_checked(port, 'G', 5, lock)
        send_checked(port, 'G', 0, lock)
        print('ANGLE_SHADOW_COMMAND_ACCEPTED', flush=True)
        if state[7] != '1':
            if not args.calibrate_if_needed:
                raise RuntimeError('explicit --calibrate-if-needed required')
            send_checked(port, 'C', 0, lock)
            print('CALIBRATION_STARTED_ONCE', flush=True)
            read_state(port, timeout=8)
            time.sleep(1)
        port.reset_input_buffer()
        print('READY', ','.join(read_state(port)), flush=True)
    finally:
        try:
            send_checked(port, 'S', 0, lock)
        finally:
            done.set()
            worker.join(1)
            port.close()


if __name__ == '__main__':
    main()
