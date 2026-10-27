#!/usr/bin/env python3
"""Local web monitor for FRDM-MCXA346 AS5600 streaming telemetry."""

from __future__ import annotations

import argparse
from collections import deque
import json
import re
import socket
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import serial
from serial.tools import list_ports
from motor_demo import DemoRunner
from web_control import DEFAULT_PROFILES, PARAMETERS, number, position_target, profile_settings


ROOT = Path(__file__).resolve().parent / "web"
SERVICE_REVISION = "manual-control-20261026b"
STATE_RE = re.compile(
    r"raw=(?P<raw>\d+)\s+angle_mrad=(?P<mrad>\d+)\s+"
    r"angle_cdeg=(?P<cdeg>\d+)\s+speed_mrad_s=(?P<speed>-?\d+)\s+"
    r"sensor=(?P<sensor>ok|error)"
)


class SerialMonitor:
    def __init__(self, baud: int) -> None:
        self.baud = baud
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.command_lock = threading.Lock()
        self.operation_lock = threading.RLock()
        self.profiles = {name: dict(values) for name, values in DEFAULT_PROFILES.items()}
        self.acknowledged = {}
        self.manual = dict(active=False, kind=None, target=None, message="待命")
        self.replies = deque(maxlen=64)
        self.demo_state = dict(running=False, phase="待命", error=None, target_deg=None,
                               origin_mrad=None)
        self.connection_id = 0
        self.demo = DemoRunner(self.request, self.snapshot, self._demo_update)
        self.revision = 0
        self.rate_start = time.monotonic()
        self.rate_count = 0
        self.rx_hz = 0.0
        self.arrivals = deque(maxlen=201)
        self.last_sequence = None
        self.dropped_frames = 0
        self.ser: serial.Serial | None = None
        self.port: str | None = None
        self.error: str | None = None
        self.last_line = ""
        self.rx_bytes = 0
        self.data: dict[str, Any] = {
            "raw": None,
            "angle_mrad": None,
            "angle_cdeg": None,
            "speed_mrad_s": None,
            "sensor": "waiting",
            "updated_at": None,
        }
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._poll, name="serial-monitor", daemon=True)
        self.thread.start()
        self.keeper = threading.Thread(target=self._manual_keepalive,
                                       name="manual-keepalive", daemon=True)
        self.keeper.start()

    def ports(self) -> list[dict[str, str]]:
        return [
            {"device": item.device, "description": item.description}
            for item in list_ports.comports()
        ]

    def connect(self, port: str) -> None:
        with self.operation_lock:
            self._connect(port)

    def _connect(self, port: str) -> None:
        if self.demo_state["running"]:
            raise ValueError("请先停止演示")
        if not port or len(port) > 64:
            raise ValueError("Invalid serial port")
        # Configure modem-control outputs before opening the port. Some USB
        # VCOM bridges connect DTR/RTS to target reset/boot circuitry; opening
        # with pySerial defaults can assert those lines and leave the MCU
        # stopped or in its bootloader instead of running the CLI firmware.
        candidate = serial.Serial(
            port=None,
            baudrate=self.baud,
            timeout=0.01,
            write_timeout=0.5,
            dsrdtr=False,
            rtscts=False,
        )
        candidate.port = port
        candidate.dtr = False
        candidate.rts = False
        candidate.open()
        with self.lock:
            old = self.ser
            self.ser = candidate
            self.port = port
            self.connection_id += 1
            self._reset_demo_state()
            self.error = None
            self.last_line = ""
            self.rx_bytes = 0
            self.arrivals.clear()
            self.last_sequence = None
            self.dropped_frames = 0
            self.rx_hz = 0.0
            self.rate_start = time.monotonic()
            self.rate_count = 0
            self.data = {
                "raw": None,
                "angle_mrad": None,
                "angle_cdeg": None,
                "speed_mrad_s": None,
                "sensor": "waiting",
                "updated_at": None,
            }
            if old is not None:
                old.close()

    def disconnect(self) -> None:
        with self.operation_lock:
            self.demo.stop(wait=True)
            with self.lock:
                current, self.ser = self.ser, None
                self.port = None
                self.error = None
                self._reset_demo_state()
                if current is not None:
                    try:
                        current.write(b'S0\n')
                    except (OSError, serial.SerialException):
                        pass
                    current.close()

    def _reset_demo_state(self):
        """Called under the monitor lock after stopping the old demo."""
        self.replies.clear()
        self.acknowledged.clear()
        self.manual = dict(active=False, kind=None, target=None, message="待命")
        self.demo_state = dict(running=False, phase="待命", error=None,
                               target_deg=None, origin_mrad=None)
        self.revision += 1
        self.changed.notify_all()

    def _demo_update(self, **values):
        with self.changed:
            self.demo_state.update(values)
            self.revision += 1
            self.changed.notify_all()

    def request(self, op, value=0):
        """One request at a time; replies are delivered by the existing reader."""
        with self.command_lock:
            with self.changed:
                if self.ser is None:
                    raise ValueError("串口未连接")
                self.replies.clear()
                boundary = time.monotonic()
                self.ser.write(f"{op}{value}\n".encode("ascii"))
                if op == "H":
                    return 0
                deadline = boundary+1
                while True:
                    for stamp, fields in self.replies:
                        if stamp < boundary:
                            continue
                        if op == "c" and fields[0] == "POS" and len(fields) == 6:
                            result, position, sensor, calibrated, fault = map(int, fields[1:])
                            if result != 0:
                                raise ValueError(f"当前位置查询失败：{','.join(fields)}")
                            return position
                        if len(fields) >= 3 and fields[:2] == ["ACK", op]:
                            if op == "c":
                                raise ValueError(f"位置查询收到 ACK 而非 POS：{','.join(fields)}")
                            if int(fields[2]) != 0:
                                raise ValueError(f"固件拒绝 {op}{value}：{','.join(fields)}")
                            self._acknowledge(op, value)
                            return 0
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise ValueError(f"{op}{value} 回复超时")
                    self.changed.wait(remaining)

    def _acknowledge(self, op, value):
        # Last successful ACKs, not fabricated firmware readbacks.
        if op in {item[0] for item in PARAMETERS.values()} | {"G", "K", "T"}:
            self.acknowledged[op] = value
        if op == "S":
            self.manual.update(active=False, message="已停止")
        self.revision += 1
        self.changed.notify_all()

    def _prepare_manual(self, kind):
        _, commands = profile_settings(self.profiles[kind])
        # Switch profiles while stopped. Repeated X commands in the same
        # profile retain the tested continuous trajectory and integral.
        if any(self.acknowledged.get(op) != value for op, value in commands):
            self.request("S", 0)
            self.request("T", 0)
            for op, value in commands:
                self.request(op, value)

    def start_demo(self, calibration=False):
        with self.operation_lock:
            if self.manual["active"]:
                self.request("S", 0)
            if calibration:
                self.demo.start_calibration()
            else:
                self.demo.start()

    def command(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("控制请求必须为JSON对象")
        with self.operation_lock:
            action = payload.get("action")
            if action == "heartbeat":
                # Compatibility with already-open pages; the server owns H0.
                return
            if action == "stop":
                self.demo.stop(wait=True)
                self.request("S", 0)
                return
            if self.demo_state["running"]:
                raise ValueError("请先停止演示或校准，再手动控制")
            if action in ("parameters", "nn") and payload.get("confirmed") is not True:
                raise ValueError("修改参数或NN设置需要二次确认")
            if action == "parameters":
                kind = payload.get("profile")
                if kind not in self.profiles:
                    raise ValueError("请选择速度环或位置环参数")
                values, commands = profile_settings(payload.get("values"))
                self.request("S", 0)
                for op, value in commands:
                    self.request(op, value)
                with self.changed:
                    self.profiles[kind] = values
                    self.manual["message"] = "参数已应用，电机保持停止"
            elif action == "nn":
                enabled = payload.get("enabled")
                if not isinstance(enabled, bool):
                    raise ValueError("NN开关必须为布尔值")
                alpha = number(payload.get("alpha"), "NN融合系数")
                if not 0 <= alpha <= 1:
                    raise ValueError("NN融合系数范围为0至1")
                self.request("S", 0)
                self.request("K", round(alpha * 1000))
                self.request("G", 2 if enabled else 0)
                with self.changed:
                    self.manual["message"] = "NN设置已应用，电机保持停止"
            elif action == "run":
                rpm = number(payload.get("rpm"), "转速")
                if rpm != round(rpm) or abs(rpm) > 1000:
                    raise ValueError("转速须为±1000 RPM内的整数")
                if rpm == 0:
                    self.request("S", 0)
                    return
                self._prepare_manual("speed")
                self.request("R", int(rpm))
                with self.changed:
                    self.manual.update(active=True, kind="speed", target=rpm,
                                       message=f"速度目标 {rpm:g} RPM")
            elif action == "position":
                degrees, reference = payload.get("degrees"), payload.get("reference", "absolute")
                # Validate before any write or profile switch.
                position_target(degrees, reference)
                origin = self.request("c", 0) if reference == "relative" else 0
                target = position_target(degrees, reference, origin)
                self._prepare_manual("position")
                self.request("X", target)
                with self.changed:
                    self.manual.update(active=True, kind="position",
                                       target=target * 180 / (1000 * 3.141592653589793),
                                       message=f"位置目标 {target * 180 / (1000 * 3.141592653589793):.2f}°")
            else:
                raise ValueError("未知控制指令")
            with self.changed:
                self.revision += 1
                self.changed.notify_all()

    def _manual_keepalive(self):
        while not self.stop_event.wait(.15):
            if not self.operation_lock.acquire(blocking=False):
                continue
            try:
                if not self.manual["active"]:
                    continue
                # Manual targets persist until an explicit stop/change.
                # Browser rendering/timer throttling must not stop the motor.
                self.request("H", 0)
            except (ValueError, OSError, serial.SerialException) as exc:
                with self.changed:
                    self.manual.update(active=False, message=f"控制通信异常：{exc}")
                    self.revision += 1
                    self.changed.notify_all()
            finally:
                self.operation_lock.release()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            result = dict(self.data)
            intervals = sorted((b - a) * 1000 for a, b in
                               zip(self.arrivals, list(self.arrivals)[1:]))
            result.update(
                service_revision=SERVICE_REVISION,
                demo=dict(self.demo_state),
                control=dict(profiles={name: dict(v) for name, v in self.profiles.items()},
                             applied={key: self.acknowledged[op] / scale
                                      for key, (op, scale, _, _) in PARAMETERS.items()
                                      if op in self.acknowledged},
                             nn_mode=self.acknowledged.get("G"),
                             nn_alpha=(self.acknowledged["K"] / 1000
                                       if "K" in self.acknowledged else None),
                             manual=dict(self.manual)),
                connection_id=self.connection_id,
                connected=self.ser is not None and self.ser.is_open,
                port=self.port,
                error=self.error,
                baud=self.baud,
                last_line=self.last_line,
                rx_bytes=self.rx_bytes,
                rx_hz=self.rx_hz,
                revision=self.revision,
                interval_mean_ms=sum(intervals) / len(intervals) if intervals else None,
                interval_p95_ms=intervals[int((len(intervals) - 1) * 0.95)] if intervals else None,
                interval_max_ms=max(intervals) if intervals else None,
                dropped_frames=self.dropped_frames,
                age_ms=(time.time() - self.data['updated_at']) * 1000 if self.data['updated_at'] else None,
            )
            return result

    def close(self) -> None:
        self.disconnect()
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.keeper.join(timeout=1.0)

    def _poll(self) -> None:
        line_buffer = bytearray()
        previous_serial = None
        while not self.stop_event.is_set():
            with self.lock:
                current = self.ser
            if current is None or not current.is_open:
                line_buffer.clear()
                self.stop_event.wait(0.1)
                continue

            try:
                if current is not previous_serial:
                    line_buffer.clear()
                    previous_serial = current
                chunk = current.read(max(1, min(current.in_waiting, 4096)))
                if chunk:
                    with self.lock:
                        if self.ser is not current:
                            continue
                        self.rx_bytes += len(chunk)
                    line_buffer.extend(chunk)
                    while b"\n" in line_buffer:
                        line, _, remainder = line_buffer.partition(b"\n")
                        line_buffer = bytearray(remainder)
                        self._consume_line(line.decode("ascii", errors="ignore").strip())
                    if len(line_buffer) > 512:
                        line_buffer.clear()
            except (serial.SerialException, OSError) as exc:
                with self.lock:
                    if self.ser is current:
                        self.error = str(exc)
                        try:
                            current.close()
                        except OSError:
                            pass
                        self.ser = None
                        self.port = None
            except Exception as exc:  # Keep the monitor alive and surface unexpected I/O errors.
                with self.lock:
                    self.error = str(exc)
                self.stop_event.wait(0.1)

    def _consume_line(self, line: str) -> None:
        with self.changed:
            self.last_line = line
            fields = line.split(",")
            if fields[0] in ("ACK", "POS"):
                self.replies.append((time.monotonic(), fields))
                self.changed.notify_all()
                return
        compact = line.split(',')
        try:
            if len(compact) in (6, 10, 12) and compact[0] == 'T':
                seq, tick, raw, speed, valid = map(int, compact[1:6])
                if not 0 <= raw < 4096 or valid not in (0, 1):
                    return
                values = dict(raw=raw, mrad=round(raw * 6.28318530718 / 4096 * 1000),
                              cdeg=round(raw * 36000 / 4096), speed=speed,
                              sensor='ok' if valid else 'error')
            else:
                match = STATE_RE.search(line)
                if not match:
                    return
                values = match.groupdict()
                seq, tick = None, None
        except ValueError:
            return
        with self.lock:
            self.arrivals.append(time.monotonic())
            if seq is not None:
                if self.last_sequence is not None:
                    delta = (seq - self.last_sequence) & 0xFFFFFFFF
                    if 0 < delta < 0x80000000:
                        self.dropped_frames += delta - 1
                self.last_sequence = seq
            self.data = {
                "raw": int(values["raw"]),
                "angle_mrad": int(values["mrad"]),
                "angle_cdeg": int(values["cdeg"]),
                "speed_mrad_s": int(values["speed"]),
                "sensor": values["sensor"],
                "updated_at": time.time(),
                "sequence": seq,
                "device_tick": tick,
                "motor_mode": int(compact[6]) if len(compact) >= 10 else None,
                "calibrated": int(compact[7]) if len(compact) >= 10 else 0,
                "target_mrad_s": int(compact[8]) if len(compact) >= 10 else 0,
                "uq_mv": int(compact[9]) if len(compact) >= 10 else 0,
                "fault": int(compact[10]) if len(compact) == 12 else None,
                "calibration_mrad": int(compact[11]) if len(compact) == 12 else None,
            }
            if self.manual["active"] and self.data["fault"]:
                self.manual.update(active=False,
                                   message=f"固件故障 {self.data['fault']}，已停止")
            self.error = None
            self.rate_count += 1
            elapsed = time.monotonic() - self.rate_start
            if elapsed >= 1.0:
                self.rx_hz = self.rate_count / elapsed
                self.rate_count = 0
                self.rate_start = time.monotonic()
            self.revision += 1
            self.changed.notify_all()


class MonitorHandler(SimpleHTTPRequestHandler):
    monitor: SerialMonitor

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def end_headers(self):
        # Static JS and the Python API must come from the same deployment.
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/ports":
            self._json(200, {"ports": self.monitor.ports()})
        elif path == "/api/state":
            self._json(200, self.monitor.snapshot())
        elif path == "/api/events":
            self._events()
        elif path == "/":
            self.path = "/index.html"
            super().do_GET()
        else:
            super().do_GET()

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        length = min(int(self.headers.get("Content-Length", "0")), 4096)
        try:
            payload = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(payload, dict):
                raise ValueError("请求必须为JSON对象")
            if path == "/api/connect":
                self.monitor.connect(str(payload.get("port", "")))
                self._json(200, {"ok": True, "state": self.monitor.snapshot()})
            elif path == "/api/disconnect":
                self.monitor.disconnect()
                self._json(200, {"ok": True})
            elif path == '/api/control':
                self.monitor.command(payload)
                self._json(200, {'ok': True, 'state': self.monitor.snapshot()})
            elif path == "/api/demo/start":
                self.monitor.start_demo()
                self._json(202, {"ok": True})
            elif path == "/api/calibrate":
                self.monitor.start_demo(calibration=True)
                self._json(202, {"ok": True})
            elif path == "/api/demo/stop":
                self.monitor.command({"action": "stop"})
                self._json(200, {"ok": True})
            else:
                self._json(404, {"error": "Not found"})
        except (ValueError, serial.SerialException, OSError) as exc:
            self._json(400, {"error": str(exc)})
        except json.JSONDecodeError:
            self._json(400, {"error": "Invalid JSON"})

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _events(self) -> None:
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.connection.settimeout(2.0)
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        revision = -1
        try:
            while not self.monitor.stop_event.is_set():
                with self.monitor.changed:
                    self.monitor.changed.wait_for(
                        lambda: self.monitor.revision != revision or self.monitor.stop_event.is_set(),
                        timeout=0.5)
                    state = self.monitor.snapshot()
                    revision = state['revision']
                self.wfile.write(('data: ' + json.dumps(state) + '\n\n').encode())
                self.wfile.flush()
        except (OSError, TimeoutError):
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--port", type=int, default=18765, help="web server port")
    args = parser.parse_args()

    MonitorHandler.monitor = SerialMonitor(args.baud)
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), MonitorHandler)
    except OSError as exc:
        # Windows may reserve a port range or deny a specific port (WinError
        # 10013), even though the process is allowed to bind localhost. Ask
        # the OS for an available ephemeral port instead of failing startup.
        if getattr(exc, "winerror", None) != 10013 and exc.errno not in (98, 10048):
            MonitorHandler.monitor.close()
            raise
        print(f"Cannot bind 127.0.0.1:{args.port} ({exc}); selecting an available local port.")
        server = ThreadingHTTPServer(("127.0.0.1", 0), MonitorHandler)
    actual_port = server.server_address[1]
    print(f"AS5600 web monitor: http://127.0.0.1:{actual_port}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping monitor.")
    finally:
        server.server_close()
        MonitorHandler.monitor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
