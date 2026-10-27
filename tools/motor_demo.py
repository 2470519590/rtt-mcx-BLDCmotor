"""Fixed demo sequence; the web monitor remains the only UART reader."""
import math
import threading
import time

POSITION_SETTINGS = (("P", 40), ("Q", 650), ("Z", 15000), ("Y", 90), ("t", 500))
RESTORE_SETTINGS = (("P", 8), ("Q", 284), ("Z", 4000), ("Y", 60), ("t", 0))


class DemoRunner:
    def __init__(self, command, snapshot, publish):
        self.command = command
        self.snapshot = snapshot
        self.publish = publish
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.worker = None

    def start(self):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError("演示正在运行")
            self._healthy()
            self.cancel.clear()
            self.publish(running=True, phase="准备", error=None,
                         target_deg=None, origin_mrad=None, operation="demo")
            self.worker = threading.Thread(target=self.run, name="motor-demo", daemon=True)
            self.worker.start()

    def start_calibration(self):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError("电机操作正在进行")
            self._healthy(require_calibration=False)
            self.cancel.clear()
            self.publish(running=True, phase="校准中", error=None,
                         target_deg=None, origin_mrad=None, operation="calibration")
            self.worker = threading.Thread(target=self.calibrate, name="motor-calibration",
                                           daemon=True)
            self.worker.start()

    def _healthy(self, require_calibration=True):
        state = self.snapshot()
        if not state.get("connected"):
            raise ValueError("请先连接串口")
        if (state.get("updated_at") is None or
                time.time()-state["updated_at"] > .5 or state.get("sensor") != "ok"):
            raise ValueError("编码器遥测未就绪")
        if require_calibration and not state.get("calibrated"):
            raise ValueError("请先点击电机校准")
        if state.get("fault"):
            raise ValueError(f"电机故障 {state['fault']}，请先停止")
        if state.get("error"):
            raise ValueError("串口读取异常："+str(state["error"]))

    def _wait(self, seconds, require_calibration=True):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if self.cancel.is_set():
                raise InterruptedError("演示已停止")
            self._healthy(require_calibration=require_calibration)
            self.command("H", 0)
            if self.cancel.wait(min(.15, max(0, end-time.monotonic()))):
                raise InterruptedError("演示已停止")

    def calibrate(self):
        failure = None
        phase = "校准完成"
        try:
            self.command("S", 0)
            self.command("C", 0)
            # Firmware aligns for 3 s, then lets the rotor settle for 750 ms.
            # Keep its existing watchdog alive through the same UART owner.
            self._wait(3.9, require_calibration=False)
            self._healthy()
            if self.snapshot().get("motor_mode") != 0:
                raise ValueError("固件尚未完成校准")
        except InterruptedError:
            phase = "校准已停止"
        except Exception as exc:
            phase, failure = "校准未完成", str(exc)
        finally:
            try:
                self.command("S", 0)
            except Exception as exc:
                failure = (failure+"；" if failure else "")+"停机确认失败："+str(exc)
                phase = "停机未确认"
            self.publish(running=False, phase=phase, error=failure)

    def _step(self, phase, **values):
        if self.cancel.is_set():
            raise InterruptedError("演示已停止")
        self.publish(running=True, phase=phase, **values)

    def run(self):
        failure = None
        phase = "演示完成"
        position_stage = False
        try:
            # Probe capability before any motor start. Old firmware must fail
            # explicitly, never estimate an absolute X target from wrapped raw.
            self.command("c", 0)
            self.command("S", 0)
            # Preserve an NN choice explicitly acknowledged through the web
            # controls. An untouched session keeps the original baseline demo.
            control = self.snapshot().get("control") or {}
            nn_mode = 2 if control.get("nn_mode") == 2 else 0
            nn_alpha = round((control.get("nn_alpha") or 0) * 1000)
            for op, value in (("T", 0), ("G", nn_mode), ("K", nn_alpha), ("V", 1000),
                              ("P", 8), ("Q", 284), ("Z", 4000), ("Y", 120)):
                self.command(op, value)
            self._step("1秒后启动", target_deg=None)
            self._wait(1)
            self.command("R", 60)
            self._step("速度环 · 60 RPM")
            self._wait(3)
            self.command("S", 0)
            self._step("停机等待 · 1秒")
            self._wait(1)
            position_stage = True
            for op, value in POSITION_SETTINGS:
                self.command(op, value)
            origin = self.command("c", 0)
            targets = [origin+round(math.radians(120*i)*1000) for i in (1, 2, 3)]
            if any(abs(target) > 100000 for target in targets):
                raise ValueError("当前位置超出位置指令范围，请重新校准位置零点")
            self.publish(origin_mrad=origin)
            for i, target in enumerate(targets, 1):
                self._step(f"位置环 · 相对 +{120*i}°", target_deg=120*i)
                self.command("X", target)
                self._wait(.5)
            # Keep the requested .5 s spacing between position commands, but
            # let the final closed-loop stop settle before removing PWM.
            self._step("位置停靠", target_deg=360)
            self._wait(1)
        except InterruptedError:
            phase = "演示已停止"
        except Exception as exc:
            phase, failure = "演示未完成", str(exc)
        finally:
            try:
                self.command("S", 0)
                if position_stage:
                    for op, value in RESTORE_SETTINGS:
                        self.command(op, value)
            except Exception as exc:
                failure = (failure+"；" if failure else "")+"停机确认失败："+str(exc)
                phase = "停机未确认"
            self.publish(running=False, phase=phase, error=failure)

    def stop(self, wait=False):
        self.cancel.set()
        if wait and self.worker and self.worker is not threading.current_thread():
            self.worker.join(3)
            if self.worker.is_alive():
                raise ValueError("演示尚未退出，串口仍保留，请再次停止")
