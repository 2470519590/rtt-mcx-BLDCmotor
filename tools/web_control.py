"""Web command validation and per-mode profiles; no serial or hardware access."""
import math


# User-facing units, identical to control_cli.py and the deployed protocol.
PARAMETERS = {
    "kp": ("P", 1000, 0, .2),
    "ki": ("Q", 1000, 0, 2),
    "poskp": ("Z", 1000, 0, 20),
    "posmax": ("Y", 1, 1, 1000),
    "posmove": ("t", 1, 0, 2000),
    "voltage": ("V", 1000, .1, 1.5),
}
DEFAULT_PROFILES = {
    "speed": dict(kp=.008, ki=.284, poskp=4, posmax=60, posmove=0, voltage=1),
    "position": dict(kp=.040, ki=.650, poskp=15, posmax=90, posmove=500, voltage=1),
}


def number(value, name):
    try:
        finite = not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{name}必须是有限数值")
    return value


def profile_settings(values):
    if not isinstance(values, dict) or set(values) != set(PARAMETERS):
        raise ValueError("请提交完整的六项控制参数")
    normalized, commands = {}, []
    for key, (op, scale, low, high) in PARAMETERS.items():
        value = number(values[key], key)
        if not low <= value <= high:
            raise ValueError(f"{key}范围为{low:g}至{high:g}")
        if key in ("posmax", "posmove") and value != round(value):
            raise ValueError(f"{key}必须为整数")
        if key == "posmove" and 0 < value < 100:
            raise ValueError("轨迹时长为0（原路径）或100至2000 ms")
        wire = round(value * scale)
        normalized[key] = wire / scale
        commands.append((op, wire))
    return normalized, commands


def position_target(degrees, reference, origin_mrad=0):
    degrees = number(degrees, "角度")
    if reference not in ("absolute", "relative"):
        raise ValueError("位置参考只能为absolute或relative")
    radians = math.radians(degrees)
    if not math.isfinite(radians) or abs(radians) > 200:
        raise ValueError("位置超出指令范围")
    target = round(radians * 1000) + (origin_mrad if reference == "relative" else 0)
    if abs(target) > 100000:
        raise ValueError("目标位置超出固件±100 rad范围")
    return target
