"""Find the external CH343 adapter without relying on its COM number."""
from serial.tools import list_ports

ADAPTER_SERIAL = "5A7B028493"


def resolve_port(port="auto"):
    if port.lower() != "auto":
        return port
    ports = list(list_ports.comports())
    matches = [p for p in ports if p.serial_number == ADAPTER_SERIAL]
    if not matches:
        matches = [p for p in ports if p.vid == 0x1A86 and p.pid == 0x55D3]
    if len(matches) != 1:
        available = ", ".join(f"{p.device}: {p.description}" for p in ports)
        raise ValueError("Cannot uniquely select motor adapter; specify COM explicitly. "
                         + available)
    return matches[0].device
