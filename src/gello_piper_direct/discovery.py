"""Conservative serial-port discovery for macOS and Linux."""

from __future__ import annotations

from dataclasses import dataclass

from serial.tools import list_ports

from .errors import ConfigurationError


@dataclass(frozen=True, slots=True)
class SerialDevice:
    device: str
    description: str
    manufacturer: str | None
    product: str | None
    serial_number: str | None
    vid: int | None
    pid: int | None

    @property
    def label(self) -> str:
        detail = self.product or self.description or "USB serial"
        serial = f" serial={self.serial_number}" if self.serial_number else ""
        return f"{self.device} ({detail}{serial})"


@dataclass(frozen=True, slots=True)
class GsUsbDevice:
    index: int
    bus: int | None
    address: int | None
    vid: int | None
    pid: int | None
    product: str | None
    serial_number: str | None

    @property
    def label(self) -> str:
        ids = (
            f"{self.vid:04x}:{self.pid:04x}"
            if self.vid is not None and self.pid is not None
            else "VID:PID=?"
        )
        product = self.product or "GS-USB CAN"
        serial = f" serial={self.serial_number}" if self.serial_number else ""
        return f"index={self.index} ({product} {ids}{serial})"


def list_serial_devices() -> list[SerialDevice]:
    """Return real USB serial devices, excluding Bluetooth pseudo-ports."""

    devices: list[SerialDevice] = []
    for item in list_ports.comports():
        text = f"{item.device} {item.description}".lower()
        if "bluetooth" in text or "debug-console" in text:
            continue
        # macOS exposes paired audio/headset devices as /dev/cu.<name> with no
        # USB metadata. Keep only plausible physical serial adapters when the
        # OS cannot provide VID/PID information.
        if item.vid is None and not any(
            token in text
            for token in (
                "usbserial",
                "usbmodem",
                "usb serial",
                "slab_usb",
                "wchusb",
                "ttyusb",
                "ttyacm",
            )
        ):
            continue
        devices.append(
            SerialDevice(
                device=item.device,
                description=item.description or "",
                manufacturer=item.manufacturer,
                product=item.product,
                serial_number=item.serial_number,
                vid=item.vid,
                pid=item.pid,
            )
        )
    return sorted(devices, key=lambda item: item.device)


def list_gs_usb_devices() -> list[GsUsbDevice]:
    """Return candleLight/GS-USB CAN adapters visible through libusb."""

    try:
        from gs_usb.gs_usb import GsUsb
    except ImportError:
        return []
    try:
        scanned = GsUsb.scan()
    except Exception as exc:
        raise ConfigurationError(f"GS-USB 扫描失败: {exc}") from exc
    devices = []
    for index, item in enumerate(scanned):
        usb_device = getattr(item, "gs_usb", None)
        if usb_device is None:
            continue
        try:
            product = usb_device.product
        except Exception:
            product = None
        try:
            serial_number = usb_device.serial_number
        except Exception:
            serial_number = None
        devices.append(
            GsUsbDevice(
                index=index,
                bus=getattr(usb_device, "bus", None),
                address=getattr(usb_device, "address", None),
                vid=getattr(usb_device, "idVendor", None),
                pid=getattr(usb_device, "idProduct", None),
                product=product,
                serial_number=serial_number,
            )
        )
    return devices


def resolve_gs_usb_index(
    requested: str | int, *, serial_number: str | None = None
) -> int:
    """Resolve a configured or uniquely discoverable GS-USB device index."""

    if isinstance(requested, int):
        if requested < 0:
            raise ConfigurationError("GS-USB index 不能为负数")
        return requested
    if requested != "auto":
        if requested.isdigit():
            return int(requested)
        raise ConfigurationError("GS-USB channel 必须是 auto 或整数索引")
    devices = list_gs_usb_devices()
    if serial_number:
        devices = [
            item for item in devices if item.serial_number == serial_number
        ]
    if len(devices) == 1:
        return devices[0].index
    labels = ", ".join(item.label for item in devices) or "无"
    raise ConfigurationError(
        f"无法唯一确定 Piper GS-USB（候选: {labels}）。"
        "请显式填写 piper.channel 或 serial_number。"
    )


def resolve_serial_port(
    requested: str,
    *,
    role: str,
    serial_number: str | None = None,
    exclude: set[str] | None = None,
) -> str:
    """Resolve an explicit or uniquely discoverable serial device."""

    if requested != "auto":
        return requested
    exclude = exclude or set()
    devices = [item for item in list_serial_devices() if item.device not in exclude]
    if serial_number:
        devices = [item for item in devices if item.serial_number == serial_number]
    elif role == "gello":
        preferred = [
            item
            for item in devices
            if any(
                token in " ".join(
                    filter(
                        None,
                        (item.product, item.description, item.manufacturer),
                    )
                ).lower()
                for token in ("ftdi", "usb serial", "u2d2", "dynamixel")
            )
        ]
        devices = preferred or devices
    elif role == "piper":
        preferred = [
            item
            for item in devices
            if "can" in f"{item.product} {item.description}".lower()
        ]
        devices = preferred or devices

    if len(devices) == 1:
        return devices[0].device
    labels = ", ".join(item.label for item in devices) or "无"
    raise ConfigurationError(
        f"无法唯一确定 {role} 串口（候选: {labels}）。"
        f"请在 config.yaml 中显式填写 {role} 的 port/channel 或 serial_number。"
    )
