from types import SimpleNamespace

import pytest

from gello_piper_direct import discovery
from gello_piper_direct.errors import ConfigurationError


def port(device, description, serial_number):
    return SimpleNamespace(
        device=device,
        description=description,
        manufacturer="FTDI",
        product=description,
        serial_number=serial_number,
        vid=0x0403,
        pid=0x6001,
    )


def test_serial_number_selects_exact_device(monkeypatch) -> None:
    monkeypatch.setattr(
        discovery.list_ports,
        "comports",
        lambda: [
            port("/dev/cu.gello", "USB Serial", "G1"),
            port("/dev/cu.can", "SLCAN", "P1"),
        ],
    )
    assert (
        discovery.resolve_serial_port(
            "auto", role="piper", serial_number="P1"
        )
        == "/dev/cu.can"
    )


def test_ambiguous_selection_requires_config(monkeypatch) -> None:
    monkeypatch.setattr(
        discovery.list_ports,
        "comports",
        lambda: [
            port("/dev/cu.one", "USB Serial", "1"),
            port("/dev/cu.two", "USB Serial", "2"),
        ],
    )
    with pytest.raises(ConfigurationError, match="无法唯一确定"):
        discovery.resolve_serial_port("auto", role="gello")


def test_macos_bluetooth_pseudo_port_is_filtered(monkeypatch) -> None:
    pseudo = port("/dev/cu.QCYH3Pro", "n/a", None)
    pseudo.vid = None
    pseudo.pid = None
    monkeypatch.setattr(discovery.list_ports, "comports", lambda: [pseudo])
    assert discovery.list_serial_devices() == []
