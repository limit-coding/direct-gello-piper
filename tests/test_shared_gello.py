import time

import numpy as np
import pytest

from gello_piper_direct.config import load_dual_config
from gello_piper_direct.errors import DeviceConnectionError
from gello_piper_direct.shared_gello import SharedGelloBus


class AliveThread:
    def is_alive(self):
        return True


class FakeDriver:
    def __init__(self):
        self._reading_thread = AliveThread()
        self._joint_angles = np.arange(14, dtype=float)


def test_shared_bus_can_reuse_latest_fresh_sample_without_blocking() -> None:
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    bus = SharedGelloBus(load_dual_config(root / "dual_config.yaml"))
    bus._driver = FakeDriver()
    first = bus._next_raw(0.1, require_fresh=True)

    started = time.perf_counter()
    second = bus._next_raw(0.1, require_fresh=False)
    elapsed = time.perf_counter() - started

    np.testing.assert_allclose(second, first)
    assert elapsed < 0.02


def test_shared_bus_rejects_stale_cached_sample() -> None:
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    bus = SharedGelloBus(load_dual_config(root / "dual_config.yaml"))
    bus._driver = FakeDriver()
    bus._next_raw(0.1, require_fresh=True)
    bus._last_packet_at = time.monotonic() - 0.3

    with pytest.raises(DeviceConnectionError, match="0.25"):
        bus._next_raw(0.1, require_fresh=False)
