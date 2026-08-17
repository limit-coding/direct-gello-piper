"""macOS compatibility for the upstream ``gs-usb`` Python backend."""

from __future__ import annotations

import platform


def apply_macos_gs_usb_compat() -> None:
    """Avoid an upstream reset/re-enumeration race on macOS.

    ``gs-usb`` 0.3.1 resets the whole USB device in ``start()``. On Darwin the
    reset re-enumerates the device, then the stale handle briefly retains
    exclusive access and the replacement handle fails with LIBUSB_ERROR_ACCESS.
    The candleLight protocol only needs bit timing followed by MODE_START.

    Linux and other platforms keep the upstream behavior unchanged.
    """

    if platform.system() != "Darwin":
        return

    import can
    import usb.util
    from can.interfaces.gs_usb import GsUsbBus
    from gs_usb.constants import (
        GS_CAN_MODE_HW_TIMESTAMP,
        GS_CAN_MODE_LISTEN_ONLY,
        GS_CAN_MODE_LOOP_BACK,
        GS_CAN_MODE_NORMAL,
        GS_CAN_MODE_ONE_SHOT,
    )
    from gs_usb.gs_usb import GsUsb, GS_CAN_MODE_START, _GS_USB_BREQ_MODE
    from gs_usb.gs_usb_structures import DeviceMode

    if getattr(GsUsb, "_gello_piper_darwin_patch", False):
        return

    def darwin_start(
        self,
        flags: int = GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP,
    ) -> None:
        # Do not call self.gs_usb.reset() on Darwin. The device has no macOS
        # kernel driver attached; a reset only creates an exclusive-handle race.
        flags &= self.device_capability.feature
        flags &= (
            GS_CAN_MODE_LISTEN_ONLY
            | GS_CAN_MODE_LOOP_BACK
            | GS_CAN_MODE_ONE_SHOT
            | GS_CAN_MODE_HW_TIMESTAMP
        )
        self.device_flags = flags
        mode = DeviceMode(GS_CAN_MODE_START, flags)
        self.gs_usb.ctrl_transfer(
            0x41,
            _GS_USB_BREQ_MODE,
            0,
            0,
            mode.pack(),
        )

    def darwin_shutdown(self) -> None:
        if self._is_shutdown:
            return
        can.BusABC.shutdown(self)
        try:
            self.gs_usb.stop()
        finally:
            # Upstream rescans and restarts the device during shutdown. That
            # repeats the same Darwin race; releasing this handle is sufficient.
            usb.util.dispose_resources(self.gs_usb.gs_usb)
            self._is_shutdown = True

    GsUsb.start = darwin_start
    GsUsbBus.shutdown = darwin_shutdown
    GsUsb._gello_piper_darwin_patch = True
