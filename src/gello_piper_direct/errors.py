"""Library-specific exceptions."""


class DirectControlError(RuntimeError):
    """Base error for direct control failures."""


class ConfigurationError(DirectControlError):
    """Raised when configuration is invalid or ambiguous."""


class DeviceConnectionError(DirectControlError):
    """Raised when a USB/CAN device cannot be opened or stops responding."""


class SafetyError(DirectControlError):
    """Raised when a command violates a safety constraint."""


class PiperControlModeError(SafetyError):
    """Raised when Piper leaves the expected CAN/MOVE_J follower mode."""
