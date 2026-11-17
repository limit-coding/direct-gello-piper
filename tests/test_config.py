from pathlib import Path

import pytest
import yaml

from gello_piper_direct.config import (
    AppConfig,
    PiperConfig,
    TeleopConfig,
    load_config,
    load_dual_config,
)
from gello_piper_direct.errors import ConfigurationError


def test_example_config_loads() -> None:
    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "config.example.yaml")
    assert config.piper.interface == "gs_usb"
    assert config.piper.bitrate == 1_000_000
    assert config.gello.joint_ids == (1, 6, 4, 5, 3, 2)


def test_uncalibrated_null_joint_ranges_loads(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    raw = yaml.safe_load((root / "dual_config.yaml").read_text(encoding="utf-8"))
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(raw["piper_b"]), encoding="utf-8")
    config = load_config(config_path)
    assert config.gello.joint_ranges_rad is None
    assert config.gello.logical_id_offset == 7
    assert not config.teleop.range_mapping


def test_range_mapping_requires_calibrated_ranges() -> None:
    config = AppConfig(teleop=TeleopConfig(range_mapping=True))
    with pytest.raises(ConfigurationError, match="joint_ranges_rad"):
        config.validate()


def test_invalid_direction_fails_closed() -> None:
    config = AppConfig(teleop=TeleopConfig(joint_directions=(1, 1, 0, 1, 1, 1)))
    with pytest.raises(ConfigurationError, match="joint_directions"):
        config.validate()


def test_non_piper_bitrate_is_rejected() -> None:
    config = AppConfig()
    config.piper.bitrate = 500_000
    with pytest.raises(ConfigurationError, match="1000000"):
        config.validate()


def test_large_piper_gripper_accepts_100mm_and_rejects_more() -> None:
    PiperConfig(gripper_max_m=0.10).validate()
    with pytest.raises(ConfigurationError, match=r"0\.10"):
        PiperConfig(gripper_max_m=0.101).validate()


def test_logical_id_offset_validates_physical_mapping() -> None:
    config = AppConfig()
    config.gello.joint_ids = (8, 9, 10, 11, 12, 13)
    config.gello.gripper_config = (14, 10.0, 20.0)
    config.gello.logical_id_offset = 7
    config.validate()

    config.gello.joint_ids = (8, 10, 9, 11, 12, 13)
    with pytest.raises(ConfigurationError, match="logical_id_offset"):
        config.validate()


def test_dual_config_loads_disjoint_gello_and_can_serials() -> None:
    root = Path(__file__).resolve().parents[1]
    config = load_dual_config(root / "dual_config.yaml")
    assert config.piper_b.gello.joint_ids == (8, 9, 10, 11, 12, 13)
    assert config.piper_b.gello.gripper_config[0] == 14
    assert set(config.piper_c.gello.joint_ids) == {1, 2, 3, 4, 5, 6}
    assert config.piper_c.gello.gripper_config[0] == 7
    assert (
        config.piper_b.piper.serial_number
        != config.piper_c.piper.serial_number
    )

