import numpy as np
import pytest
import yaml

from gello_piper_direct.calibration import (
    _save_dual_range_calibration,
    _save_range_calibration,
    _select_seven_id_block,
    wrapped_delta,
)
from gello_piper_direct.config import load_config, load_dual_config


def test_selects_physical_ids_8_through_14() -> None:
    assert _select_seven_id_block((8, 9, 10, 11, 12, 13, 14)) == tuple(
        range(8, 15)
    )


def test_calibration_delta_unwraps_360_boundary() -> None:
    delta = wrapped_delta(np.deg2rad(2.0), np.deg2rad(358.0))
    assert np.rad2deg(delta) == pytest.approx(4.0)


def test_range_calibration_preserves_ids_signs_and_gripper(tmp_path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        """
gello:
  joint_ids: [8, 9, 10, 11, 12, 13]
  logical_id_offset: 7
  joint_offsets: [0, 0, 0, 0, 0, 0]
  joint_signs: [1, -1, -1, -1, 1, 1]
  gripper_config: [14, 194.2, 146.162]
teleop:
  range_mapping: false
""".strip()
        + "\n",
        encoding="utf-8",
    )
    before = yaml.safe_load(path.read_text(encoding="utf-8"))["gello"]
    ranges = tuple((-0.5, 0.5) for _ in range(6))

    backup = _save_range_calibration(path, np.arange(6, dtype=float), ranges)

    after_yaml = yaml.safe_load(path.read_text(encoding="utf-8"))["gello"]
    after = load_config(path)
    assert after_yaml["joint_ids"] == before["joint_ids"]
    assert after_yaml["joint_signs"] == before["joint_signs"]
    assert after_yaml["gripper_config"] == before["gripper_config"]
    assert after.gello.joint_ranges_rad == ranges
    assert after.teleop.range_mapping
    assert backup.exists()


def test_dual_range_calibration_only_updates_selected_arm(tmp_path) -> None:
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    source = root / "dual_config.yaml"
    path = tmp_path / "dual_config.yaml"
    path.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    before = yaml.safe_load(path.read_text(encoding="utf-8"))
    ranges = tuple((-0.6, 0.6) for _ in range(6))

    backup = _save_dual_range_calibration(
        path, "B", np.arange(6, dtype=float), ranges
    )

    after_yaml = yaml.safe_load(path.read_text(encoding="utf-8"))
    after = load_dual_config(path)
    assert after_yaml["piper_c"] == before["piper_c"]
    assert after.piper_b.gello.joint_ranges_rad == ranges
    assert after.piper_b.teleop.range_mapping
    assert after.piper_c.teleop.range_mapping == bool(
        before["piper_c"]["teleop"]["range_mapping"]
    )
    assert backup.exists()
