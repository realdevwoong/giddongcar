"""vision_drive --preset: 설정 파일만으로는 로봇이 움직이지 않아야 한다."""
from pathlib import Path

import pytest

from pinky_fleet.vision_drive import parse_args

PRESET = Path(__file__).resolve().parents[1] / 'config' / 'vision_drive_supervised.yaml'


def test_shipped_preset_loads_without_motion_permission():
    args = parse_args(['--robot-ip', '192.0.2.1', '--preset', str(PRESET)])
    assert args.mode == 'drive'
    assert args.stop_distance == pytest.approx(0.35)
    assert not args.enable_motion
    assert not args.confirm_supervised_test
    assert not args.watchdog_verified
    assert not args.confirm_attended_test_without_watchdog


def test_command_line_overrides_preset():
    args = parse_args(['--robot-ip', '192.0.2.1', '--preset', str(PRESET),
                       '--mode', 'observe', '--max-linear=0.03', '--enable-motion'])
    assert args.mode == 'observe'
    assert args.max_linear == pytest.approx(0.03)
    assert args.enable_motion


@pytest.mark.parametrize('key', [
    'robot_ip', 'enable_motion', 'confirm_supervised_test',
    'watchdog_verified', 'confirm_attended_test_without_watchdog',
])
def test_preset_rejects_protected_keys(tmp_path, key):
    preset = tmp_path / 'preset.yaml'
    preset.write_text(f'model: /tmp/model.pt\n{key}: true\n', encoding='utf-8')
    with pytest.raises(SystemExit):
        parse_args(['--robot-ip', '192.0.2.1', '--preset', str(preset)])


def test_preset_rejects_unknown_key(tmp_path):
    preset = tmp_path / 'preset.yaml'
    preset.write_text('model: /tmp/model.pt\nmax_speed: 1.0\n', encoding='utf-8')
    with pytest.raises(SystemExit):
        parse_args(['--robot-ip', '192.0.2.1', '--preset', str(preset)])
