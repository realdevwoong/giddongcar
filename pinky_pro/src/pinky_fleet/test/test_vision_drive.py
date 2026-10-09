"""vision_drive: preset 안전 규칙과 조향용 차선 mask."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from pinky_fleet.vision_drive import _lane_error, _lane_mask, _mask_for_class, parse_args

PRESET = Path(__file__).resolve().parents[1] / 'config' / 'vision_drive_supervised.yaml'
LANE_ARGS = SimpleNamespace(driveable_class='driveable_area', crosswalk_class='crosswalk')


class _Tensor:
    def __init__(self, array):
        self.array = array

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.array


def _result(*instances):
    """Ultralytics 결과 흉내: (class_id, 240x320 mask) 목록."""
    return SimpleNamespace(
        names={0: 'driveable_area', 1: 'crosswalk'},
        boxes=[SimpleNamespace(cls=SimpleNamespace(item=lambda c=c: c)) for c, _ in instances],
        masks=SimpleNamespace(data=[_Tensor(m.astype(np.float32)) for _, m in instances]))


def _rect(top, bottom, left=80, right=240):
    mask = np.zeros((240, 320), np.uint8)
    mask[top:bottom, left:right] = 1
    return mask


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


def test_lane_continues_over_crosswalk():
    # 횡단보도 바로 앞: 학습 결과 주행 영역은 횡단보도 부분이 비어 있다.
    result = _result((0, _rect(100, 140)), (1, _rect(140, 240)))
    assert _lane_error(_mask_for_class(result, 'driveable_area', (240, 320))) is None
    assert _lane_error(_lane_mask(result, LANE_ARGS, (240, 320))) == pytest.approx(0.0, abs=0.01)


def test_lane_mask_without_crosswalk_is_driveable_area():
    lane = _rect(100, 240)
    mask = _lane_mask(_result((0, lane)), LANE_ARGS, (240, 320))
    assert np.array_equal(mask, lane)


def test_lane_mask_uses_crosswalk_when_driveable_missing():
    crosswalk = _rect(120, 240)
    assert np.array_equal(_lane_mask(_result((1, crosswalk)), LANE_ARGS, (240, 320)), crosswalk)
    assert _lane_mask(_result(), LANE_ARGS, (240, 320)) is None
