"""Read the same generated steering limits as the vehicle controllers."""

import math
import os
from pathlib import Path
import warnings

import yaml


def load_steering_geometry(fallback_wheelbase=0.33, fallback_max_steer=0.36):
    """Return (wheelbase, max_steer) from the active vehicle overlay.

    race.sh supplies RACE_VEHICLE_OVERLAY. Resolve source symlinks as well
    for direct Python/ROS entry points; RACE_WS is not the config repository.
    An explicitly selected or malformed overlay must not silently fall back.
    """
    selected = os.environ.get('RACE_VEHICLE_OVERLAY')
    if selected:
        candidates = [Path(selected).expanduser()]
    else:
        candidates = []
        if os.environ.get('RACE_REPO'):
            candidates.append(Path(os.environ['RACE_REPO']).expanduser()
                              / 'config' / '.vehicle_overlay.yaml')
        if os.environ.get('RACE_SCRIPT'):
            candidates.append(Path(os.environ['RACE_SCRIPT']).expanduser().resolve()
                              .parent / 'config' / '.vehicle_overlay.yaml')
        candidates.extend(parent / 'config' / '.vehicle_overlay.yaml'
                          for parent in Path(__file__).resolve().parents)

    for path in candidates:
        if not path.is_file():
            if selected:
                raise FileNotFoundError(f'차량 설정을 찾을 수 없습니다: {path}')
            continue
        with path.open() as stream:
            config = yaml.safe_load(stream)['pure_pursuit']
        wheelbase = float(config['wheelbase'])
        max_steer = float(config['max_steer'])
        if (not math.isfinite(wheelbase) or wheelbase <= 0
                or not math.isfinite(max_steer) or not 0 < max_steer < math.pi / 2):
            raise ValueError(f'잘못된 차량 조향 설정: {path}')
        return wheelbase, max_steer

    warnings.warn('차량 overlay가 없어 분석용 기본 조향값을 사용합니다. '
                  '실차에서는 apply_vehicle.py 실행 후 다시 생성하세요.',
                  RuntimeWarning, stacklevel=2)
    return fallback_wheelbase, fallback_max_steer
