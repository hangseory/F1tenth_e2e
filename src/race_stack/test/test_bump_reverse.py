import math

import numpy as np

from race_stack.bump_reverse import BumpReverse, BumpReverseConfig

INC = math.radians(0.25)
AMIN = -math.radians(135)
N = int(round(math.radians(270) / INC)) + 1
ANG = AMIN + np.arange(N) * INC


def scan(front=None, left_rear=5.0, right_rear=5.0, side=None):
    r = np.full(N, 5.0)
    deg = np.degrees(ANG)
    r[(deg >= 90)] = left_rear
    r[(deg <= -90)] = right_rear
    if side is not None:      # 차 바로 옆(±60~80°)에 붙은 벽
        r[(np.abs(deg) >= 60) & (np.abs(deg) <= 80)] = side
    if front is not None:     # 정면 벽: x = front 인 평면
        m = np.abs(deg) <= 30
        r[m] = front / np.cos(ANG[m])
    return r, AMIN, INC


def make(**kw):
    return BumpReverse(BumpReverseConfig(**kw), max_steer=0.48)


def test_no_trigger_when_front_clear():
    br = make()
    assert br.update(0., scan(front=0.20 + 0.05), (0, 0), 0.) is None
    assert not br.active


def test_sides_ignored():
    br = make()
    assert br.update(0., scan(side=0.05), (0, 0), 0.) is None


def test_trigger_full_speed_reverse():
    br = make(reverse_speed=1.5)
    out = br.update(0., scan(front=0.175), (0, 0), 0.)
    assert out is not None and out[1] == -1.5
    assert br.active and br.mode == 'path' and out[0] == 0.


def test_path_alignment_sign():
    # 경로가 차보다 왼쪽(+)으로 돌아가 있음 → 후진 중 yaw 를 늘리려면 우조향(δ<0)
    br = make()
    steer, _ = br.update(0., scan(front=0.175), (0, 0), 0.3)
    assert steer < 0 and br.mode == 'path'
    # 반대
    br = make()
    steer, _ = br.update(0., scan(front=0.175), (0, 0), -0.3)
    assert steer > 0


def test_fallback_to_free_side_when_path_side_blocked():
    # 경로 정렬은 우조향(δ<0 → 뒤가 오른쪽) 인데 오른쪽 뒤가 막힘 → 왼쪽으로
    br = make()
    steer, _ = br.update(0., scan(front=0.175, right_rear=0.1), (0, 0), 0.3)
    assert br.mode == 'free_side' and steer > 0


def test_no_path_uses_free_side():
    br = make()
    steer, _ = br.update(0., scan(front=0.175, left_rear=0.3, right_rear=2.0), (0, 0), None)
    assert br.mode == 'free_side' and steer < 0


def test_both_rear_blocked_goes_straight():
    br = make()
    steer, _ = br.update(0., scan(front=0.175, left_rear=0.1, right_rear=0.1), (0, 0), None)
    assert br.mode == 'straight' and steer == 0.


def test_stops_at_backoff_gap_then_releases():
    br = make()
    assert br.update(0., scan(front=0.175), (0, 0), 0.) is not None
    assert br.update(.05, scan(front=0.25), (-.05, 0), 0.) is not None   # 5cm
    assert br.update(.10, scan(front=0.31), (-.11, 0), 0.) is None       # 11cm
    assert not br.active


def test_time_and_distance_caps():
    br = make(max_reverse_time=0.5)
    br.update(0., scan(front=0.175), (0, 0), 0.)
    assert br.update(0.6, scan(front=0.175), (0, 0), 0.) is None
    br = make(max_reverse_dist=0.3)
    br.update(0., scan(front=0.175), (0, 0), 0., speed=-0.1)
    assert br.update(0.1, scan(front=0.175), (-0.31, 0), 0.) is None


def test_scan_lost_ends_reverse():
    br = make()
    br.update(0., scan(front=0.175), (0, 0), 0.)
    assert br.update(0.02, None, (0, 0), 0.) is None and not br.active


def test_reverse_kinematics_converges_to_path_heading():
    # 자전거 모델로 실제 후진을 굴려 heading 오차가 줄어드는지 (부호 검증)
    br = make(max_reverse_time=5., max_reverse_dist=5.)
    L, dt, v = 0.33, 0.02, -1.5
    yaw, x, y = 0.0, 0.0, 0.0
    path_yaw = 0.4
    steer, _ = br.update(0., scan(front=0.175), (x, y), path_yaw - yaw)
    for k in range(1, 60):
        x += v*math.cos(yaw)*dt
        y += v*math.sin(yaw)*dt
        yaw += v*math.tan(steer)/L*dt
        steer, _ = br.update(k*dt, scan(front=0.175), (x, y), path_yaw - yaw)
    assert abs(path_yaw - yaw) < 0.1
    assert y < 0      # 우조향 후진 → 뒤(차 위치)가 오른쪽으로 감


# ── 2026-11-14 추가: 합의 기하 · 선행 · 유예 · 잡음 · 조향 상한 ─────────

def _point(deg, dist, base=5.0):
    """한 각도 근처(±0.5°)에만 물체를 둔 스캔."""
    r = np.full(N, base)
    d = np.degrees(ANG)
    r[np.abs(d - deg) <= 0.5] = dist
    return r, AMIN, INC


def test_geometry_front_18cm_edge_21cm():
    # 정면: 0.18 이하면 트리거, 0.19 면 아님
    assert make().update(0., _point(0., 0.179), (0, 0), 0.) is not None
    assert make().update(0., _point(0., 0.19), (0, 0), 0.) is None
    # 콘 끝(±38°, 윤곽 ≈0.1985): 0.205 면 트리거, 0.22 면 아님
    assert make().update(0., _point(38., 0.205), (0, 0), 0.) is not None
    assert make().update(0., _point(-38., 0.205), (0, 0), 0.) is not None
    assert make().update(0., _point(38., 0.22), (0, 0), 0.) is None


def test_outside_80deg_cone_ignored():
    # ±40° 밖(45°)은 붙어 있어도 안 본다 (차 옆)
    assert make().update(0., _point(45., 0.05), (0, 0), 0.) is None


def test_single_noise_point_ignored():
    # 0.25° 간격에서 ±0.1° 폭이면 빔 1개 — 잡음으로 보고 거른다
    r = np.full(N, 5.0)
    r[np.argmin(np.abs(np.degrees(ANG)))] = 0.10
    assert make().update(0., (r, AMIN, INC), (0, 0), 0.) is None


def test_speed_lead_triggers_earlier_only_when_moving():
    # 정면 여유 8cm: 멈춰 있으면 아님, 1.25㎧ 면 선행(25cm) 때문에 트리거
    assert make().update(0., scan(front=0.25), (0, 0), 0., speed=0.0) is None
    assert make().update(0., scan(front=0.25), (0, 0), 0., speed=1.25) is not None
    # 후진(음수 속도)엔 선행을 안 붙인다
    assert make().update(0., scan(front=0.25), (0, 0), 0., speed=-1.0) is None


def test_reentry_keeps_speed_lead_even_during_legacy_grace():
    br = make(reentry_grace_time=1.0)
    br.update(0., scan(front=0.175), (0, 0), 0.)
    assert br.update(0.1, scan(front=0.30), (-0.12, 0), 0.) is None
    # 복구 직후라도 1.25m/s 에서 여유 8cm 는 선행 제동해야 한다.
    out = br.update(0.5, scan(front=0.25), (0, 0), 0.3, speed=1.25)
    assert out == (0., -1.5) and br.phase == 'BRAKING'


def test_reverse_steer_capped():
    br = make()
    steer, _ = br.update(0., scan(front=0.175), (0, 0), 1.0)   # 오차 57°
    assert abs(steer) <= 0.7 * 0.48 + 1e-9
    br = make()
    steer, _ = br.update(0., scan(front=0.175, left_rear=2.0, right_rear=0.3), (0, 0), None)
    assert br.mode == 'free_side' and abs(steer) <= 0.7 * 0.48 + 1e-9


def test_finish_reason_reports_distance_and_time():
    br = make()
    br.update(0., scan(front=0.175), (0, 0), 0.2)
    br.update(0.4, scan(front=0.30), (-0.11, 0), 0.05)
    assert '11.0cm' in br.last_reason and '0.40s' in br.last_reason


# ── 2026-11-14 00:19 주행 뒤 추가: 관성 · 정지 대기 · 전진 중 미종료 ────────

def test_never_finishes_while_still_moving_forward():
    # 선행으로 15cm 에서 트리거 — 아직 +1.2㎧ 전진 중이면 여유가 10cm 넘어도 계속 민다
    br = make()
    assert br.update(0., scan(front=0.32), (0, 0), 0., speed=1.2) is not None
    out = br.update(0.02, scan(front=0.33), (0, 0), 0., speed=0.8)
    assert out is not None and out == (0., -1.5) and br.phase == 'BRAKING'


def test_release_counts_coast_then_holds_until_stopped():
    br = make()
    br.update(0., scan(front=0.175), (0, 0), 0.)
    # 여유 0.5cm 인데 −0.6㎧ 로 뒤로 가는 중 → 관성 20cm 예상 → 제동한다(HOLD, +1.5m/s)
    out = br.update(0.2, scan(front=0.175), (-0.05, 0), 0., speed=-0.6)
    assert out is not None and out[1] == 1.5 and br.phase == 'HOLD'
    # 아직 굴러가는 중 → 계속 반대 방향 속도로 제동한다
    assert br.update(0.4, scan(front=0.25), (-0.15, 0), 0., speed=-0.3)[1] == 1.5
    # 멈췄다 → 끝, 전진 제어에 넘김
    assert br.update(0.6, scan(front=0.29), (-0.19, 0), 0., speed=-0.01) is None
    assert not br.active and '관성' in br.last_reason


def test_hold_timeout_does_not_release_a_moving_vehicle():
    br = make(hold_max_time=0.5)
    br.update(0., scan(front=0.175), (0, 0), 0.)
    br.update(0.1, scan(front=0.175), (-0.05, 0), 0., speed=-0.6)        # HOLD 진입
    assert br.update(0.4, scan(front=0.25), (-0.1, 0), 0., speed=-0.3) is not None
    assert br.update(0.7, scan(front=0.25), (-0.1, 0), 0., speed=-0.3)[1] == 1.5
    assert br.active
    assert br.update(0.8, scan(front=0.25), (-0.11, 0), 0., speed=0.) is None
    assert '정지대기 초과' in br.last_reason


def test_full_cycle_lands_near_backoff_gap():
    """1차원 물리로 '세게 차고 일찍 놓기' 를 굴려 본다.
    밀 땐 4㎧² 로 후진, 반대속도 명령에는 3㎧² 감속을 가정한다(실차 보장 아님). 벽에 닿은 상태(여유 0)에서
    시작해 다 멈췄을 때 여유가 10cm 근처여야 한다 — 예전 로직은 1m 넘게 밀렸다."""
    br = make(coast_decel=3.0)
    dt, v, x = 0.02, 0.0, 0.0        # x = 범퍼~벽 여유 [m]
    out = br.update(0., scan(front=0.17 + x), (0, 0), 0., speed=v)
    assert out is not None
    t = 0.0
    while br.active and t < 5.0:
        t += dt
        cmd = out[1]
        if cmd < 0:                       # 밀기
            v = max(v - 4.0 * dt, cmd)
        elif cmd > 0:                     # 반대속도로 감속, 0 통과도 허용
            v = min(v + 3.0 * dt, cmd)
        elif v < 0:
            v = min(v + 0.9 * dt, 0.0)
        x += -v * dt
        out = br.update(t, scan(front=0.17 + x), (-x, 0), 0., speed=v)
        if out is None:
            break
    assert not br.active
    assert 0.08 <= x <= 0.20, f'멈춘 여유 {x*100:.1f}cm'


def test_forward_braking_distance_does_not_consume_reverse_budget():
    br = make(coast_decel=3.0)
    assert br.update(0., scan(front=.40), (0., 0.), .3, speed=1.7) == (0., -1.5)
    # 로그의 실패 형태: 제동 중 이미 40cm 이상 전진, 여전히 +1.12m/s.
    assert br.update(.28, scan(front=.175), (.45, 0.), .4, speed=1.12) == (0., -1.5)
    assert br.phase == 'BRAKING' and br._reverse_dist == 0.
    steer, cmd = br.update(.6, scan(front=.175), (.5, 0.), .3, speed=0.)
    assert cmd == -1.5 and steer < 0 and br.phase == 'REVERSING'
    br.update(.7, scan(front=.175), (.49, 0.), .2, speed=-.1)
    assert br.update(.8, scan(front=.175), (.39, 0.), .2, speed=-.1)[1] == -1.5
    assert br._reverse_dist < .12


def test_reverse_distance_accumulates_along_curve_not_start_displacement():
    br = make(max_reverse_dist=.3, coast_decel=100.)
    br.update(0., scan(front=.175), (0., 0.), 0., speed=-.1)
    br.update(.1, scan(front=.175), (-.15, 0.), 0., speed=-.1)
    br.update(.2, scan(front=.175), (-.15, .15), 0., speed=-.1)
    out = br.update(.3, scan(front=.175), (0., .15), 0., speed=-.1)
    assert out[1] == 1.5 and br.phase == 'HOLD'
    assert br._reverse_dist >= .3


def test_hold_updates_alignment_and_unwinds_at_target():
    br = make()
    br.update(0., scan(front=.175), (0., 0.), .3)
    br.update(.2, scan(front=.175), (-.05, 0.), .3, speed=-.6)
    assert br.phase == 'HOLD'
    steer, speed = br.update(.3, scan(front=.25), (-.1, 0.), .0, speed=-.3)
    assert steer == 0. and speed == 1.5
    steer, speed = br.update(.4, scan(front=.25), (-.12, 0.), -.1, speed=-.2)
    assert steer > 0. and speed == 1.5


def test_free_side_rechecks_rear_space_and_returns_to_path():
    br = make()
    br.update(0., scan(front=.175, right_rear=.1), (0., 0.), .3)
    assert br.mode == 'free_side'
    steer, _ = br.update(.1, scan(front=.175, left_rear=.1), (-.01, 0.), .2, speed=-.1)
    assert br.mode == 'path' and steer < 0.


def test_braking_has_its_own_timeout_and_scan_loss_stops_command():
    br = make(max_reverse_time=.5)
    br.update(0., scan(front=.25), (0., 0.), .3, speed=1.5)
    assert br.update(.6, scan(front=.175), (.2, 0.), .3, speed=.5) == (0., 0.)
    assert br.phase == 'HOLD'
    br = make()
    br.update(0., scan(front=.25), (0., 0.), .3, speed=1.5)
    assert br.update(.1, None, (.1, 0.), .3, speed=1.) == (0., 0.)
    assert br.phase == 'HOLD'


def test_reverse_gets_full_time_budget_after_braking():
    br = make(max_reverse_time=.5)
    br.update(0., scan(front=.25), (0., 0.), .3, speed=1.5)
    br.update(.4, scan(front=.175), (.2, 0.), .3, speed=0.)
    assert br.update(.6, scan(front=.175), (.19, 0.), .3, speed=-.1)[1] == -1.5
    assert br.phase == 'REVERSING'


def test_corner_braking_preserves_forward_steer_until_reverse():
    # 02:25 주행: 좌 15도, +1.3m/s 에서 재감지 시 중립으로 펴던 회귀.
    br = make()
    steer = math.radians(15.)
    assert br.update(0., scan(front=.40), (0., 0.), .4,
                     speed=1.3, forward_steer=steer) == (steer, -1.5)
    assert br.update(.2, scan(front=.25), (.2, 0.), .4,
                     speed=.8, forward_steer=0.) == (steer, -1.5)
    # 정지에 도달한 뒤에만 후진 정렬 조향으로 전환.
    out = br.update(.4, scan(front=.175), (.3, 0.), .4, speed=0.)
    assert out[0] < 0. and out[1] == -1.5


def test_scan_loss_timeout_never_releases_while_rolling():
    br = make(hold_max_time=.1)
    br.update(0., scan(front=.175), (0., 0.), .2)
    assert br.update(.1, None, (-.05, 0.), .2, speed=-.6)[1] == 0.
    assert br.update(1., None, (-.2, 0.), .2, speed=-.2)[1] == 0.
    assert br.active


def test_reverse_stop_command_is_opposite_and_releases_on_sign_crossing():
    br = make(reverse_speed=1.2)
    br.update(0., scan(front=.175), (0., 0.), .2)
    assert br.update(.2, scan(front=.175), (-.05, 0.), .2, speed=-.6)[1] == 1.2
    assert br.update(.3, scan(front=.25), (-.10, 0.), .1, speed=-.15)[1] == 1.2
    # 샘플 사이에 0을 지나도 반대속도를 계속 걸어 급출발하지 않는다.
    assert br.update(.4, scan(front=.28), (-.11, 0.), .05, speed=.10) is None
    assert not br.active


def test_scan_loss_cancels_reverse_counter_command_even_after_scan_returns():
    br = make()
    br.update(0., scan(front=.175), (0., 0.), .2)
    assert br.update(.2, scan(front=.175), (-.05, 0.), .2, speed=-.6)[1] == 1.5
    assert br.update(.3, None, (-.10, 0.), .2, speed=-.4)[1] == 0.
    assert br.update(.4, scan(front=.25), (-.11, 0.), .2, speed=-.2)[1] == 0.
