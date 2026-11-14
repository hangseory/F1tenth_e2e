"""범퍼 접촉 후진 (bump reverse) — ROS 없는 순수 로직. 2026-11-13.

규칙 (다른 팀 방식):
  1. 트리거  : 라이다 정면 ±fov/2 콘 안에서 범퍼보다 trigger_gap(1cm) 앞에
               무언가 있으면 후진. **차 양옆은 보지 않는다.**
  2. 방향    : 후진하면서 차 방향이 경로 접선과 평행해지도록 조향.
               그게 안 되면(경로/pose 없음, 그쪽 뒤가 막힘) 라이다 270° 중
               뒤쪽 좌/우에서 더 빈 쪽으로 꺾는다.
  3. 거리    : 정면 여유가 backoff_gap(10cm) 이 되면 끝 → 컨트롤러가 다시 전진.
  4. 속도    : 천천히가 아니라 reverse_speed 로 한 번에 지른다.

부호 규약 (base_link/라이다, x 앞, y 왼쪽, 각도 반시계 +):
  후진(v<0)에서 yaw_dot = v·tan(δ)/L 이므로 yaw 를 path_yaw 로 돌리려면
  δ = −k·(path_yaw − yaw). 그리고 δ>0(좌조향)으로 후진하면 **차 뒤가 왼쪽**
  으로 간다 — 그래서 좌조향 후진에는 왼쪽 뒤 여유가 필요하다.
"""
from dataclasses import dataclass, fields
import math

import numpy as np


@dataclass
class BumpReverseConfig:
    # ── 트리거 기하 (2026-11-14 실측·합의) ────────────────────────────
    #   정면 총 80°(±40°). 라이다→범퍼 정면 0.17m, 콘 끝(±40°) 방향 범퍼
    #   모서리 0.20m. 그 사이는 각도에 선형으로 잇는다 — 평평한 범퍼 모델
    #   (0.17/cosθ)과 20~30° 에서 1cm 안쪽 차이라 라이다 오차보다 작다.
    #   트리거선 = 범퍼 윤곽 + trigger_gap(1cm) → 정면 18cm, ±40° 에서 21cm.
    #   ★ 11/13 실행값은 ±20°·0.20m 였다 — 모서리 쪽을 못 보고 박았다.
    trigger_fov_deg: float = 80.0           # 정면 콘 전체 폭 (±40°)
    bumper_from_lidar: float = 0.17         # 라이다 → 앞범퍼 정면 [m] (실측)
    bumper_corner_from_lidar: float = 0.20  # 라이다 → 콘 끝 방향 범퍼 모서리 [m]
    trigger_gap: float = 0.01               # 범퍼 윤곽 앞 이만큼 안이면 후진
    trigger_min_points: float = 2.0         # 트리거선 안 점이 이만큼 있어야 (잡음 1점 거름)
    # ★ 속도 비례 선행 [s]. 트리거선을 v × lead 만큼 앞으로 민다.
    #   1cm 만으로는 달리는 중에 **닿은 뒤** 트리거된다 — 1.25㎧ 에서 1cm 는
    #   8ms 로 한 제어 주기(20ms)보다 짧다. 11/13 두 주행 모두 gap 0.5~0.9cm,
    #   속도 +1.25㎧ 에서 트리거 = 이미 충돌. 모터 제동(≈3㎧²) 정지거리
    #   v²/2a ≈ 26cm + 지연 ≈ 6cm 라 0.20s 면 1.25㎧ 에서 닿기 직전에 선다.
    #   bag 재생(11/14 00:01): 0.00~0.20 모두 오탐 0, 0.20 은 실제보다 0.25s 먼저.
    #   정지·밀착 상태(v≈0)에는 영향이 없다 — 그땐 1cm 규칙 그대로다.
    trigger_lead_time: float = 0.20
    # 이전 설정/launch 호환용. 복구 직후에도 속도 선행 감지는 끄지 않는다.
    reentry_grace_time: float = 1.0
    # ── 후진 ────────────────────────────────────────────────────────
    backoff_gap: float = 0.10         # 범퍼 윤곽 앞 여유가 이만큼 되면 후진 끝
    reverse_speed: float = 1.5        # 후진 속도 크기 [m/s]
    max_reverse_dist: float = 0.40    # 안전핀: odom 이동거리 상한 [m]
    # ★ 1.0 → 2.0s. 11/13 엔 전진 관성을 죽이는 데만 0.5s 가 걸렸고(65A 상한),
    #   남은 0.5s 동안 −0.1㎧ 로 기어 6.7cm 밖에 못 물러난 채 시간 상한으로 끝났다.
    max_reverse_time: float = 2.0
    # ★ 2.0 → 1.0, 그리고 조향 상한. 이득 2 면 heading 오차 14° 만 넘어도 풀락에
    #   붙는다 — 11/13 두 주행 모두 −27.5° 고정. 풀락 후진은 마찰이 커서 모터가
    #   65A 상한에 붙은 채 거의 못 움직였다.
    align_gain: float = 1.0           # δ = −gain·(heading 오차)
    reverse_steer_frac: float = 0.7   # 후진 조향 상한 = max_steer × 이 비율
    rear_window_deg: tuple = (90.0, 135.0)   # 뒤쪽 좌/우 창 (라이다 270° 끝)
    side_clear_min: float = 0.25      # 그쪽으로 꺾어 후진하려면 필요한 여유 [m]
    # ── 후진 정지 ──────────────────────────────────────────────────
    # 여유 + v²/(2·coast_decel) 로 정지 시작 시점을 예측한다.
    # 후진 중 HOLD 는 +reverse_speed 를 명령해 반대 방향으로 감속한다.
    # |v| < stop_speed 또는 전진 부호 전환 시 끝내고 재출발 제한으로 넘긴다.
    # coast_decel 은 예상 감속도일 뿐 실제 VESC 제동 설정이 아니다.
    coast_decel: float = 0.9          # 타력 감속도 [㎧²] (실측 0.85~1.2)
    stop_speed: float = 0.05          # |v| 이 이하면 멈춘 것으로 본다 [㎧]
    hold_max_time: float = 1.5        # 정지 대기 상한 [s]
    relaunch_speed: float = 0.8       # 복구 끝난 뒤 출발 제한 시작 속도 [㎧] (0 이면 모터 데드밴드에서 1s 정지)



# ROS 파라미터로 노출할 키 = 기본값이 float 인 필드 전부 (2026-11-14).
#   예전엔 같은 목록을 map_controller 두 곳·launch 한 곳에 손으로 복사해 두어서,
#   새 필드를 하나 빠뜨리면 race_script.yaml 값이 **조용히 안 먹었다**.
BUMP_PARAM_KEYS = tuple(f.name for f in fields(BumpReverseConfig)
                        if isinstance(f.default, float))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BumpReverse:
    """update() 를 매 제어 틱 호출. None 이면 컨트롤러 그대로, (steer, speed)
    면 그 명령으로 덮어쓴다."""

    def __init__(self, cfg: BumpReverseConfig, max_steer: float):
        self.cfg = cfg
        self.max_steer = float(max_steer)
        self.active = False
        self.mode = ''
        self._t0 = None
        self._xy0 = None
        self._steer = 0.0
        self._brake_steer = 0.0
        self.last_reason = ''
        self.count = 0
        self._ended_t = None         # 마지막 복구 종료 시각
        self.phase = ''              # 'BRAKING' | 'REVERSING' | 'HOLD' | ''
        self._hold_from_reverse = False
        self._release = None         # (시각, xy, 이유) — 후진을 놓은 순간
        self._start_info = {}        # 트리거 순간 값 (끝날 때 요약에 씀)
        self._last_xy = None
        self._last_speed = 0.0
        self._reverse_dist = 0.0
        self._reverse_t0 = None
        self.last_clear = (math.inf, 0.0, 0)   # 직전 주기 (최소 여유, 각도, 트리거선 안 점수)

    # ── 스캔 해석 ──────────────────────────────────────────────────────
    def front_gap(self, ranges, angle_min, angle_inc):
        """정면 콘 안 물체까지 **범퍼 기준** 종방향 여유 [m]. 없으면 inf."""
        r, a = self._valid(ranges, angle_min, angle_inc)
        half = math.radians(self.cfg.trigger_fov_deg) / 2
        m = np.abs(a) <= half
        if not m.any():
            return math.inf
        return float(np.min(r[m] * np.cos(a[m]))) - self.cfg.bumper_from_lidar

    def _outline(self, a):
        """각도 a[rad] 에서 라이다 → 범퍼 윤곽 거리. 정면 0.17 → 콘 끝 0.20 선형."""
        half = math.radians(self.cfg.trigger_fov_deg) / 2
        f = np.clip(np.abs(a) / max(half, 1e-6), 0.0, 1.0)
        return self.cfg.bumper_from_lidar + (
            self.cfg.bumper_corner_from_lidar - self.cfg.bumper_from_lidar) * f

    def front_clearance(self, ranges, angle_min, angle_inc):
        """정면 콘 안 각 빔의 **범퍼 윤곽까지** 여유 [m] 배열과 그 각도.

        r·cosθ(종방향)가 아니라 빔 방향 거리로 잰다 — 모서리 쪽은 범퍼가
        비스듬해 종방향으로 재면 실제보다 멀게 나온다."""
        r, a = self._valid(ranges, angle_min, angle_inc)
        half = math.radians(self.cfg.trigger_fov_deg) / 2
        m = np.abs(a) <= half
        return r[m] - self._outline(a[m]), a[m]

    def rear_clear(self, ranges, angle_min, angle_inc):
        """(왼쪽 뒤, 오른쪽 뒤) 최소거리. 빔이 없으면 inf."""
        r, a = self._valid(ranges, angle_min, angle_inc)
        lo, hi = (math.radians(v) for v in self.cfg.rear_window_deg)
        left = (a >= lo) & (a <= hi)
        right = (a <= -lo) & (a >= -hi)
        f = lambda m: float(np.min(r[m])) if m.any() else math.inf
        return f(left), f(right)

    @staticmethod
    def _valid(ranges, angle_min, angle_inc):
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(len(r)) * angle_inc
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = np.isfinite(r) & (r > 0.02)
        return r[ok], a[ok]

    # ── 조향 선택 ──────────────────────────────────────────────────────
    @property
    def _steer_cap(self):
        return self.max_steer * float(np.clip(self.cfg.reverse_steer_frac, 0.1, 1.0))

    def _path_steer(self, heading_err):
        g = -self.cfg.align_gain * heading_err
        return float(np.clip(g, -self._steer_cap, self._steer_cap))

    def choose(self, heading_err, left_clear, right_clear):
        """(mode, steer). heading_err = path_yaw − yaw (없으면 None)."""
        need = self.cfg.side_clear_min
        if heading_err is not None:
            d = self._path_steer(heading_err)
            side = left_clear if d > 0 else right_clear
            if abs(d) < 1e-3 or side >= need:
                return 'path', d
        if max(left_clear, right_clear) < need:
            return 'straight', 0.0
        return 'free_side', (self._steer_cap if left_clear >= right_clear
                             else -self._steer_cap)

    def _hold_speed(self, speed):
        # 후진 중 정지 요청은 반대 방향 속도로 제동한다. 정지/전진이면 해제.
        if self._hold_from_reverse and speed <= -self.cfg.stop_speed:
            return self.cfg.reverse_speed
        return 0.0

    # ── 메인 ───────────────────────────────────────────────────────────
    def update(self, now, scan, odom_xy, heading_err, speed=0.0, forward_steer=0.0):
        """scan = (ranges, angle_min, angle_inc) 또는 None.
        speed = 부호 있는 실측 전진 속도 [m/s] (선행 트리거용, 없으면 0)."""
        # 전진 제동 거리는 후진 거리에서 제외. 왕복/곡선에서도 누적 후진만 센다.
        v = float(speed)
        if (self.active and self.phase in ('REVERSING', 'HOLD')
                and self._last_xy is not None and odom_xy is not None
                and self._last_speed < 0.0 and v <= 0.0):
            self._reverse_dist += math.hypot(odom_xy[0] - self._last_xy[0],
                                             odom_xy[1] - self._last_xy[1])
        self._last_speed = v
        if odom_xy is not None:
            self._last_xy = tuple(odom_xy)
        if scan is None:
            if self.active:
                self._hold_from_reverse = False  # 스캔 소실은 반대속도 제동 대신 기존 정지 요청
                # 스캔이 끊겨도 굴러가는 중이면 붙잡고 끝낸다
                if self.phase in ('BRAKING', 'REVERSING') and abs(v) >= self.cfg.stop_speed:
                    self._release = (now, self._last_xy, 'scan_lost')
                    self.phase = 'HOLD'
                    return self._steer, 0.0
                if self.phase == 'HOLD' and abs(float(speed)) >= self.cfg.stop_speed:
                    return self._steer, 0.0
                return self._finish('scan_lost' if self._release is None else self._release[2],
                                    now, heading_err)
            return None
        c, ca = self.front_clearance(*scan)
        if len(c):
            i = int(np.argmin(c))
            cmin, cang = float(c[i]), float(ca[i])
        else:
            cmin, cang = math.inf, 0.0

        if not self.active:
            lead = self.cfg.trigger_lead_time * max(v, 0.0)
            thr = self.cfg.trigger_gap + lead
            n_in = int((c <= thr).sum()) if len(c) else 0
            self.last_clear = (cmin, cang, n_in)
            if n_in < max(int(round(self.cfg.trigger_min_points)), 1):
                return None
            left, right = self.rear_clear(*scan)
            self.mode, self._steer = self.choose(heading_err, left, right)
            self.active, self._t0 = True, now
            self.phase = 'BRAKING' if v > self.cfg.stop_speed else 'REVERSING'
            self._release = None
            self._hold_from_reverse = False
            self._reverse_dist = 0.0
            self._reverse_t0 = now if self.phase == 'REVERSING' else None
            self._xy0 = None if odom_xy is None else tuple(odom_xy)
            self.count += 1
            he = 'None' if heading_err is None else f'{math.degrees(heading_err):+.0f}°'
            self._start_info = dict(speed=float(speed), heading=heading_err)
            # 제동 중에는 진입 직전 전진 조향을 유지한다. 중립 강제는 코너 이탈 원인.
            self._brake_steer = float(np.clip(forward_steer, -self.max_steer, self.max_steer))
            if self.phase == 'BRAKING':
                self._steer = self._brake_steer
            # ★ 판정 근거를 한 줄에 다 싣는다 (2026-11-14) — 로그만 보고
            #   "왜 이때 / 얼마나 앞서 / 잡음은 아닌지 / 어디로" 가 보이게.
            self.last_reason = (
                f'트리거 여유 {cmin*100:.1f}cm@{math.degrees(cang):+.0f}° '
                f'(선 {thr*100:.1f}cm = {self.cfg.trigger_gap*100:.0f}cm + 선행 {lead*100:.1f}cm, '
                f'{n_in}점) 속도 {float(speed):+.2f}㎧ → {self.phase}/{self.mode} '
                f'δ={math.degrees(self._steer):+.1f}° heading오차 {he} '
                f'(뒤 좌 {left:.2f} / 우 {right:.2f}m)')
            return self._steer, -self.cfg.reverse_speed

        # 진행 중 ───────────────────────────────────────────────────────
        self.last_clear = (cmin, cang, 0)
        stopped = abs(v) < self.cfg.stop_speed

        if self.phase == 'BRAKING':
            self._steer = self._brake_steer
            if v > self.cfg.stop_speed:
                if now - self._t0 >= self.cfg.max_reverse_time:
                    self._release = (now, self._last_xy, '제동 시간 상한')
                    self.phase = 'HOLD'
                    return self._steer, 0.0
                return self._steer, -self.cfg.reverse_speed
            self.phase = 'REVERSING'
            self._reverse_t0 = now
            self._xy0 = self._last_xy

        # 후진/정지 대기 중에도 경로 오차와 뒤쪽 여유를 매번 확인한다.
        # 전진 잔속도가 다시 보이면 마지막 전진 조향을 유지한다.
        if v > self.cfg.stop_speed:
            self._steer = self._brake_steer
        else:
            self.mode, self._steer = self.choose(heading_err, *self.rear_clear(*scan))

        if self.phase == 'HOLD':
            # 후진 정지: +reverse_speed 로 감속, 정지/부호전환 시 전진 제어에 넘긴다.
            held = now - self._release[0]
            if stopped or (self._hold_from_reverse and v >= 0.0):
                return self._finish(self._release[2], now, heading_err,
                                    hold_timeout=False)
            # 시간 초과만으로 아직 움직이는 차에 전진 명령을 허가하지 않는다.
            if held >= self.cfg.hold_max_time and '정지대기 초과' not in self._release[2]:
                rt, xy, why = self._release
                self._release = (rt, xy, why + ' (정지대기 초과: 정지 명령 유지)')
            return self._steer, self._hold_speed(v)

        # REVERSING
        why = None
        coast = (v * v) / (2.0 * max(self.cfg.coast_decel, 1e-3)) if v < 0 else 0.0
        # ③ 전진 중(v > stop_speed)에는 여유로 끝내지 않는다 — 아직 브레이크 중이다
        if v <= self.cfg.stop_speed and cmin + coast >= self.cfg.backoff_gap:
            why = (f'여유 {cmin*100:.1f}cm + 관성 {coast*100:.1f}cm ≥ '
                   f'{self.cfg.backoff_gap*100:.0f}cm')
        elif now - self._reverse_t0 >= self.cfg.max_reverse_time:
            why = f'시간 상한 (여유 {cmin*100:.1f}cm)'
        elif self._reverse_dist >= self.cfg.max_reverse_dist:
            why = f'후진 거리 상한 (여유 {cmin*100:.1f}cm)'
        if why is not None:
            self._hold_from_reverse = v < 0.0
            self._release = (now, self._last_xy, why)
            if stopped:                       # 이미 섰으면 붙잡을 것 없이 끝
                return self._finish(why, now, heading_err)
            self.phase = 'HOLD'
            return self._steer, self._hold_speed(v)
        return self._steer, -self.cfg.reverse_speed

    def _finish(self, why, now=None, heading_err=None, hold_timeout=False):
        self.active = False
        self.phase = ''
        self._ended_t = now
        dist = lambda a, b: (math.hypot(a[0] - b[0], a[1] - b[1])
                             if a is not None and b is not None else float('nan'))
        dur = (now - self._t0) if (now is not None and self._t0 is not None) else float('nan')
        h0 = self._start_info.get('heading')
        hs = ('' if h0 is None or heading_err is None else
              f' heading오차 {math.degrees(h0):+.0f}°→{math.degrees(heading_err):+.0f}°')
        # 후진 1회 요약 — 미는 구간과 놓은 뒤 굴러간 구간을 따로 적는다.
        #   "관성으로 너무 밀린다" 를 숫자로 보이게 (11/14 에 1m 넘게 밀렸다).
        if self._release is not None:
            rt, rxy, _ = self._release
            push = dist(rxy, self._xy0)
            roll = dist(self._last_xy, rxy)
            tail = (f'  밀기 {push*100:.1f}cm/{rt - self._t0:.2f}s'
                    f' + 정지구간 {roll*100:.1f}cm/{(now or rt) - rt:.2f}s'
                    + (' (정지대기 시간초과)' if hold_timeout else ''))
        else:
            tail = f'  {dist(self._last_xy, self._xy0)*100:.1f}cm / {dur:.2f}s'
        self.last_reason = (f'후진 끝: {why}{tail}{hs}'
                            f' 누적후진 {self._reverse_dist*100:.1f}cm')
        self._release = None
        return None
