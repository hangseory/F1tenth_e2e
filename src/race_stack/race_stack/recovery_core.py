"""Bounded map-assisted reverse recovery. No ROS, publication, or hardware.

The map constrains static space; it is not a substitute for a rear sensor for
new/dynamic obstacles. The caller owns localization and observation freshness,
operator permission, and the final motor/watchdog path.
"""
from dataclasses import dataclass, field, fields
import math

import numpy as np

from .collision_guard_core import GuardConfig


@dataclass(frozen=True)
class RecoveryConfig:
    blocked_wait: float = 1.
    reverse_speed: float = .25
    reverse_distance: float = .35
    max_reverse_time: float = 3.
    stop_hold: float = .3
    stopped_speed: float = .05
    min_progress: float = .03
    progress_timeout: float = 1.
    max_attempts: int = 2
    reset_forward_distance: float = 1.
    sample_step: float = .02
    forward_probe: float = 1.
    min_escape_gain: float = .05
    occupancy_threshold: int = 65

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid recovery config: ' + item.name)
        if int(self.max_attempts) != self.max_attempts or not 1 <= self.occupancy_threshold <= 100:
            raise ValueError('invalid attempts/occupancy threshold')
        if self.reverse_speed > .5 or self.reverse_distance > 1.:
            raise ValueError('recovery must remain a short low-speed maneuver')


@dataclass(frozen=True)
class RecoveryResult:
    override: bool
    speed: float
    steering: float
    state: str
    reason: str
    details: dict = field(default_factory=dict)


def _pose(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('invalid SE2 pose')
    return value


def _points(value):
    value = np.asarray(value, dtype=float)
    if value.size == 0:
        return np.empty((0, 2))
    if value.ndim != 2 or value.shape[1] != 2 or not np.isfinite(value).all():
        raise ValueError('invalid obstacle points')
    return value


def _rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _compose(a, b):
    xy = a[:2] + _rotation(a[2]) @ b[:2]
    return np.array([xy[0], xy[1], a[2]+b[2]])


def _arc(distance, steering, wheelbase):
    k = math.tan(steering)/wheelbase
    angle = distance*k
    if abs(k) < 1e-10:
        return np.array([distance, 0., angle])
    return np.array([math.sin(angle)/k, (1-math.cos(angle))/k, angle])


class OccupancyGrid:
    def __init__(self, data, resolution, origin, occupancy_threshold=65):
        a = np.asarray(data)
        if (a.ndim != 2 or a.size == 0 or not np.issubdtype(a.dtype, np.number)
                or not np.isfinite(a).all() or (a < -1).any() or (a > 100).any()
                or not math.isfinite(resolution) or resolution <= 0
                or not math.isfinite(occupancy_threshold) or not 1 <= occupancy_threshold <= 100):
            raise ValueError('invalid occupancy grid')
        self.data = a.copy()
        self.resolution = float(resolution)
        self.origin = _pose(origin)
        self.occupancy_threshold = occupancy_threshold
        self.blocked = (a < 0) | (a >= occupancy_threshold)
        self.occupied = a >= occupancy_threshold
        self.height, self.width = a.shape
        self._rotation = _rotation(self.origin[2])
        self.cell_padding = math.sqrt(2)*self.resolution/2

    def _grid_xy(self, world):
        return (_points(world)-self.origin[:2]) @ self._rotation / self.resolution

    def contains(self, world):
        xy = self._grid_xy(world)
        return ((xy[:, 0] >= 0) & (xy[:, 0] < self.width)
                & (xy[:, 1] >= 0) & (xy[:, 1] < self.height))

    def obstacles_near(self, center, radius):
        # Rotation preserves radius; square ROI in grid coordinates contains
        # every occupied/unknown cell which could intersect the local scene.
        c = self._grid_xy(np.asarray(center).reshape(1, 2))[0]
        n = radius/self.resolution + 2
        lo = np.maximum(np.floor(c-n).astype(int), [0, 0])
        hi = np.minimum(np.ceil(c+n).astype(int)+1, [self.width, self.height])
        if (hi <= lo).any():
            return np.empty((0, 2))
        ij = np.argwhere(self.blocked[lo[1]:hi[1], lo[0]:hi[0]])
        xy = (ij[:, ::-1]+lo+.5)*self.resolution
        return xy @ self._rotation.T + self.origin[:2]

    def matches(self, world_points, tolerance=.15):
        """Whether points lie near known occupied cells (unknown never matches)."""
        if not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError('invalid match tolerance')
        p = _points(world_points)
        xy = self._grid_xy(p)
        result = np.zeros(len(p), dtype=bool)
        n = int(math.ceil(tolerance/self.resolution)) + 1
        inside = ((xy[:, 0] >= 0) & (xy[:, 0] < self.width)
                  & (xy[:, 1] >= 0) & (xy[:, 1] < self.height))
        selected = np.flatnonzero(inside)
        if n <= 64:
            # Broadcast points against a small cell neighborhood; avoid a
            # Python loop for each scan return. Chunk only for bounded memory.
            offsets = np.stack(np.meshgrid(np.arange(-n, n+1), np.arange(-n, n+1)), axis=-1).reshape(-1, 2)
            chunk = max(1, 400000//len(offsets))
            for first in range(0, len(selected), chunk):
                indices = selected[first:first+chunk]
                q = xy[indices, None, :]
                cells = np.floor(q).astype(np.int64)+offsets[None, :, :]
                cx, cy = cells[..., 0], cells[..., 1]
                valid = (cx >= 0) & (cx < self.width) & (cy >= 0) & (cy < self.height)
                occupied = self.occupied[np.clip(cy, 0, self.height-1), np.clip(cx, 0, self.width-1)]
                delta = np.maximum(np.abs(cells+.5-q)-.5, 0.)*self.resolution
                close = np.sum(delta*delta, axis=-1) <= tolerance*tolerance
                result[indices] = np.any(valid & occupied & close, axis=1)
            return result
        # Extremely fine maps/large tolerances would create huge broadcast
        # arrays. Crop each query to the finite grid in that unusual case.
        for i in selected:
            q = xy[i]
            lo = np.maximum(np.floor(q).astype(int)-n, [0, 0])
            hi = np.minimum(np.floor(q).astype(int)+n+1, [self.width, self.height])
            ij = np.argwhere(self.occupied[lo[1]:hi[1], lo[0]:hi[0]])
            if not len(ij):
                continue
            delta = np.maximum(np.abs(ij[:, ::-1]+lo+.5-q)-.5, 0.)*self.resolution
            result[i] = np.any(np.linalg.norm(delta, axis=1) <= tolerance)
        return result


class _Scene:
    def __init__(self, grid, world_pose, points, footprint, radius):
        self.grid, self.world_pose, self.cfg = grid, world_pose, footprint
        map_points = (grid.obstacles_near(world_pose[:2], radius)-world_pose[:2]) @ _rotation(world_pose[2])
        self.layers = [(map_points, grid.cell_padding), (points, 0.)]
        self.allowances = []
        for p, cell in self.layers:
            separation = self._separation(p)
            allowed = ((p[:, 0] > footprint.front+cell)
                       & (np.abs(p[:, 1]) <= footprint.half_width+footprint.margin+cell)
                       & (separation <= footprint.margin+cell))
            self.allowances.append((allowed, separation))

    def _separation(self, points, padding=(0., 0.)):
        cfg = self.cfg
        px, py = padding
        return np.maximum.reduce([points[:, 0]-cfg.front-px,
                                  -cfg.rear-points[:, 0]-px,
                                  np.abs(points[:, 1])-cfg.half_width-py])

    def clear(self, local_pose, padding=0.):
        cfg = self.cfg
        world = _compose(self.world_pose, local_pose)
        px, py = np.broadcast_to(np.asarray(padding, dtype=float), (2,))
        # Unknown/outside-map is blocked, including margins and sweep padding.
        mx, my = cfg.margin+px, cfg.margin+py
        corners = np.array([[-cfg.rear-mx, -cfg.half_width-my],
                            [-cfg.rear-mx, cfg.half_width+my],
                            [cfg.front+mx, -cfg.half_width-my],
                            [cfg.front+mx, cfg.half_width+my]])
        if not self.grid.contains(corners @ _rotation(world[2]).T+world[:2]).all():
            return False
        for (points, cell), (allowed, initial_sep) in zip(self.layers, self.allowances):
            local = (points-local_pose[:2]) @ _rotation(local_pose[2])
            sep = self._separation(local, (px, py))
            if np.any(sep <= cell):
                return False  # Physical body collision; never an escape allowance.
            collision = sep <= cfg.margin+cell
            retreating_front = allowed & (sep >= initial_sep-1e-9)
            if np.any(collision & ~retreating_front):
                return False
        return True


class RecoveryController:
    OBSTACLE_REASONS = {'footprint_obstacle', 'stopping_distance_obstacle', 'obstacle_latch'}

    def __init__(self, guard_config=None, config=None):
        self.guard = guard_config if guard_config is not None else GuardConfig()
        self.config = config if config is not None else RecoveryConfig()
        if not isinstance(self.guard, GuardConfig) or not isinstance(self.config, RecoveryConfig):
            raise ValueError('invalid recovery configuration')
        self.state = 'IDLE'
        self._blocked_since = self._last_now = None
        self._last_map = self._last_odom = None
        self._attempts = 0
        self._forward_distance = 0.
        self._steering = 0.
        self._started = self._hold_since = self._progress_time = None
        self._start_map = self._start_odom = None
        self._wheel_progress = self._checkpoint = 0.
        self._lock_reason = ''
        self._plan_details = {}

    def _result(self, override=False, speed=0., reason='', **details):
        return RecoveryResult(override, speed, self._steering, self.state, reason,
                              dict(self._plan_details, attempts=self._attempts, **details))

    def _lock(self, reason):
        self.state, self._lock_reason = 'LOCKED', reason
        self._blocked_since = None
        return self._result(True, reason=reason)

    def _scene(self, grid, pose, points):
        cfg, g = self.config, self.guard
        radius = (cfg.reverse_distance + cfg.forward_probe + g.front+g.rear
                  + g.margin + self._stop_extension() + grid.cell_padding + .2)
        return _Scene(grid, pose, points, g, radius)

    def _stop_extension(self):
        v = self.config.reverse_speed
        return v*self.guard.reaction_time+v*v/(2*self.guard.brake_decel)

    def _path_clearance(self, scene, start, distance, steering):
        if not scene.clear(start):
            return 0.
        length = abs(distance)
        if length <= 1e-9:
            return length
        n = max(1, int(math.ceil(length/self.config.sample_step)))
        step = distance/n
        k = abs(math.tan(steering)/self.guard.wheelbase)
        radius = math.hypot(max(self.guard.front, self.guard.rear)+self.guard.margin+scene.grid.cell_padding,
                            self.guard.half_width+self.guard.margin+scene.grid.cell_padding)
        # With no rotation, translation is exactly longitudinal. Keeping its
        # lateral padding zero avoids artificial new side collisions while
        # retreating from a front wall that already touches the margin.
        padding = ((abs(step)/2, 0.) if k < 1e-10
                   else (1+k*radius)*abs(step)/2)
        for i in range(n):
            pose = _compose(start, _arc((i+.5)*step, steering, self.guard.wheelbase))
            if not scene.clear(pose, padding):
                return i*abs(step)
        return length

    def _forward_score(self, scene, start, requested_steering):
        options = (requested_steering, 0., -self.guard.max_steer, self.guard.max_steer)
        return max(self._path_clearance(scene, start, self.config.forward_probe, s) for s in options)

    def _plan(self, scene, requested_steering):
        zero = np.zeros(3)
        baseline = self._forward_score(scene, zero, requested_steering)
        distance = self.config.reverse_distance
        checked_distance = distance+self._stop_extension()
        candidates = []
        for steer in (0., -self.guard.max_steer, self.guard.max_steer):
            available = self._path_clearance(scene, zero, -checked_distance, steer)
            if available < checked_distance-1e-8:
                continue
            end = _arc(-distance, steer, self.guard.wheelbase)
            escape = self._forward_score(scene, end, requested_steering)
            if escape-baseline < self.config.min_escape_gain:
                continue
            # Prefer straighter reverse when forward clearance is essentially tied.
            score = escape-.02*abs(steer)/self.guard.max_steer
            candidates.append((score, steer, escape))
        if not candidates:
            return None
        score, steer, escape = max(candidates)
        return steer, {'reverse_distance': distance, 'checked_reverse_distance': checked_distance,
                       'baseline_forward_clearance': baseline, 'escape_forward_clearance': escape,
                       'selected_steering': steer, 'candidate_count': len(candidates)}

    def update(self, grid, pose_map, obstacle_points_base, requested_speed,
               requested_steering, actual_speed, now, guard_state, guard_reason,
               healthy=True, permitted=True, odom_pose=None):
        cfg = self.config
        active = self.state in ('REVERSING', 'STOP_HOLD')
        try:
            pose = _pose(pose_map)
            odom = _pose(odom_pose) if odom_pose is not None else None
            points = _points(obstacle_points_base)
            if not isinstance(grid, OccupancyGrid):
                raise ValueError('missing map')
            if not np.isfinite([requested_speed, requested_steering, actual_speed, now]).all():
                raise ValueError('nonfinite motion/time')
        except (ValueError, TypeError, OverflowError) as exc:
            # ★ **무엇이** 잘못됐는지 남긴다 (2026-11-13). 판정은 그대로다.
            #   예전엔 예외 메시지('missing map', 'nonfinite motion/time', pose
            #   해석 실패 …)를 버려서, 시험 한 번에 invalid_recovery_input 으로
            #   18번 잠겼는데 원인이 지도인지 위치인지 시간인지 알 수 없었다.
            #   details['input_error'] 로 실어 가드 상태 JSON·로그 줄에 나온다.
            err = f'{type(exc).__name__}: {exc}'
            if active:
                res = self._lock('invalid_recovery_input')
                res.details['input_error'] = err
                return res
            self._blocked_since = None
            if self.state == 'WAIT_BLOCKED':
                self.state = 'IDLE'
            return self._result(self.state == 'LOCKED' and bool(permitted),
                                reason='invalid_recovery_input', input_error=err)
        if self._last_now is not None and now < self._last_now:
            return self._lock('nonmonotonic_time')
        dt = min(.2, max(0., now-self._last_now)) if self._last_now is not None else 0.
        map_step = float(np.linalg.norm(pose[:2]-self._last_map[:2])) if self._last_map is not None else 0.
        odom_step = (float(np.linalg.norm(odom[:2]-self._last_odom[:2]))
                     if odom is not None and self._last_odom is not None else map_step)
        yaw_step = abs(_wrap(pose[2]-self._last_map[2])) if self._last_map is not None else 0.
        jump = (map_step > abs(actual_speed)*dt+.15 or yaw_step > .35
                or (odom is not None and abs(map_step-odom_step) > .15))
        self._last_now, self._last_map = now, pose.copy()
        self._last_odom = odom.copy() if odom is not None else None
        if active and jump:
            return self._lock('localization_jump')
        if jump:
            self._blocked_since = None
        if healthy and not jump and actual_speed > cfg.stopped_speed:
            self._forward_distance += min(actual_speed*dt, map_step, odom_step)
            if self._forward_distance >= cfg.reset_forward_distance:
                self._attempts = 0
                if self.state == 'LOCKED':
                    self.state, self._lock_reason = 'IDLE', ''
        elif actual_speed < -cfg.stopped_speed:
            self._forward_distance = 0.
        if not permitted or not healthy or requested_speed <= 0:
            self._blocked_since = None
            if active:
                return self._lock('recovery_cancelled')
            if self.state == 'LOCKED':
                return self._result(bool(permitted and requested_speed > 0), reason=self._lock_reason)
            self.state = 'IDLE'
            return self._result(reason='not_permitted' if not permitted else 'unhealthy' if not healthy else 'no_forward_request')
        if self.state == 'LOCKED':
            # No candidate/attempt exhaustion disables further automatic
            # reversals. A separately cleared forward guard may still let an
            # obstacle that moved away be passed. Fault locks stay stopped.
            if (self._lock_reason in ('no_safe_reverse_candidate', 'recovery_attempt_limit')
                    and guard_state in ('CLEAR', 'LIMITED', 'RESTARTING')):
                return self._result(reason=self._lock_reason)
            return self._result(True, reason=self._lock_reason)
        if self.state == 'STOP_HOLD':
            if abs(actual_speed) > cfg.stopped_speed:
                self._hold_since = now
            if now-self._hold_since < cfg.stop_hold:
                return self._result(True, reason='post_reverse_stop')
            self.state, self._blocked_since = 'IDLE', None
            return self._result(reason='recovery_complete', completed=True)
        scene = self._scene(grid, pose, points)
        if self.state == 'REVERSING':
            self._wheel_progress += max(0., -actual_speed)*dt
            map_progress = float(np.linalg.norm(pose[:2]-self._start_map[:2]))
            odom_progress = (float(np.linalg.norm(odom[:2]-self._start_odom[:2]))
                             if odom is not None and self._start_odom is not None else map_progress)
            progress = min(self._wheel_progress, map_progress, odom_progress)
            elapsed = now-self._started
            if actual_speed > cfg.stopped_speed:
                return self._lock('unexpected_forward_motion')
            if actual_speed < -cfg.reverse_speed*1.2:
                return self._lock('reverse_overspeed')
            if elapsed >= cfg.max_reverse_time:
                return self._lock('reverse_timeout')
            if progress-self._checkpoint >= cfg.min_progress:
                self._checkpoint, self._progress_time = progress, now
            if now-self._progress_time >= cfg.progress_timeout:
                return self._lock('reverse_no_progress')
            if max(self._wheel_progress, map_progress, odom_progress) >= cfg.reverse_distance:
                self.state, self._hold_since = 'STOP_HOLD', now
                return self._result(True, reason='reverse_distance_reached', progress=progress)
            v = max(cfg.reverse_speed, abs(actual_speed))
            stopping = v*self.guard.reaction_time+v*v/(2*self.guard.brake_decel)
            remaining = cfg.reverse_distance-progress+stopping
            if self._path_clearance(scene, np.zeros(3), -remaining, self._steering) < remaining-1e-8:
                return self._lock('reverse_corridor_blocked')
            return self._result(True, -cfg.reverse_speed, reason='reverse_recovery', progress=progress, elapsed=elapsed)
        blocked = guard_state == 'STOPPED' and guard_reason in self.OBSTACLE_REASONS
        if not blocked or abs(actual_speed) > cfg.stopped_speed:
            self.state, self._blocked_since = 'IDLE', None
            return self._result()
        self.state = 'WAIT_BLOCKED'
        if self._blocked_since is None:
            self._blocked_since = now
        if now-self._blocked_since < cfg.blocked_wait:
            return self._result(reason='waiting_stationary_obstacle')
        if self._attempts >= cfg.max_attempts:
            return self._lock('recovery_attempt_limit')
        plan = self._plan(scene, float(np.clip(requested_steering, -self.guard.max_steer, self.guard.max_steer)))
        self._attempts += 1
        self._forward_distance = 0.
        if plan is None:
            return self._lock('no_safe_reverse_candidate')
        self._steering, self._plan_details = plan
        self.state, self._started, self._progress_time = 'REVERSING', now, now
        self._start_map = pose.copy()
        self._start_odom = odom.copy() if odom is not None else None
        self._wheel_progress = self._checkpoint = 0.
        return self._result(True, -cfg.reverse_speed, reason='reverse_recovery', progress=0.)
