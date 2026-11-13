"""Predictive Ackermann collision guard, independent of ROS and localization.

Obstacle points are in the vehicle's current base_link frame (x forward,
y left). Unknown-space coverage and sensor freshness belong to the caller.
This module never publishes a command or accesses vehicle hardware.
"""

from dataclasses import dataclass, field, fields
import math
import time

import numpy as np


@dataclass(frozen=True)
class GuardConfig:
    front: float = .50
    rear: float = .15
    half_width: float = .17
    margin: float = .06
    wheelbase: float = .33
    reaction_time: float = .45
    brake_decel: float = 2.0
    prediction_dt: float = .02
    max_horizon: float = 4.0
    max_speed: float = 6.0
    release_margin: float = .15
    release_hold: float = .5
    restart_accel: float = .8
    max_steer: float = .4796

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('nonfinite/invalid guard config: ' + item.name)
        nonnegative = ('rear', 'margin', 'reaction_time', 'release_margin', 'release_hold')
        for item in fields(self):
            value = getattr(self, item.name)
            if value < 0 or (item.name not in nonnegative and value == 0):
                raise ValueError('invalid guard config: ' + item.name)
        if self.max_steer >= math.pi / 2 or self.prediction_dt > self.max_horizon:
            raise ValueError('invalid steering angle or prediction interval')


@dataclass(frozen=True)
class GuardResult:
    speed: float
    steering: float
    state: str
    reason: str
    details: dict = field(default_factory=dict)


class CollisionGuard:
    def __init__(self, config=None):
        self.config = config if config is not None else GuardConfig()
        if not isinstance(self.config, GuardConfig):
            raise ValueError('config must be GuardConfig')
        self._latched = False
        self._release_since = None
        self._last_now = None
        self._last_output = 0.
        self._last_intent = None
        self._ramping = True

    def stopping_distance(self, speed):
        v = abs(speed)
        return v * self.config.reaction_time + v*v / (2*self.config.brake_decel)

    def _pose(self, speed, yaw_rate, elapsed):
        angle = yaw_rate * elapsed
        if abs(yaw_rate) < 1e-10:
            return speed * elapsed, 0., angle
        radius = speed / yaw_rate
        return radius * math.sin(angle), radius * (1-math.cos(angle)), angle

    def _hit(self, points, pose, padding=0.):
        if len(points) == 0:
            return False
        x, y, angle = pose
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = points[:, 0]-x, points[:, 1]-y
        local_x, local_y = c*dx+s*dy, -s*dx+c*dy
        cfg = self.config
        inflate = cfg.margin + padding
        return bool(np.any((local_x >= -cfg.rear-inflate)
                           & (local_x <= cfg.front+inflate)
                           & (np.abs(local_y) <= cfg.half_width+inflate)))

    def _sweep(self, points, speed, yaw_rate, duration):
        """Conservative first collision distance, and tested interval count.

        Midpoint rectangles are inflated by the maximum footprint-point travel
        over half the interval. Thus the complete continuous swept rectangle is
        covered, including rotating corners between samples. A hit reports the
        interval START distance, never an optimistic midpoint distance.
        """
        if self._hit(points, (0., 0., 0.)):
            return 0., 0
        if len(points) == 0 or duration <= 0:
            return math.inf, 0
        cfg = self.config
        radius = math.hypot(max(cfg.front, cfg.rear)+cfg.margin,
                            cfg.half_width+cfg.margin)
        point_speed_bound = abs(speed) + abs(yaw_rate)*radius
        count = max(1, int(math.ceil(duration/cfg.prediction_dt)))
        step = duration/count
        padding = point_speed_bound*step/2
        for i in range(count):
            middle = (i+.5)*step
            if self._hit(points, self._pose(speed, yaw_rate, middle), padding):
                return abs(speed)*i*step, i+1
        return math.inf, count

    def _stop(self, steering, reason, details):
        self._latched = True
        self._release_since = None
        self._last_output = 0.
        self._ramping = True
        details['latched'] = True
        return GuardResult(0., steering, 'STOPPED', reason, details)

    def update(self, points, requested_speed, requested_steering, actual_speed,
               actual_yaw_rate, now, healthy=True, unhealthy_reason='unhealthy'):
        start = time.perf_counter()
        cfg = self.config
        steering = (float(np.clip(requested_steering, -cfg.max_steer, cfg.max_steer))
                    if isinstance(requested_steering, (int, float, np.number))
                    and math.isfinite(requested_steering) else 0.)
        details = {'latched': self._latched}
        try:
            numeric = np.asarray([requested_speed, requested_steering, actual_speed,
                                  actual_yaw_rate, now], dtype=float)
            if not np.isfinite(numeric).all():
                raise ValueError('nonfinite motion/time')
            points = np.asarray(points, dtype=float)
            if points.size == 0:
                points = np.empty((0, 2), dtype=float)
            if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
                raise ValueError('invalid obstacle array')
        except (ValueError, TypeError, OverflowError):
            return self._stop(steering, 'invalid_input', details)
        if self._last_now is not None and now < self._last_now:
            self._last_now = now
            return self._stop(steering, 'nonmonotonic_time', details)
        dt = min(.1, max(0., now-self._last_now)) if self._last_now is not None else 0.
        self._last_now = now
        if not healthy:
            return self._stop(steering, unhealthy_reason or 'unhealthy', details)
        request = float(np.clip(requested_speed, -cfg.max_speed, cfg.max_speed))
        if abs(request) > 1e-9:
            self._last_intent = (request, steering)
        # A zero request must not forget the obstacle path that caused a stop.
        intent, intent_steer = (self._last_intent if self._latched
                                and abs(request) <= 1e-9 and self._last_intent
                                else (request, steering))
        check_speed = max(abs(intent), abs(actual_speed))
        stop_time = cfg.reaction_time + check_speed/cfg.brake_decel
        details.update(requested_speed=request, actual_speed=float(actual_speed),
                       stopping_distance=self.stopping_distance(actual_speed),
                       required_horizon=stop_time, max_horizon=cfg.max_horizon)
        if stop_time > cfg.max_horizon:
            return self._stop(steering, 'insufficient_prediction_horizon', details)
        if self._hit(points, (0., 0., 0.)):
            details['clearance'] = 0.
            return self._stop(steering, 'footprint_obstacle', details)
        direction = math.copysign(1., intent if abs(intent) > 1e-9
                                  else actual_speed if abs(actual_speed) > 1e-9 else 1.)
        search_distance = self.stopping_distance(check_speed) + cfg.release_margin
        # Geometric path sampling may run faster than the car at crawl speed;
        # otherwise the extra release-margin search would take unbounded time.
        sampling_speed = max(check_speed, .1)
        request_duration = search_distance/sampling_speed if check_speed > 1e-9 else 0.
        # Explicitly refuse insufficient prediction coverage, never silently clip.
        if request_duration > cfg.max_horizon:
            return self._stop(steering, 'insufficient_prediction_horizon', details)
        request_v = direction*sampling_speed
        request_w = request_v*math.tan(intent_steer)/cfg.wheelbase
        requested_clearance, n_req = self._sweep(points, request_v, request_w,
                                                request_duration)
        actual_duration = cfg.reaction_time + abs(actual_speed)/cfg.brake_decel
        actual_clearance, n_actual = self._sweep(points, actual_speed, actual_yaw_rate,
                                                actual_duration)
        clearance = min(requested_clearance, actual_clearance)
        details.update(clearance=clearance, requested_clearance=requested_clearance,
                       actual_clearance=actual_clearance,
                       requested_horizon=request_duration, actual_horizon=actual_duration,
                       checked_intervals=n_req+n_actual,
                       compute_ms=(time.perf_counter()-start)*1000)
        if clearance <= self.stopping_distance(actual_speed):
            return self._stop(steering, 'stopping_distance_obstacle', details)
        if self._latched:
            release_distance = self.stopping_distance(abs(intent)) + cfg.release_margin
            if abs(request) <= 1e-9 or clearance <= release_distance:
                self._release_since = None
                details['latched'] = True
                return GuardResult(0., steering, 'STOPPED', 'obstacle_latch', details)
            if self._release_since is None:
                self._release_since = now
            if now-self._release_since < cfg.release_hold:
                details['release_elapsed'] = now-self._release_since
                return GuardResult(0., steering, 'STOPPED', 'release_hold', details)
            self._latched = False
            self._release_since = None
        # The inflated footprint already accounts for the static margin.
        safe_speed = (math.sqrt((cfg.brake_decel*cfg.reaction_time)**2
                                + 2*cfg.brake_decel*clearance)
                      - cfg.brake_decel*cfg.reaction_time) if math.isfinite(clearance) else cfg.max_speed
        target = min(abs(request), max(0., safe_speed))
        previous = abs(self._last_output) if self._last_output*request >= 0 else 0.
        limited = target < abs(request)-1e-9
        if limited or self._last_output*request < 0 or request == 0:
            self._ramping = True
        # Normal controller speed profiles pass through once restart is over.
        # Following a stop or obstacle cap, increases recover gradually; speed
        # reductions are never delayed by this limiter.
        output = min(target, previous+cfg.restart_accel*dt) if self._ramping else target
        restarting = output < target-1e-9
        if not restarting and not limited and request != 0:
            self._ramping = False
        self._last_output = math.copysign(output, request) if request else 0.
        details.update(latched=self._latched, safe_speed=safe_speed, target_speed=target)
        return GuardResult(self._last_output, steering,
                           'RESTARTING' if restarting else 'LIMITED' if limited else 'CLEAR',
                           'acceleration_limit' if restarting else 'obstacle_speed_cap' if limited else '',
                           details)
