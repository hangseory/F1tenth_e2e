"""Opt-in, post-mux Ackermann collision filter. Never owns a vehicle driver.

The geometry policy is pure Python/NumPy. This adapter supplies fresh, stamped
sensor data in base_link and emits zero continuously when an input is missing.
"""
from dataclasses import fields
import json
import math
import os
import signal
import time

import numpy as np
import rclpy
import rclpy.logging  # get_logging_directory — 상태 기록 파일 위치
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from race_stack.collision_guard_core import CollisionGuard, GuardConfig
from race_stack.collision_guard_core import GuardResult
from race_stack.recovery_core import RecoveryConfig, RecoveryController
from race_stack.recovery_context import RecoveryContext


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


class CollisionGuardNode(Node):
    def __init__(self):
        super().__init__('collision_guard')
        readonly = ParameterDescriptor(read_only=True)
        defaults = GuardConfig()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name), readonly)
        recovery_defaults = RecoveryConfig()
        for field in fields(recovery_defaults):
            self.declare_parameter('recovery_' + field.name,
                                   getattr(recovery_defaults, field.name), readonly)
        for name, value in {
            'input_topic': '/safety/raw_drive', 'output_topic': '/ackermann_cmd',
            'scan_topic': '/scan', 'odom_topic': '/odom_imu',
            'base_frame': 'base_link', 'status_topic': '/safety/collision_guard/status',
            'reverse_mode': 'block', 'command_timeout': .2,
            'scan_timeout': .2, 'odom_timeout': .2, 'motor_ready_timeout': .25,
            'control_rate': 40.,
            'max_unknown_gap_deg': 5., 'min_valid_fraction': .2,
            'required_forward_fov_deg': 180., 'bench': False,
            'recovery_enabled': False, 'recovery_map_topic': '/map',
            'recovery_pose_topic': '/amcl_pose', 'recovery_map_frame': 'map',
            'recovery_pose_timeout': 10., 'recovery_enabled_timeout': 1.2,
            'recovery_teleop_timeout': .3, 'recovery_max_position_std': .25,
            'recovery_max_yaw_std': .25, 'recovery_pose_jump_distance': .25,
            'recovery_pose_jump_yaw': .25, 'recovery_scan_match_tolerance': .15,
            'recovery_scan_match_fraction': .25, 'recovery_scan_match_min_points': 20,
            'recovery_reentry_speed': .25, 'recovery_reentry_distance': .25,
        }.items():
            self.declare_parameter(name, value, readonly)
        self.cfg = GuardConfig(**{f.name: float(self.p(f.name)) for f in fields(defaults)})
        for name in ('command_timeout', 'scan_timeout', 'odom_timeout', 'motor_ready_timeout', 'control_rate',
                     'max_unknown_gap_deg', 'required_forward_fov_deg'):
            if not math.isfinite(float(self.p(name))) or float(self.p(name)) <= 0:
                raise ValueError('invalid parameter: ' + name)
        if not 0 < self.p('min_valid_fraction') <= 1:
            raise ValueError('min_valid_fraction must be in (0, 1]')
        if not 0 < self.p('required_forward_fov_deg') <= 180:
            raise ValueError('required_forward_fov_deg must be in (0, 180]')
        if self.p('reverse_mode') != 'block':
            raise ValueError('Only reverse_mode=block is supported with the rear blind zone')
        self.policy = CollisionGuard(self.cfg)
        self.recovery_cfg = RecoveryConfig(**{
            f.name: self.p('recovery_' + f.name) for f in fields(recovery_defaults)})
        for name in ('pose_timeout', 'enabled_timeout', 'teleop_timeout',
                     'max_position_std', 'max_yaw_std', 'pose_jump_distance',
                     'pose_jump_yaw', 'scan_match_tolerance', 'scan_match_min_points',
                     'reentry_speed', 'reentry_distance'):
            if not math.isfinite(float(self.p('recovery_' + name))) or self.p('recovery_' + name) <= 0:
                raise ValueError('invalid recovery parameter: ' + name)
        if not 0 < self.p('recovery_scan_match_fraction') <= 1:
            raise ValueError('recovery_scan_match_fraction must be in (0, 1]')
        self.recovery = (RecoveryController(self.cfg, self.recovery_cfg)
                         if self.p('recovery_enabled') else None)
        self.recovery_context = (RecoveryContext(self, self.recovery_cfg.occupancy_threshold)
                                 if self.recovery is not None else None)
        self._reentry_remaining = 0.
        self._reentry_odom = None
        self._cmd = self._odom = None
        self._cmd_rx = self._odom_rx = self._scan_rx = None
        self._cmd_stamp = self._odom_stamp = self._scan_stamp = 0.
        self._points = np.empty((0, 2))
        self._scan_reason = 'waiting_scan'
        self._scan_meta = {}
        self._range_max = 0.
        self._sensor_offset = 0.
        self._estop = False
        self._motor_ready = False
        self._motor_ready_rx = None
        self._last_status = None
        self._status_t = 0.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.out = self.create_publisher(AckermannDriveStamped, self.p('output_topic'), 1)
        self.status = self.create_publisher(String, self.p('status_topic'), 1)
        self.create_subscription(AckermannDriveStamped, self.p('input_topic'), self._command, 1)
        self.create_subscription(Odometry, self.p('odom_topic'), self._odometry, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p('scan_topic'), self._scan, qos_profile_sensor_data)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/safety/emergency_stop', self._emergency, latch)
        self.create_subscription(Bool, '/safety/motor_ready', self._motor_ready_cb, 1)
        self.create_timer(1. / self.p('control_rate'), self._tick)
        self._open_status_log()
        self.get_logger().info(
            f'Predictive collision guard: {self.p("input_topic")} -> {self.p("output_topic")}; '
            + ('map-checked automatic reverse recovery enabled; manual reverse blocked.'
               if self.recovery else 'forward protection, reverse blocked (rear blind zone).'))

    # ── 상태 기록 (2026-11-13) ────────────────────────────────────────────
    #   /safety/collision_guard/status 의 JSON 을 **파일로도** 남긴다.
    #   예전엔 --bag 을 안 켜면 전부 사라졌다. 후진 복구 시험 세 번이 bag 없이
    #   돌아서, 막힘 판정·후진 시도·후보 수·후진 거리를 하나도 볼 수 없었다.
    #   ROS 로그 디렉토리에 <이름>_<pid>_<시각>.jsonl 로 쓴다 — race.sh 가
    #   주행 끝에 PID 로 찾아 logs/<런>/ros/collision_guard.jsonl 로 모은다.
    #   평소엔 상태 발행 주기(≥5Hz + 전이마다), **후진 중엔 매 주기** 기록한다.
    #   ★ 이 노드는 안전 장치다. 기록이 실패해도 주행에 영향이 가면 안 되므로
    #     열기·쓰기 실패는 전부 삼키고 기록만 끈다.
    def _open_status_log(self):
        self._status_log = None
        try:
            d = rclpy.logging.get_logging_directory()
            os.makedirs(d, exist_ok=True)
            path = os.path.join(
                d, f'collision_guard_status_{os.getpid()}_{int(time.time() * 1000)}.jsonl')
            self._status_log = open(path, 'w', buffering=1)
        except Exception as exc:
            self._status_log = None
            self.get_logger().warn(f'상태 기록 파일을 못 열었다 — 기록 없이 계속: {exc}')

    def _write_status_log(self, payload, ros_now):
        if self._status_log is None:
            return
        try:
            line = json.dumps(json_safe(dict(payload, t=ros_now)), allow_nan=False)
            self._status_log.write(line + '\n')
        except Exception:
            self._status_log = None          # 한 번 실패하면 끈다 (주행 우선)

    def p(self, name):
        return self.get_parameter(name).value

    def _emergency(self, msg):
        if msg.data:
            self._estop = True  # No accidental reset from another publisher.

    def _motor_ready_cb(self, msg):
        self._motor_ready, self._motor_ready_rx = bool(msg.data), time.monotonic()

    def _command(self, msg):
        self._cmd = (float(msg.drive.speed), float(msg.drive.steering_angle))
        self._cmd_rx = time.monotonic()
        self._cmd_stamp = stamp_seconds(msg.header.stamp)

    def _odometry(self, msg):
        self._odom = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
        self._odom_rx = time.monotonic()
        self._odom_stamp = stamp_seconds(msg.header.stamp)
        if self.recovery_context is not None:
            self.recovery_context.odometry(msg)

    def _scan(self, msg):
        self._scan_rx = time.monotonic()
        self._scan_stamp = stamp_seconds(msg.header.stamp)
        self._scan_reason = 'invalid_scan'
        self._scan_meta = {}
        try:
            r = np.asarray(msg.ranges, dtype=float)
            metadata = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            if (r.ndim != 1 or not 10 <= len(r) <= 8192
                    or not all(math.isfinite(v) for v in metadata)
                    or msg.angle_increment <= 0 or msg.range_min < 0
                    or msg.range_max <= msg.range_min or not msg.header.frame_id):
                return
            if msg.header.frame_id.lstrip('/') == self.p('base_frame').lstrip('/'):
                tx = ty = yaw = 0.
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.p('base_frame'), msg.header.frame_id, Time.from_msg(msg.header.stamp))
                q = transform.transform.rotation
                tr = transform.transform.translation
                values = (q.x, q.y, q.z, q.w, tr.x, tr.y, tr.z)
                if not all(math.isfinite(v) for v in values):
                    return
                if abs(q.x) > .01 or abs(q.y) > .01 or abs(sum(v*v for v in values[:4])-1) > .01:
                    self._scan_reason = 'nonplanar_scan_transform'
                    return
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                tx, ty = tr.x, tr.y
            angles = msg.angle_min + np.arange(len(r))*msg.angle_increment + yaw
            wrapped = np.arctan2(np.sin(angles), np.cos(angles))
            half_fov = math.radians(self.p('required_forward_fov_deg'))/2
            forward = np.abs(wrapped) <= half_fov
            # Coverage is required on both sides of forward; absent beams are
            # unknown, not infinite free space.
            if (not forward.any() or np.min(wrapped[forward]) > -half_fov + 2*msg.angle_increment
                    or np.max(wrapped[forward]) < half_fov - 2*msg.angle_increment):
                self._scan_reason = 'scan_forward_fov_missing'
                return
            valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max) & (r > 0)
            front_valid = valid[forward]
            run = longest = 0
            for missing in ~front_valid:
                run = run + 1 if missing else 0
                longest = max(longest, run)
            unknown_deg = math.degrees(longest*msg.angle_increment)
            self._scan_meta = {'points': int(valid.sum()), 'beams': len(r),
                               'forward_valid_fraction': float(front_valid.mean()),
                               'unknown_forward_gap_deg': unknown_deg}
            if front_valid.mean() < self.p('min_valid_fraction'):
                self._scan_reason = 'scan_insufficient_returns'
                return
            if unknown_deg > self.p('max_unknown_gap_deg'):
                self._scan_reason = 'scan_unknown_forward'
                return
            self._points = np.column_stack((tx + r[valid]*np.cos(angles[valid]),
                                            ty + r[valid]*np.sin(angles[valid])))
            self._range_max = float(msg.range_max)
            self._sensor_offset = math.hypot(tx, ty)
            self._scan_reason = ''
        except TransformException:
            self._scan_reason = 'scan_transform_unavailable'
        except (ValueError, TypeError, OverflowError):
            self._scan_reason = 'invalid_scan'

    def _health(self, now, ros_now):
        if self._estop:
            return 'emergency_stop'
        if self.p('bench'):
            return 'bench'
        if (not self._motor_ready or self._motor_ready_rx is None
                or now-self._motor_ready_rx > self.p('motor_ready_timeout')):
            return 'motor_watchdog_not_ready'
        for name, received, stamp in (
                ('command', self._cmd_rx, self._cmd_stamp),
                ('odom', self._odom_rx, self._odom_stamp),
                ('scan', self._scan_rx, self._scan_stamp)):
            timeout = self.p(name + '_timeout')
            if received is None:
                return 'waiting_' + name
            if now - received > timeout:
                return name + '_timeout'
            if stamp <= 0 or not -.05 <= ros_now - stamp <= timeout:
                return name + '_stamp_stale'
        if not all(math.isfinite(v) for v in (*self._cmd, *self._odom)):
            return 'nonfinite_motion'
        if self._scan_reason:
            return self._scan_reason
        own_reverse = self.recovery is not None and self.recovery.state in ('REVERSING', 'STOP_HOLD')
        if self._cmd[0] < -1e-6 or (self._odom[0] < -.05 and not own_reverse):
            return 'reverse_blind_zone'
        speed = max(min(abs(self._cmd[0]), self.cfg.max_speed), abs(self._odom[0]))
        coverage = (self.policy.stopping_distance(speed) + self.cfg.release_margin
                    + max(self.cfg.front, self.cfg.rear) + self.cfg.margin + self._sensor_offset)
        if coverage > self._range_max:
            return 'scan_range_insufficient'
        return ''

    def _publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p('base_frame')
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.out.publish(msg)

    def _recovery_step(self, result, request, steering, v, w, now, ros_now, reason):
        context = self.recovery_context
        permitted_reason = context.permission(now)
        active = self.recovery.state in ('REVERSING', 'STOP_HOLD')
        check_match = active or result.state == 'STOPPED'
        pose, map_reason = context.assess(now, ros_now, self._points, check_match)
        if not active and abs(w) >= .1:
            permitted_reason = permitted_reason or 'not_stationary'
        recovery_result = self.recovery.update(
            context.grid, pose, self._points, request, steering, v, now,
            result.state, result.reason, healthy=not reason and not map_reason,
            permitted=not permitted_reason, odom_pose=context.odom_pose)
        details = dict(recovery_result.details, state=recovery_result.state,
                       reason=recovery_result.reason, permission=permitted_reason,
                       map_reason=map_reason, map_match_fraction=context.match_fraction)
        if recovery_result.details.get('completed'):
            self._reentry_remaining = self.p('recovery_reentry_distance')
            self._reentry_odom = context.odom_pose
        if recovery_result.override:
            # Keep the original guard latched throughout reversal and settling.
            # It must independently clear and ramp the next forward command.
            if recovery_result.state in ('REVERSING', 'STOP_HOLD'):
                self.policy.update(self._points, request, steering, v, w, now,
                                   healthy=False, unhealthy_reason='recovery_active')
            if recovery_result.state == 'REVERSING':
                self._reentry_remaining = 0.
            return GuardResult(recovery_result.speed, recovery_result.steering,
                               recovery_result.state, recovery_result.reason,
                               dict(result.details, recovery=details))
        return GuardResult(result.speed, result.steering, result.state, result.reason,
                           dict(result.details, recovery=details))

    def _forward_request(self, request, v):
        if self.recovery_context is None:
            return request
        pose = self.recovery_context.odom_pose
        permitted = not self.recovery_context.permission(time.monotonic())
        if self._reentry_remaining > 0. and pose is not None and self._reentry_odom is not None:
            previous = self._reentry_odom
            dx, dy = pose[0]-previous[0], pose[1]-previous[1]
            distance = dx*math.cos(previous[2])+dy*math.sin(previous[2])
            if v > .01 and 0 < distance < .15:
                self._reentry_remaining = max(0., self._reentry_remaining-distance)
        self._reentry_odom = pose
        # Operator takeover may bypass the crawl cap, but must not erase the
        # remaining distance and restore full autonomous speed on the next tick.
        return (min(request, self.p('recovery_reentry_speed'))
                if permitted and self._reentry_remaining > 0 else request)

    def _tick(self):
        started = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds*1e-9
        reason = self._health(started, ros_now)
        request, steering = self._cmd if self._cmd is not None else (0., 0.)
        v, w = self._odom if self._odom is not None else (0., 0.)
        try:
            forward_request = self._forward_request(request, v)
            result = self.policy.update(self._points, forward_request, steering, v, w, started,
                                        healthy=not reason, unhealthy_reason=reason)
            if self.recovery is not None:
                result = self._recovery_step(result, request, steering, v, w,
                                             started, ros_now, reason)
        except Exception as exc:
            # A numerical failure must not leave the previous drive command live.
            self._estop = True
            self._publish(0., 0.)
            self.get_logger().error(f'Guard error; stop latched: {exc}')
            return
        # Heavy computation must not turn old observations into a new valid command.
        ended = time.monotonic()
        late_ros_now = self.get_clock().now().nanoseconds*1e-9
        late_reason = self._health(ended, late_ros_now)
        if self.recovery is not None and result.speed < 0:
            late_reason = late_reason or self.recovery_context.permission(ended)
            if not late_reason:
                _, late_reason = self.recovery_context.assess(ended, late_ros_now, self._points,
                                                             check_match=False)
        if late_reason:
            result = self.policy.update(self._points, request, steering, v, w, ended,
                                        healthy=False, unhealthy_reason=late_reason)
            if self.recovery is not None:
                result = self._recovery_step(result, request, steering, v, w,
                                             ended, late_ros_now, late_reason)
        speed = result.speed
        self._publish(speed, result.steering)
        key = (result.state, late_reason or result.reason)
        publish = key != self._last_status or ended-self._status_t >= .2
        # 후진 중(speed<0)엔 발행 주기와 무관하게 매 주기 파일에 남긴다 —
        # 후진은 1초 남짓이라 0.2s 간격으로는 몇 점밖에 안 잡힌다.
        if publish or speed < 0.:
            payload = dict(result.details, state='STOPPED' if late_reason else result.state,
                           reason=late_reason or result.reason, speed=speed,
                           steering=result.steering, requested_speed=request,
                           measured_speed=v, yaw_rate=w,
                           elapsed_ms=(ended-started)*1000, scan=self._scan_meta)
            self._write_status_log(payload, late_ros_now)
        if publish:
            self.status.publish(String(data=json.dumps(json_safe(payload), allow_nan=False)))
            if key != self._last_status:
                msg = f'{payload["state"]}: {payload["reason"] or "clear"}'
                # ★ 원인을 같이 찍는다 (2026-11-13). 예전엔 'LOCKED:
                #   invalid_recovery_input' 만 나와 무엇이 잘못됐는지 몰랐다.
                #   input_error·map_reason·permission 은 JSON 에는 있었는데
                #   로그 줄에는 안 나왔다.
                rec = payload.get('recovery') or {}
                extra = [f'{k}={rec[k]}' for k in ('input_error', 'map_reason', 'permission')
                         if isinstance(rec, dict) and rec.get(k)]
                if extra:
                    msg += '  (' + ', '.join(extra) + ')'
                self.get_logger().info(msg)
            self._last_status, self._status_t = key, ended

    def stop(self):
        self._estop = True
        self._publish(0., 0.)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CollisionGuardNode()
    stopping = False

    def handle_signal(_sig, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
