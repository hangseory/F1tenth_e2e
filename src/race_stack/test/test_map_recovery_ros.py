#!/usr/bin/env python3
"""Standalone, synthetic map-recovery ROS integration test; no vehicle drivers.

Source ROS and the built workspace, then run this file with Python. Every case
starts guarded_bringup hardware:=false recover:=true in localhost domain 93.
Only isolated /safety/test motor outputs move the synthetic vehicle. No actual
driver, hardware service, or real motor/servo command publisher is started.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    names = (
        'complete', 'rear_occupied', 'rear_unknown', 'missing_map',
        'missing_pose', 'poor_pose', 'mission_idle', 'manual_cancel',
        'enabled_cancel', 'teleop_cancel', 'scan_cancel', 'pose_jump_cancel',
        'no_progress',
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path('/tmp/yax_map_recovery_test'))
    parser.add_argument('--case', choices=names, action='append', dest='cases',
                        help='Run selected cases when investigating a failure.')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lock = Path('/tmp/yax_map_recovery_domain93.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.environ['ROS_DOMAIN_ID'] = '93'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(key, None)

    import numpy as np
    import rclpy
    import yaml
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64, String
    from tf2_ros import StaticTransformBroadcaster

    repo = Path(__file__).resolve().parents[3]
    calibration = yaml.safe_load((repo/'config/vesc.yaml').read_text())['/**']['ros__parameters']
    gain = float(calibration['speed_to_erpm_gain'])
    assert math.isfinite(gain) and gain > 0
    rclpy.init()
    checks, case_results = [], {}
    report = {'passed': False, 'hardware': False, 'domain': 93,
              'checks': checks, 'cases': case_results}
    started = time.monotonic()

    def record(case, label):
        checks.append(case+': '+label)
        print('PASS:', checks[-1], flush=True)

    class Harness:
        def __init__(self, name):
            self.name = name
            self.directory = args.output_dir/name
            self.directory.mkdir(parents=True, exist_ok=True)
            self.node = rclpy.create_node('map_recovery_test_'+name)
            latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.drive_pub = self.node.create_publisher(AckermannDriveStamped, '/drive', 1)
            self.teleop_pub = self.node.create_publisher(AckermannDriveStamped, '/teleop', 1)
            self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            self.odom_pub = self.node.create_publisher(Odometry, '/odom_imu', qos_profile_sensor_data)
            self.pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 1)
            self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', latch)
            self.enabled_pub = self.node.create_publisher(Bool, '/race/enabled', latch)
            self.manual_pub = self.node.create_publisher(Bool, '/race/manual', latch)
            self.tf_pub = StaticTransformBroadcaster(self.node)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.transform.translation.x = .27
            transform.transform.translation.z = .11
            transform.transform.rotation.w = 1.
            self.tf_pub.sendTransform(transform)
            self.motor, self.commands, self.statuses, self.motion = [], [], [], []
            self.node.create_subscription(Float64, '/safety/test/motor_speed',
                lambda msg: self.motor.append((time.monotonic(), float(msg.data)/gain)), 20)
            self.node.create_subscription(AckermannDriveStamped, '/ackermann_cmd',
                lambda msg: self.commands.append((time.monotonic(), float(msg.drive.speed),
                                                 float(msg.drive.steering_angle))), 20)
            self.node.create_subscription(String, '/safety/collision_guard/status', self.status_cb, 10)
            self.x = self.y = self.yaw = self.distance = 0.
            self.enabled = self.manual = self.teleop = self.invalid_scan = False
            self.publish_pose = name != 'missing_pose'
            self.poor_pose = name == 'poor_pose'
            self.progress = name != 'no_progress'
            self.pose_bias = 0.
            self.request = .3
            self.angles = np.linspace(-3*math.pi/4, 3*math.pi/4, 1081)
            # World-coordinate physical walls. The map uses the same walls.
            self.walls = [('x', .55, -1.5, 1.5), ('x', -3.5, -1.5, 1.5),
                          ('y', -1.5, -3.5, .55), ('y', 1.5, -3.5, .55)]
            if name == 'rear_occupied':
                self.walls.append(('x', -.27, -1., 1.))
            self.map_msg = self.make_map(name == 'rear_unknown')
            self.publish_map = name != 'missing_map'
            self.next_publish = self.next_map = 0.
            self.next_enabled = 0.
            self.last_enabled = self.last_manual = None
            self.last_step = time.monotonic()
            self.log = (self.directory/'integration.log').open('w')
            self.process = subprocess.Popen(
                ['ros2', 'launch', 'race_stack', 'guarded_bringup.launch.py',
                 'hardware:=false', 'recover:=true'],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

        def status_cb(self, msg):
            try:
                value = json.loads(msg.data)
            except (ValueError, TypeError):
                value = {'invalid_json': msg.data}
            self.statuses.append((time.monotonic(), value))

        def make_map(self, unknown):
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.info.resolution = .025
            msg.info.width, msg.info.height = 200, 160
            msg.info.origin.position.x, msg.info.origin.position.y = -4., -2.
            msg.info.origin.orientation.w = 1.
            cells = np.zeros((160, 200), dtype=np.int8)
            for axis, position, lo, hi in self.walls:
                if axis == 'x':
                    col = int(math.floor((position+4.)/.025+1e-7))
                    row0, row1 = int((lo+2.)/.025), int((hi+2.)/.025)
                    cells[row0:row1+1, col] = 100
                else:
                    row = int(math.floor((position+2.)/.025+1e-7))
                    col0, col1 = int((lo+4.)/.025), int((hi+4.)/.025)
                    cells[row, col0:col1+1] = 100
            if unknown:
                # Behind the current body but inside the proposed reverse sweep.
                cells[40:121, :int((-.27+4.)/.025)] = -1
            msg.data = cells.reshape(-1).tolist()
            return msg

        def ranges(self):
            angles = self.angles+self.yaw
            dx, dy = np.cos(angles), np.sin(angles)
            lx, ly = self.x+.27*math.cos(self.yaw), self.y+.27*math.sin(self.yaw)
            result = np.full(len(angles), 8.)
            for axis, position, lo, hi in self.walls:
                direction, origin, cross_origin, cross_direction = (
                    (dx, lx, ly, dy) if axis == 'x' else (dy, ly, lx, dx))
                with np.errstate(divide='ignore', invalid='ignore'):
                    distance = (position-origin)/direction
                    cross = cross_origin+distance*cross_direction
                valid = (distance > .02) & (cross >= lo-1e-8) & (cross <= hi+1e-8)
                result = np.minimum(result, np.where(valid, distance, 8.))
            return result.tolist()

        def step(self):
            if self.process.poll() is not None:
                raise AssertionError('isolated launch exited: '+str(self.directory/'integration.log'))
            now = time.monotonic()
            if now >= self.next_publish:
                dt = min(.1, max(0., now-self.last_step))
                self.last_step = now
                # Only final isolated motor output drives this synthetic model.
                # Forward outputs are observed, but do not approach the wall.
                speed = min(0., self.motor[-1][1]) if self.motor and now-self.motor[-1][0] < .2 else 0.
                if not self.progress:
                    speed = 0.
                steering = self.commands[-1][2] if self.commands else 0.
                yaw_rate = speed*math.tan(steering)/.33
                self.x += speed*math.cos(self.yaw+yaw_rate*dt/2)*dt
                self.y += speed*math.sin(self.yaw+yaw_rate*dt/2)*dt
                self.yaw += yaw_rate*dt
                self.distance += abs(speed)*dt
                self.motion.append((now, self.x, self.y, self.yaw, self.distance))
                stamp = self.node.get_clock().now().to_msg()
                if self.enabled != self.last_enabled or now >= self.next_enabled:
                    self.enabled_pub.publish(Bool(data=self.enabled))
                    self.last_enabled, self.next_enabled = self.enabled, now+.5
                if self.manual != self.last_manual:
                    self.manual_pub.publish(Bool(data=self.manual))
                    self.last_manual = self.manual
                msg = AckermannDriveStamped()
                msg.header.frame_id, msg.header.stamp = 'base_link', stamp
                msg.drive.speed = self.request
                self.drive_pub.publish(msg)
                if self.teleop:
                    msg.drive.speed = 0.
                    self.teleop_pub.publish(msg)
                odom = Odometry()
                odom.header.frame_id, odom.header.stamp = 'odom', stamp
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = (
                    math.sin(self.yaw/2), math.cos(self.yaw/2))
                odom.twist.twist.linear.x = speed
                odom.twist.twist.angular.z = yaw_rate
                self.odom_pub.publish(odom)
                if self.publish_pose:
                    pose = PoseWithCovarianceStamped()
                    pose.header.frame_id, pose.header.stamp = 'map', stamp
                    pose.pose.pose.position.x = self.x+self.pose_bias
                    pose.pose.pose.position.y = self.y
                    pose.pose.pose.orientation = odom.pose.pose.orientation
                    pose.pose.covariance[0] = pose.pose.covariance[7] = (1. if self.poor_pose else .0025)
                    pose.pose.covariance[35] = .0025
                    self.pose_pub.publish(pose)
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = 'laser', stamp
                scan.angle_min, scan.angle_max = -3*math.pi/4, 3*math.pi/4
                scan.angle_increment = math.pi/720
                scan.scan_time, scan.time_increment = .025, .025/1081
                scan.range_min, scan.range_max = .02, 30.
                scan.ranges = [float('nan')]*1081 if self.invalid_scan else self.ranges()
                self.scan_pub.publish(scan)
                if self.publish_map and now >= self.next_map:
                    self.map_pub.publish(self.map_msg)
                    self.next_map = math.inf  # Static map_server-style latched publication.
                self.next_publish = now+.025
            rclpy.spin_once(self.node, timeout_sec=.003)

        def run(self, seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                self.step()

        def until(self, condition, timeout, label):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                self.step()
                if condition():
                    return
            raise AssertionError((label, 'timeout', self.statuses[-4:]))

        def start(self):
            self.until(lambda: bool(self.motor and self.commands and self.statuses),
                       12., 'graph startup')
            self.run(.35)
            node_names = self.node.get_node_names()
            assert not any(name in node_names for name in ('vesc_driver_node', 'urg_node', 'joy'))
            assert self.node.count_publishers('/commands/motor/speed') == 0
            assert self.node.count_publishers('/commands/servo/position') == 0
            record(self.name, 'isolated graph has no vehicle drivers or hardware output publishers')
            self.enabled = self.name != 'mission_idle'
            self.begin = time.monotonic()

        def negative(self, since=None):
            since = self.begin if since is None else since
            return [(t, v) for t, v in self.motor if t >= since and v < -1e-5]

        def stopped(self, since):
            samples = [(t, v) for t, v in self.motor if t >= since]
            assert samples and all(abs(v) < 1e-6 for _, v in samples), (samples[-12:], self.statuses[-4:])

        def close(self):
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=3.)
            (self.directory/'trace.json').write_text(json.dumps({
                'motor_mps': self.motor, 'commands': self.commands,
                'motion': self.motion, 'statuses': self.statuses}, indent=2)+'\n')
            self.node.destroy_node()
            self.log.close()

    try:
        for name in args.cases or names:
            harness = Harness(name)
            result = case_results[name] = {'passed': False}
            try:
                harness.start()
                if name in ('rear_occupied', 'rear_unknown', 'missing_map',
                            'missing_pose', 'poor_pose', 'mission_idle'):
                    harness.run(2.6)
                    assert not harness.negative(), (name, harness.negative(), harness.statuses[-4:])
                    harness.stopped(harness.begin+.4)
                    record(name, 'recovery stays stopped when authorization or rear clearance is absent')
                    if name == 'rear_occupied':
                        # The reverse plan remains blocked, but forward AEB must
                        # still release after the front obstacle moves away.
                        harness.walls = [('x', .95, -1.5, 1.5), ('x', -3.5, -1.5, 1.5),
                                         ('y', -1.5, -3.5, .95), ('y', 1.5, -3.5, .95),
                                         ('x', -.27, -1., 1.)]
                        harness.map_msg = harness.make_map(False)
                        harness.next_map = 0.
                        cleared = time.monotonic()
                        harness.run(2.)
                        forward = [(t, v) for t, v in harness.motor if t >= cleared and v > 1e-5]
                        assert forward and not harness.negative(), harness.statuses[-5:]
                        assert forward[0][0]-cleared >= .4 and forward[0][1] <= .12
                        tail = [v for t, v in harness.motor if t >= time.monotonic()-.15]
                        assert tail and all(0 < v <= harness.request+1e-6 for v in tail), tail
                        record(name, 'blocked reverse plan still permits guarded forward restart once clear')
                else:
                    harness.until(lambda: bool(harness.negative()), 5., 'reverse starts')
                    first_t, first_v = harness.negative()[0]
                    result['reverse_delay_s'] = round(first_t-harness.begin, 3)
                    assert first_t-harness.begin >= .9, (name, result)
                    record(name, 'reverse starts only after the one-second stopped hold')
                    if name == 'complete':
                        harness.until(lambda: any(t > first_t and v > 1e-5 for t, v in harness.motor),
                                      6., 'forward resumes after recovery')
                        harness.run(.6)
                        reverse = harness.negative()
                        forward = [(t, v) for t, v in harness.motor if t > first_t and v > 1e-5]
                        assert reverse and all(-.25001 <= v < 0 for _, v in reverse)
                        assert .20 <= harness.distance <= .40, harness.distance
                        record(name, 'reverse speed and traveled distance remain bounded')
                        ft, fv = forward[0]
                        assert ft-reverse[-1][0] >= .25, (reverse[-1], forward[:5])
                        assert fv <= .12 and all(v <= fv+.8*(t-ft)+.06 for t, v in forward)
                        # This model does not advance on positive output, so the
                        # .25 m low-speed reentry distance cannot expire here.
                        assert all(v <= min(.25, harness.request)+1e-6 for _, v in forward)
                        record(name, 'reverse settles before forward output restarts with acceleration limit')
                        result.update(distance_m=round(harness.distance, 4),
                                      first_forward_mps=round(fv, 4),
                                      settle_before_forward_s=round(ft-reverse[-1][0], 3))
                    elif name == 'no_progress':
                        harness.run(4.)
                        harness.stopped(first_t+3.2)
                        assert harness.distance == 0.
                        zeros = [(t, v) for t, v in harness.motor if t > first_t+.05 and abs(v) < 1e-6]
                        assert zeros and zeros[0][0]-first_t <= 3.2
                        locks = [(t, s) for t, s in harness.statuses if t >= first_t
                                 and s.get('recovery', {}).get('reason') in
                                 ('reverse_no_progress', 'reverse_timeout')]
                        assert locks, harness.statuses[-5:]
                        harness.stopped(locks[0][0]+.15)
                        result['stop_reason'] = locks[0][1]['recovery']['reason']
                        record(name, 'lack of actual progress stops reverse within the time bound')
                    else:
                        harness.run(.15)
                        cancelled = time.monotonic()
                        if name == 'manual_cancel':
                            harness.manual = True
                        elif name == 'enabled_cancel':
                            harness.enabled = False
                        elif name == 'teleop_cancel':
                            harness.teleop = True
                        elif name == 'scan_cancel':
                            harness.invalid_scan = True
                        elif name == 'pose_jump_cancel':
                            harness.pose_bias = 1.
                        harness.run(.7)
                        zeros = [(t, v) for t, v in harness.motor if t >= cancelled and abs(v) < 1e-6]
                        assert zeros and zeros[0][0]-cancelled <= .20, (name, harness.statuses[-4:])
                        harness.stopped(cancelled+.20)
                        result['stop_latency_s'] = round(zeros[0][0]-cancelled, 3)
                        record(name, 'reverse cancels promptly and remains stopped')
                assert all(math.isfinite(v) for _, v in harness.motor)
                assert any(isinstance(s.get('recovery'), dict) for _, s in harness.statuses), harness.statuses[-3:]
                record(name, 'finite motor output and structured recovery diagnostics')
                result['passed'] = True
            except Exception as exc:
                result.update(error=repr(exc), last_statuses=[s for _, s in harness.statuses[-5:]])
                raise
            finally:
                harness.close()
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        report['elapsed_sec'] = round(time.monotonic()-started, 2)
        if rclpy.ok():
            rclpy.shutdown()
        (args.output_dir/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        lock.close()


if __name__ == '__main__':
    main()
