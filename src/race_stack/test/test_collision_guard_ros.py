#!/usr/bin/env python3
"""Isolated collision-guard ROS integration test. No vehicle drivers are started.

After building race_stack, source ROS and f1tenth_ws/install/setup.bash, then run:
    python3 src/race_stack/test/test_collision_guard_ros.py

This script forces domain 92, localhost-only FastDDS, hardware:=false, and
recover:=false to exercise the collision filter independently of map recovery.
It is an executable integration test, deliberately not an ordinary pytest test:
ROS, launch, and the real mux/converter executables are required.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path('/tmp/yax_collision_guard_test'))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lock = (args.output_dir / 'test.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    # Set before importing/initializing rclpy, and inherit into every child.
    os.environ['ROS_DOMAIN_ID'] = '92'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(key, None)

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64, String
    from tf2_ros import StaticTransformBroadcaster
    import yaml

    repo = Path(__file__).resolve().parents[3]
    vesc = yaml.safe_load((repo / 'config/vesc.yaml').read_text())
    calibration = vesc['/**']['ros__parameters']
    gain = float(calibration['speed_to_erpm_gain'])
    steering_gain = float(calibration['steering_angle_to_servo_gain'])
    center = float(calibration['steering_angle_to_servo_offset'])
    servo_min = float(calibration['servo_min'])
    servo_max = float(calibration['servo_max'])

    rclpy.init()
    node = rclpy.create_node('collision_guard_integration_test')
    drive_pub = node.create_publisher(AckermannDriveStamped, '/drive', 1)
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, '/odom_imu', qos_profile_sensor_data)
    tf_pub = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'laser'
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.transform.translation.x = .27
    transform.transform.translation.z = .11
    transform.transform.rotation.w = 1.
    tf_pub.sendTransform(transform)

    commands, motor, servo, statuses, ready = [], [], [], [], []
    node.create_subscription(
        AckermannDriveStamped, '/ackermann_cmd',
        lambda m: commands.append((time.monotonic(), float(m.drive.speed),
                                    float(m.drive.steering_angle))), 10)
    node.create_subscription(
        Float64, '/safety/test/motor_speed',
        lambda m: motor.append((time.monotonic(), float(m.data))), 10)
    node.create_subscription(
        Float64, '/safety/test/servo_position',
        lambda m: servo.append((time.monotonic(), float(m.data))), 10)
    node.create_subscription(
        Bool, '/safety/motor_ready',
        lambda m: ready.append((time.monotonic(), bool(m.data))), 10)

    def status_cb(msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = {'invalid_json': msg.data}
        statuses.append((time.monotonic(), data))

    node.create_subscription(String, '/safety/collision_guard/status', status_cb, 10)
    log_path = args.output_dir / 'integration.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'race_stack', 'guarded_bringup.launch.py',
         'hardware:=false', 'recover:=false'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    stopped_pids = set()
    checks = []
    restart_measurements = {}
    started = time.monotonic()

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.005, max(0., deadline-time.monotonic())))

    def phase(duration, speed=.6, steering=0., measured_speed=0., yaw_rate=0.,
              wall_x=None, drive=True, scan=True, odom=True,
              old_command=False, old_scan=False, old_odom=False,
              invalid_scan=False, scan_frame='laser'):
        start = time.monotonic()
        finish = start + duration
        next_publish = start
        while time.monotonic() < finish:
            if process.poll() is not None:
                raise AssertionError('Guard launch exited early; inspect ' + str(log_path))
            now = time.monotonic()
            if now >= next_publish:
                stamp = node.get_clock().now()
                if drive:
                    msg = AckermannDriveStamped()
                    msg.header.frame_id = 'base_link'
                    msg.header.stamp = (stamp-Duration(seconds=.8) if old_command else stamp).to_msg()
                    msg.drive.speed = float(speed)
                    msg.drive.steering_angle = float(steering)
                    drive_pub.publish(msg)
                if odom:
                    msg = Odometry()
                    msg.header.frame_id = 'odom'
                    msg.child_frame_id = 'base_link'
                    msg.header.stamp = (stamp-Duration(seconds=.8) if old_odom else stamp).to_msg()
                    msg.pose.pose.orientation.w = 1.
                    msg.twist.twist.linear.x = float(measured_speed)
                    msg.twist.twist.angular.z = float(yaw_rate)
                    odom_pub.publish(msg)
                if scan:
                    msg = LaserScan()
                    msg.header.frame_id = scan_frame
                    msg.header.stamp = (stamp-Duration(seconds=.8) if old_scan else stamp).to_msg()
                    msg.angle_min = -3*math.pi/4
                    msg.angle_max = 3*math.pi/4
                    msg.angle_increment = math.pi/720
                    msg.range_min, msg.range_max = .02, 30.
                    msg.scan_time, msg.time_increment = .025, .025/1081
                    msg.ranges = [float('inf') if invalid_scan else 8.] * 1081
                    if wall_x is not None:
                        # A vertical wall in front of the vehicle, with the
                        # actual laser offset already respected by the scan.
                        for idx in range(420, 661):
                            angle = msg.angle_min + idx*msg.angle_increment
                            msg.ranges[idx] = (wall_x-.27)/math.cos(angle)
                    scan_pub.publish(msg)
                next_publish = now + .025
            pump(.003)
        cutoff = finish-min(.18, duration/3)
        return {
            'commands': [x for x in commands if x[0] >= cutoff],
            'motor': [x for x in motor if x[0] >= cutoff],
            'servo': [x for x in servo if x[0] >= cutoff],
            'statuses': [x for x in statuses if x[0] >= start],
            'start': start, 'end': time.monotonic(),
        }

    def record(label):
        checks.append(label)
        print('PASS:', label, flush=True)

    def expect_zero(label, sample, check_command=True):
        assert sample['motor'], (label, 'no motor samples', statuses[-3:])
        assert all(abs(x[1]) < 1e-6 for x in sample['motor']), (label, sample)
        if check_command:
            assert sample['commands'], (label, 'no filtered commands', statuses[-3:])
            assert all(abs(x[1]) < 1e-8 for x in sample['commands']), (label, sample)
        record(label)

    def expect_drive(label, sample, requested=.6, expected_steer=None):
        assert sample['commands'] and sample['motor'], (label, sample, statuses[-3:])
        assert all(0 < x[1] <= requested+1e-6 for x in sample['commands']), (label, sample)
        assert all(0 < x[1]/gain <= requested+1e-6 for x in sample['motor']), (label, sample)
        if expected_steer is not None:
            expected_servo = min(servo_max, max(servo_min, center+steering_gain*expected_steer))
            assert sample['servo'], (label, 'no servo output')
            assert all(abs(x[1]-expected_servo) < 1e-5 for x in sample['servo']), (label, sample)
        record(label)

    def process_pid(executable):
        # Signals are restricted to the session created for this test launch.
        listing = subprocess.check_output(['ps', '-eo', 'pid,sid,args'], text=True)
        matches = []
        for line in listing.splitlines()[1:]:
            columns = line.split(None, 2)
            if len(columns) != 3:
                continue
            pid, sid, command = int(columns[0]), int(columns[1]), columns[2]
            argv = shlex.split(command)
            # Python console scripts can appear after /usr/bin/python3.
            if sid == process.pid and any(Path(arg).name == executable for arg in argv[:4]):
                matches.append(pid)
        assert len(matches) == 1, (executable, matches)
        return matches[0]

    def suspend(executable):
        pid = process_pid(executable)
        os.kill(pid, signal.SIGSTOP)
        stopped_pids.add(pid)
        return pid

    def resume(pid):
        os.kill(pid, signal.SIGCONT)
        stopped_pids.discard(pid)

    report = {'passed': False, 'hardware': False, 'domain': 92, 'checks': checks}
    try:
        expect_zero('startup without data is stopped', phase(5., drive=False, scan=False, odom=False))
        names = node.get_node_names()
        assert not any(name in names for name in ('vesc_driver_node', 'urg_node', 'joy')), names
        assert node.count_publishers('/commands/motor/speed') == 0
        assert node.count_publishers('/commands/servo/position') == 0
        record('hardware=false has no vehicle driver or real motor/servo publisher')

        expect_drive('clear path resumes after startup hold', phase(2.))
        expect_drive('actual converter preserves requested steering',
                     phase(.5, steering=.15), expected_steer=.15)
        expect_drive('safe output below controller speed floor remains below 1.4 m/s',
                     phase(.7, speed=.4), requested=.4)

        sample = phase(1.7, speed=1.2, wall_x=1.)
        expect_drive('predicted obstacle allows only a lower safe speed', sample, requested=1.2)
        assert all(x[1] < 1.05 for x in sample['commands']), sample
        record('predictive slowdown reduces a nonzero request before near-stop zone')

        expect_zero('close obstacle stops and holds beyond old one-second timer',
                    phase(1.7, wall_x=.52))
        expect_zero('zero request does not clear obstacle latch',
                    phase(.6, speed=0., wall_x=.52))
        expect_zero('repeated forward request with obstacle remains stopped',
                    phase(.7, wall_x=.52))
        expect_zero('brief clear scan does not release stop', phase(.2))
        expect_zero('obstacle returning resets release timer', phase(.2, wall_x=.52))
        start = time.monotonic()
        sample = phase(1.6)
        expect_drive('sustained clear path releases stop and ramps speed', sample)
        resumed = [x for x in commands if x[0] >= start and x[1] > 1e-5]
        assert resumed and resumed[0][0]-start >= .40, resumed[:5]
        assert resumed[0][1] <= .12, resumed[:5]
        record('release uses hold time and restarts from low output')

        faults = [
            ('scan timeout', {'scan': False}),
            ('odom timeout', {'odom': False}),
            ('command timeout', {'drive': False}),
            ('replayed stale command header', {'old_command': True}),
            ('replayed stale scan header', {'old_scan': True}),
            ('replayed stale odom header', {'old_odom': True}),
            ('all invalid scan', {'invalid_scan': True}),
            ('unknown scan transform', {'scan_frame': 'unavailable_laser'}),
            ('nonfinite speed command', {'speed': float('nan')}),
            ('nonfinite steering command', {'steering': float('nan')}),
            ('reverse request in rear blind zone', {'speed': -.4}),
        ]
        for label, options in faults:
            expect_zero(label+' -> stop', phase(.7, **options))
            expect_drive(label+' cleared -> guarded restart', phase(1.5))

        for executable in ('collision_guard', 'ackermann_to_vesc_node', 'ackermann_mux'):
            expect_drive(executable+' active before freeze', phase(.4))
            pid = suspend(executable)
            frozen_at = time.monotonic()
            expect_zero(executable+' frozen -> independent motor stop',
                        phase(.6), check_command=(executable == 'ackermann_mux'))
            zero_times = [t for t, value in motor if t >= frozen_at and abs(value) < 1e-6]
            assert zero_times and zero_times[0]-frozen_at < .45, (executable, zero_times)
            record(executable+' freeze stops motor within 0.45 s')
            if executable != 'ackermann_mux':
                assert any(t > frozen_at+.25 and not value for t, value in ready), (executable, ready[-20:])
                record(executable+' freeze disarms final motor watchdog')
            resumed_at = time.monotonic()
            resume(pid)
            expect_drive(executable+' resumed -> restart', phase(1.7))
            resumed_motor = [(t, value/gain) for t, value in motor if t >= resumed_at]
            moving = [(t, value) for t, value in resumed_motor if value > 1e-5]
            assert moving and moving[0][1] <= .12, (executable, moving[:8])
            delay = moving[0][0]-resumed_at
            assert delay >= (.55 if executable != 'ackermann_mux' else .40), (executable, delay)
            # Do not merely check that the first output is small: catch a
            # one-sample zero/small command followed by the old full request.
            first_t, first_v = moving[0]
            assert all(value <= first_v+.8*(t-first_t)+.06 for t, value in moving), (executable, moving[:20])
            restart_measurements[executable] = {
                'delay_after_resume_s': round(delay, 3),
                'first_motor_speed_mps': round(first_v, 4),
            }
            record(executable+' restart waits, starts low, and obeys acceleration ramp')

        assert statuses and all('invalid_json' not in x[1] for x in statuses)
        record('guard emits structured diagnostic status')
        assert all(math.isfinite(x[1]) for x in motor)
        assert all(servo_min-1e-6 <= x[1] <= servo_max+1e-6 for x in servo)
        record('motor values finite and servo inside calibrated limits')

        # The final fault verifies orderly launch shutdown and the stop helper.
        expect_drive('active before unexpected guard exit', phase(.5))
        os.kill(process_pid('collision_guard'), signal.SIGTERM)
        deadline = time.monotonic()+8.
        while process.poll() is None and time.monotonic() < deadline:
            pump(.02)
        assert process.poll() is not None, 'launch did not shut down after critical exit'
        assert motor and abs(motor[-1][1]) < 1e-6, motor[-10:]
        record('critical process exit sends zero before launch shutdown')
        report.update(passed=True, elapsed_sec=round(time.monotonic()-started, 2),
                      restart_measurements=restart_measurements,
                      command_samples=len(commands), motor_samples=len(motor),
                      status_samples=len(statuses),
                      max_abs_motor_rpm=max(abs(x[1]) for x in motor),
                      servo_min=min(x[1] for x in servo), servo_max=max(x[1] for x in servo))
    except Exception as exc:
        report.update(error=repr(exc), last_statuses=[x[1] for x in statuses[-5:]])
        raise
    finally:
        for pid in stopped_pids:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        (args.output_dir / 'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        lock.close()


if __name__ == '__main__':
    main()
