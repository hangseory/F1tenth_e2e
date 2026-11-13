"""Independent final motor watchdog for guarded_bringup only.

This process stays responsive if geometric prediction or the converter stalls.
It is the sole motor/servo publisher in the opt-in guarded launch graph.
"""
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float64, String


class CollisionMotorWatchdog(Node):
    def __init__(self):
        super().__init__('collision_motor_watchdog')
        readonly = ParameterDescriptor(read_only=True)
        for name, value in {
            'motor_input_topic': '/safety/motor_request',
            'servo_input_topic': '/safety/servo_request',
            'motor_output_topic': '/commands/motor/speed',
            'servo_output_topic': '/commands/servo/position',
            'command_timeout': .2, 'rearm_hold': .2, 'control_rate': 50., 'max_speed': 6.,
            'speed_to_erpm_gain': 0., 'speed_to_erpm_offset': 0.,
            'steering_angle_to_servo_offset': .485, 'servo_min': .12, 'servo_max': .85,
        }.items():
            self.declare_parameter(name, value, readonly)
        p = lambda key: self.get_parameter(key).value
        numeric = ('command_timeout', 'rearm_hold', 'control_rate', 'max_speed', 'speed_to_erpm_gain',
                   'speed_to_erpm_offset', 'steering_angle_to_servo_offset', 'servo_min', 'servo_max')
        if not all(math.isfinite(float(p(key))) for key in numeric):
            raise ValueError('nonfinite watchdog configuration')
        if (p('command_timeout') <= 0 or p('rearm_hold') <= 0 or p('control_rate') <= 0 or p('max_speed') <= 0
                or p('speed_to_erpm_gain') == 0 or p('speed_to_erpm_offset') != 0
                or not 0 <= p('servo_min') < p('servo_max') <= 1
                or not p('servo_min') <= p('steering_angle_to_servo_offset') <= p('servo_max')):
            raise ValueError('invalid watchdog calibration (zero speed must be zero ERPM)')
        self.limit = abs(p('speed_to_erpm_gain'))*p('max_speed')
        self.timeout = p('command_timeout')
        self.rearm_hold = p('rearm_hold')
        self._armed = False
        self._zero_since = None
        self.servo_min, self.servo_max = p('servo_min'), p('servo_max')
        self._motor = 0.
        self._servo = p('steering_angle_to_servo_offset')
        self._motor_rx = self._servo_rx = None
        self._motor_valid = self._servo_valid = False
        self._estop = False
        self._last_status = None
        self._status_t = 0.
        self.motor_pub = self.create_publisher(Float64, p('motor_output_topic'), 1)
        self.servo_pub = self.create_publisher(Float64, p('servo_output_topic'), 1)
        self.status_pub = self.create_publisher(String, '/safety/motor_watchdog/status', 1)
        self.ready_pub = self.create_publisher(Bool, '/safety/motor_ready', 1)
        self.create_subscription(Float64, p('motor_input_topic'), self._motor_cb, 1)
        self.create_subscription(Float64, p('servo_input_topic'), self._servo_cb, 1)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/safety/emergency_stop', self._emergency, latch)
        self.create_timer(1./p('control_rate'), self._tick)

    def _motor_cb(self, msg):
        self._motor_rx = time.monotonic()
        self._motor_valid = math.isfinite(msg.data) and abs(msg.data) <= self.limit + 1e-6
        self._motor = float(msg.data) if self._motor_valid else 0.

    def _servo_cb(self, msg):
        self._servo_rx = time.monotonic()
        self._servo_valid = math.isfinite(msg.data)
        if self._servo_valid:
            self._servo = min(self.servo_max, max(self.servo_min, float(msg.data)))

    def _emergency(self, msg):
        if msg.data:
            self._estop = True
            self.stop()

    def _tick(self):
        now = time.monotonic()
        reason = ''
        if self._estop:
            reason = 'emergency_stop'
        elif not self._motor_valid or not self._servo_valid:
            reason = 'waiting_or_invalid_command'
        elif now-self._motor_rx > self.timeout or now-self._servo_rx > self.timeout:
            reason = 'command_timeout'
        if reason:
            self._armed, self._zero_since = False, None
        elif not self._armed:
            if abs(self._motor) <= 1e-6:
                if self._zero_since is None:
                    self._zero_since = now
                if now-self._zero_since >= self.rearm_hold:
                    self._armed = True
            else:
                self._zero_since = None
            if not self._armed:
                reason = 'waiting_zero_rearm'
        output = 0. if reason else self._motor
        self.motor_pub.publish(Float64(data=float(output)))
        self.servo_pub.publish(Float64(data=float(self._servo)))
        self.ready_pub.publish(Bool(data=self._armed and not self._estop))
        if reason != self._last_status or now-self._status_t > .2:
            self.status_pub.publish(String(data=json.dumps({'reason': reason, 'erpm': output})))
            self._last_status, self._status_t = reason, now

    def stop(self):
        self._estop = True
        self.motor_pub.publish(Float64(data=0.))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CollisionMotorWatchdog()
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


def stop_main(args=None):
    """One-shot orderly shutdown helper; also used before killing launch groups."""
    rclpy.init(args=args)
    node = Node('collision_stop')
    latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    estop = node.create_publisher(Bool, '/safety/emergency_stop', latch)
    motor = node.create_publisher(Float64, '/commands/motor/speed', 1)
    # Brief DDS discovery wait, then repeatedly send both latch and final zero.
    end = time.monotonic() + .6
    while rclpy.ok() and time.monotonic() < end:
        estop.publish(Bool(data=True))
        motor.publish(Float64(data=0.))
        rclpy.spin_once(node, timeout_sec=.02)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
