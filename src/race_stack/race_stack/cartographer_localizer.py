"""Start/reseed Cartographer and adapt its base_link pose to /amcl_pose.

Covariance is a configurable compatibility value, not a Cartographer estimate.
Forwarding requires recent scan-matched data, not just extrapolated poses.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import (
    FinishTrajectory, GetTrajectoryStates, StartTrajectory, TrajectoryQuery)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def pose2d(x, y, heading):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z, p.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
    return p


def relative_pose(reference, target):
    """StartTrajectory uses the frozen trajectory's pose at time zero as origin."""
    angle = yaw(reference.orientation)
    dx, dy = target.position.x-reference.position.x, target.position.y-reference.position.y
    return pose2d(math.cos(angle)*dx + math.sin(angle)*dy,
                  -math.sin(angle)*dx + math.cos(angle)*dy,
                  yaw(target.orientation)-angle)


class CartographerLocalizer(Node):
    def __init__(self):
        super().__init__('cartographer_localizer')
        for name, default in (
            ('configuration_directory', ''),
            ('configuration_basename', 'cartographer_driving.lua'),
            ('odom_topic', '/odom_imu'), ('init_x', 0.0), ('init_y', 0.0),
            ('init_yaw', 0.0), ('pose_variance_xy', 0.04),
            ('pose_variance_yaw', 0.03), ('scan_timeout', 0.3),
        ):
            self.declare_parameter(name, default)
        self.target = pose2d(self.p('init_x'), self.p('init_y'), self.p('init_yaw'))
        self.reference = None
        self.frozen_id = None
        self.active_id = None
        self.ready = False
        self.pending = None
        self.future = None
        self.matched_stamp = None
        self.started_ns = 0
        self.odom = None
        self.odom_received = 0.0
        self.queued_pose = None
        self.service_clients = {
            'states': self.create_client(GetTrajectoryStates, '/get_trajectory_states'),
            'query': self.create_client(TrajectoryQuery, '/trajectory_query'),
            'start': self.create_client(StartTrajectory, '/start_trajectory'),
            'finish': self.create_client(FinishTrajectory, '/finish_trajectory'),
        }
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.create_subscription(PoseStamped, '/tracked_pose', self.pose_cb, 10)
        self.create_subscription(PointCloud2, '/scan_matched_points2', self.match_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_cb, 10)
        self.create_subscription(Odometry, self.p('odom_topic'), self.odom_cb,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.request('states', GetTrajectoryStates.Request(), self.states_cb)
        self.get_logger().info('Cartographer 위치추정 준비: 저장 맵 로딩 후 초기 위치 적용')

    def p(self, name):
        return self.get_parameter(name).value

    def request(self, name, request, callback):
        self.pending = (name, request, callback, time.monotonic())
        self.future = None

    def tick(self):
        if self.pending is None:
            return
        name, request, callback, start = self.pending
        if time.monotonic() - start > 120:
            self.fail(f'{name} service timeout; restart localization after checking Cartographer')
            return
        if self.future is None:
            if self.service_clients[name].service_is_ready():
                self.future = self.service_clients[name].call_async(request)
            return
        if not self.future.done():
            return
        try:
            result = self.future.result()
            self.pending, self.future = None, None
            if result is None or result.status.code != 0:
                raise RuntimeError(str(result))
            callback(result)
        except Exception as exc:
            self.fail(f'{name}: {exc}')

    def fail(self, message):
        self.pending, self.future, self.ready = None, None, False
        self.get_logger().error(message)

    def states_cb(self, result):
        states = result.trajectory_states
        pairs = list(zip(states.trajectory_id, states.trajectory_state))
        frozen = [tid for tid, state in pairs if state == TrajectoryStates.FROZEN]
        if len(frozen) != 1 or any(state == TrajectoryStates.ACTIVE for _, state in pairs):
            raise RuntimeError(f'Expected one frozen map and no active trajectory: {pairs}')
        self.frozen_id = frozen[0]
        self.request('query', TrajectoryQuery.Request(trajectory_id=self.frozen_id), self.query_cb)

    def query_cb(self, result):
        if not result.trajectory:
            raise RuntimeError('Loaded map has no trajectory poses')
        self.reference = result.trajectory[0].pose
        self.start_trajectory()

    def start_trajectory(self):
        self.ready = False
        self.matched_stamp = None
        self.request('start', StartTrajectory.Request(
            configuration_directory=self.p('configuration_directory'),
            configuration_basename=self.p('configuration_basename'),
            use_initial_pose=True,
            initial_pose=relative_pose(self.reference, self.target),
            relative_to_trajectory_id=self.frozen_id), self.started_cb)

    def started_cb(self, result):
        self.active_id = result.trajectory_id
        self.started_ns = self.get_clock().now().nanoseconds
        self.ready = True
        self.get_logger().info(f'Cartographer trajectory {self.active_id} 시작 → /amcl_pose')
        if self.queued_pose is not None:
            target, self.queued_pose = self.queued_pose, None
            self.reset(target)

    def odom_cb(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def initial_cb(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warn('/initialpose frame_id must be map')
            return
        p = msg.pose.pose
        values = (p.position.x, p.position.y, p.orientation.x, p.orientation.y,
                  p.orientation.z, p.orientation.w)
        norm = sum(v*v for v in values[2:])
        if not all(math.isfinite(v) for v in values) or abs(norm-1) > 0.01:
            self.get_logger().warn('Invalid /initialpose')
            return
        # Starting/restarting a trajectory is only accepted while stationary.
        if self.odom is None or time.monotonic()-self.odom_received > 0.5:
            self.get_logger().warn('초기 위치 재설정 보류: 최신 odom 없음')
            return
        v = self.odom.twist.twist
        if math.hypot(v.linear.x, v.linear.y) > 0.05 or abs(v.angular.z) > 0.1:
            self.get_logger().warn('차를 세운 뒤 초기 위치를 다시 지정하세요')
            return
        if self.pending is not None:
            self.queued_pose = p
            return
        if self.active_id is None:
            self.get_logger().error('Cartographer 초기화 실패 상태 — 노드를 재시작하세요')
            return
        self.reset(p)

    def reset(self, pose):
        self.target, self.ready = pose, False
        self.matched_stamp = None
        self.request('finish', FinishTrajectory.Request(trajectory_id=self.active_id),
                     lambda result: self.start_trajectory())

    def match_cb(self, msg):
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        if self.ready and stamp > self.started_ns:
            self.matched_stamp = stamp

    def pose_cb(self, msg):
        if not self.ready or self.matched_stamp is None or msg.header.frame_id != 'map':
            return
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        timeout = self.p('scan_timeout')*1e9
        if not (0 <= now-self.matched_stamp <= timeout and 0 <= now-stamp <= timeout):
            return
        out = PoseWithCovarianceStamped()
        out.header, out.pose.pose = msg.header, msg.pose
        # Compatibility values only: Cartographer PoseStamped has no covariance.
        out.pose.covariance[0] = out.pose.covariance[7] = self.p('pose_variance_xy')
        out.pose.covariance[35] = self.p('pose_variance_yaw')
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
