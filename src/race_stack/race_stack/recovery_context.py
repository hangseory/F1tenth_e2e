"""Map/localization and operator permission checks for opt-in reverse recovery.

This object observes the existing ROS graph. It never changes mission state or
publishes motion. Static map age is deliberately not a freshness condition.
"""
from collections import deque
import math
import time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid as GridMessage
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from race_stack.recovery_core import OccupancyGrid


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def planar_pose(pose):
    q, p = pose.orientation, pose.position
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(x) for x in values)
            or abs(q.x) > .01 or abs(q.y) > .01
            or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
        raise ValueError('invalid planar pose')
    return p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def propagate(pose_map, odom_anchor, odom_current):
    """map_T_base(anchor) * inverse(odom_T_base(anchor)) * odom_T_base(now)."""
    yaw = pose_map[2]-odom_anchor[2]
    dx, dy = odom_current[0]-odom_anchor[0], odom_current[1]-odom_anchor[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (pose_map[0]+c*dx-s*dy, pose_map[1]+s*dx+c*dy,
            pose_map[2]+angle_difference(odom_current[2], odom_anchor[2]))


class RecoveryContext:
    def __init__(self, node, occupancy_threshold):
        self.node = node
        self.threshold = occupancy_threshold
        self.grid = None
        self._map_geometry = None
        self._map_reason = 'waiting_map'
        self.odom_pose = None
        self._odom_frame = None
        self._odom_history = deque(maxlen=256)
        self._pose_anchor = self._odom_anchor = None
        self._pose_rx = None
        self._pose_stamp = 0.
        self._pose_reason = 'waiting_localization'
        self._blocked_until = 0.
        self._enabled, self._enabled_rx = False, None
        self._manual = None
        self._teleop_rx = None
        self.match_fraction = None
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(GridMessage, self.p('map_topic'), self._map, latch)
        node.create_subscription(PoseWithCovarianceStamped, self.p('pose_topic'), self._pose, 1)
        node.create_subscription(Bool, '/race/enabled', self._mission, latch)
        node.create_subscription(Bool, '/race/manual', self._manual_cb, latch)
        node.create_subscription(AckermannDriveStamped, '/teleop', self._teleop, 1)
        node.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._reset, 1)
        node.create_subscription(Bool, '/race/relocalize', self._relocalize, 1)

    def p(self, name):
        return self.node.p('recovery_' + name)

    def _mission(self, msg):
        self._enabled, self._enabled_rx = bool(msg.data), time.monotonic()

    def _manual_cb(self, msg):
        self._manual = bool(msg.data)

    def _teleop(self, _msg):
        # Includes the joystick-loss stop command, even in autonomous mode.
        self._teleop_rx = time.monotonic()

    def _reset(self, _msg):
        self._pose_anchor = self._odom_anchor = None
        self._pose_reason = 'localization_reset'
        self._blocked_until = time.monotonic()+1.

    def _relocalize(self, msg):
        if msg.data:
            self._reset(msg)

    def odometry(self, msg):
        try:
            if (not msg.header.frame_id or
                    msg.child_frame_id.lstrip('/') != self.node.p('base_frame').lstrip('/')):
                raise ValueError('wrong odometry frame')
            pose = planar_pose(msg.pose.pose)
            stamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            if self._odom_frame is not None and self._odom_frame != msg.header.frame_id:
                self._reset(msg)
                self._odom_history.clear()
            self._odom_frame = msg.header.frame_id
            self.odom_pose = pose
            self._odom_history.append((stamp, pose))
        except (ValueError, TypeError):
            self.odom_pose = None

    def _map(self, msg):
        self._map_reason = 'invalid_map'
        try:
            info = msg.info
            if (msg.header.frame_id.lstrip('/') != self.p('map_frame').lstrip('/')
                    or not math.isfinite(info.resolution) or not .005 <= info.resolution <= .5
                    or not 1 <= info.width*info.height <= 16000000
                    or info.width*info.height != len(msg.data)):
                raise ValueError('invalid map metadata')
            origin = planar_pose(info.origin)
            geometry = (info.width, info.height, info.resolution, *origin)
            data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
            if np.any((data < -1) | (data > 100)):
                raise ValueError('invalid occupancy value')
            self.grid = OccupancyGrid(data, info.resolution, origin,
                                      occupancy_threshold=self.threshold)
            if self._map_geometry is not None and geometry != self._map_geometry:
                self._blocked_until = time.monotonic()+1.
            self._map_geometry = geometry
            self._map_reason = ''
        except (ValueError, TypeError, OverflowError):
            self.grid = None

    def _pose(self, msg):
        self._pose_reason = 'invalid_localization'
        try:
            if msg.header.frame_id.lstrip('/') != self.p('map_frame').lstrip('/'):
                return
            pose = planar_pose(msg.pose.pose)
            cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
            if not np.isfinite(cov).all() or np.any(np.diag(cov) < 0):
                return
            if (max(cov[0, 0], cov[1, 1]) > self.p('max_position_std')**2
                    or cov[5, 5] > self.p('max_yaw_std')**2):
                self._pose_reason = 'localization_uncertain'
                return
            stamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            ros_now = self.node.get_clock().now().nanoseconds*1e-9
            if stamp <= 0 or not -.05 <= ros_now-stamp <= self.p('pose_timeout'):
                self._pose_reason = 'localization_stale'
                return
            if not self._odom_history:
                self._pose_reason = 'waiting_pose_odometry'
                return
            odom_stamp, odom = min(self._odom_history, key=lambda item: abs(item[0]-stamp))
            if abs(odom_stamp-stamp) > self.node.p('odom_timeout'):
                self._pose_reason = 'localization_odometry_unsynchronized'
                return
            if self._pose_anchor is not None and self._odom_anchor is not None:
                predicted = propagate(self._pose_anchor, self._odom_anchor, odom)
                if (math.hypot(pose[0]-predicted[0], pose[1]-predicted[1]) > self.p('pose_jump_distance')
                        or abs(angle_difference(pose[2], predicted[2])) > self.p('pose_jump_yaw')):
                    self._blocked_until = time.monotonic()+1.
            self._pose_anchor, self._odom_anchor = pose, odom
            self._pose_rx, self._pose_stamp = time.monotonic(), stamp
            self._pose_reason = ''
        except (ValueError, TypeError, OverflowError):
            pass

    def permission(self, now):
        if (not self._enabled or self._enabled_rx is None
                or now-self._enabled_rx > self.p('enabled_timeout')):
            return 'mission_not_enabled'
        if self._manual is not False:
            return 'manual_or_unknown_mode'
        if self._teleop_rx is not None and now-self._teleop_rx < self.p('teleop_timeout'):
            return 'teleop_override'
        return ''

    def assess(self, now, ros_now, points, check_match=True):
        """Return propagated map pose and an error, independent of drive permission."""
        self.match_fraction = None
        if self._map_reason or self.grid is None:
            return None, self._map_reason or 'waiting_map'
        if now < self._blocked_until:
            return None, 'localization_or_map_changed'
        if self._pose_reason or self._pose_anchor is None or self.odom_pose is None:
            return None, self._pose_reason or 'invalid_recovery_odometry'
        if (self._pose_rx is None or now-self._pose_rx > self.p('pose_timeout')
                or not -.05 <= ros_now-self._pose_stamp <= self.p('pose_timeout')):
            return None, 'localization_stale'
        pose = propagate(self._pose_anchor, self._odom_anchor, self.odom_pose)
        if check_match:
            ranges = np.linalg.norm(points, axis=1)
            local = points[(ranges >= .1) & (ranges <= 8.)]
            if len(local) < self.p('scan_match_min_points'):
                return pose, 'insufficient_map_scan_matches'
            if len(local) > 120:
                local = local[np.linspace(0, len(local)-1, 120).astype(int)]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            world = local @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])
            matched = self.grid.matches(world, tolerance=self.p('scan_match_tolerance'))
            self.match_fraction = float(np.mean(matched))
            if self.match_fraction < self.p('scan_match_fraction'):
                return pose, 'map_scan_disagreement'
        return pose, ''
