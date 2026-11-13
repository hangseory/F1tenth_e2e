"""ROS-message boundary tests; no ROS node, graph, or hardware is started."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from race_stack import recovery_context as module


class FakeNode:
    def p(self, name):
        return {
            'base_frame': 'base_link', 'odom_timeout': .2,
            'recovery_map_topic': '/map', 'recovery_pose_topic': '/amcl_pose',
            'recovery_map_frame': 'map', 'recovery_pose_timeout': 10.,
            'recovery_enabled_timeout': 1.2, 'recovery_teleop_timeout': .3,
            'recovery_max_position_std': .25, 'recovery_max_yaw_std': .25,
            'recovery_pose_jump_distance': .25, 'recovery_pose_jump_yaw': .25,
            'recovery_scan_match_min_points': 20,
            'recovery_scan_match_fraction': .25, 'recovery_scan_match_tolerance': .15,
        }[name]

    def create_subscription(self, *args):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000_000_000_000))


def odom(x=0., y=0., yaw=0., stamp=1000):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp.sec = stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return msg


def pose(x=0., y=0., yaw=0., stamp=1000):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = .04
    msg.pose.covariance[35] = .03  # Current Cartographer compatibility covariance.
    return msg


def grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = .1
    msg.info.width = msg.info.height = 100
    msg.info.origin.position.x = msg.info.origin.position.y = -5.
    msg.info.origin.orientation.w = 1.
    data = np.zeros((100, 100), dtype=int)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    msg.data = data.ravel().tolist()
    return msg


@pytest.fixture
def context(monkeypatch):
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    ctx = module.RecoveryContext(FakeNode(), 65)
    ctx._map(grid())
    ctx.odometry(odom())
    ctx._pose(pose())
    ctx._mission(Bool(data=True))
    ctx._manual_cb(Bool(data=False))
    return ctx


def test_stationary_localization_and_static_map_may_be_old(context):
    p, reason = context.assess(105., 1005., np.empty((0, 2)), check_match=False)
    assert reason == '' and np.allclose(p, (0, 0, 0))
    assert context.assess(111., 1011., np.empty((0, 2)), False)[1] == 'localization_stale'


def test_propagates_odometry_from_localization_frame(context):
    context._pose_anchor = None
    context._pose(pose(1., 2., math.pi/2))
    context.odometry(odom(.2, 0., .1))
    p, reason = context.assess(100., 1000., np.empty((0, 2)), False)
    assert not reason and np.allclose(p, (1., 2.2, math.pi/2+.1))


def test_mission_heartbeat_manual_and_teleop_are_all_required(context):
    assert context.permission(100.) == ''
    assert context.permission(102.) == 'mission_not_enabled'
    context._manual_cb(Bool(data=True))
    assert context.permission(100.) == 'manual_or_unknown_mode'
    context._manual_cb(Bool(data=False))
    context._teleop(AckermannDriveStamped())  # Even speed=0 cancels recovery.
    assert context.permission(100.) == 'teleop_override'
    assert context.permission(100.31) == ''


def test_localization_jump_and_reset_block_recovery(context):
    context._pose(pose(.8))
    assert context.assess(100., 1000., np.empty((0, 2)), False)[1] == 'localization_or_map_changed'
    context._reset(None)
    assert context.assess(102., 1002., np.empty((0, 2)), False)[1] == 'localization_reset'


@pytest.mark.parametrize('failure', ['frame', 'nan', 'covariance', 'timestamp', 'unsynchronized'])
def test_bad_localization_is_not_reused(context, failure):
    msg = pose()
    if failure == 'frame':
        msg.header.frame_id = 'odom'
    elif failure == 'nan':
        msg.pose.pose.position.x = math.nan
    elif failure == 'covariance':
        msg.pose.covariance[0] = 1.
    elif failure == 'timestamp':
        msg.header.stamp.sec = 900
    else:
        msg.header.stamp.sec = 999
    context._pose(msg)
    assert context.assess(100., 1000., np.empty((0, 2)), False)[1]


def test_scan_must_agree_with_map_even_with_small_covariance(context):
    unmatched = np.column_stack((np.ones(40), np.linspace(-.5, .5, 40)))
    assert context.assess(100., 1000., unmatched)[1] == 'map_scan_disagreement'
    matched = np.column_stack((np.full(40, 4.95), np.linspace(-.5, .5, 40)))
    assert context.assess(100., 1000., matched)[1] == ''


def test_map_metadata_change_requires_new_checks(context):
    msg = grid()
    msg.info.origin.position.x = -4.
    context._map(msg)
    assert context.assess(100., 1000., np.empty((0, 2)), False)[1] == 'localization_or_map_changed'
    msg.header.frame_id = 'wrong_map'
    context._map(msg)
    assert context.grid is None


def test_invalid_odometry_pose_blocks_map_propagation(context):
    msg = odom()
    msg.pose.pose.orientation.w = 0.
    context.odometry(msg)
    assert context.assess(100., 1000., np.empty((0, 2)), False)[1] == 'invalid_recovery_odometry'


def test_operator_takeover_does_not_erase_autonomous_reentry_cap(context):
    from race_stack.collision_guard import CollisionGuardNode
    guard = SimpleNamespace(recovery_context=context, _reentry_remaining=.25,
                            _reentry_odom=context.odom_pose, p=lambda _: .25)
    context._manual_cb(Bool(data=True))
    assert CollisionGuardNode._forward_request(guard, 1.4, 0.) == 1.4
    assert guard._reentry_remaining == .25
    context._manual_cb(Bool(data=False))
    assert CollisionGuardNode._forward_request(guard, 1.4, 0.) == .25
