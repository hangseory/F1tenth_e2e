"""Swept-footprint, stopping envelope, and restart-state regression tests."""
import math
import unittest

import numpy as np

from race_stack.collision_guard_core import CollisionGuard, GuardConfig


class CollisionGuardTest(unittest.TestCase):
    def call(self, guard, points=(), request=1., steer=0., actual=0., yaw=0.,
             now=0., healthy=True):
        return guard.update(points, request, steer, actual, yaw, now, healthy)

    def test_clear_path_respects_request_and_restart_acceleration(self):
        guard = CollisionGuard()
        self.assertEqual(self.call(guard, request=.5).speed, 0.)
        previous = 0.
        for i in range(1, 12):
            result = self.call(guard, request=.5, now=i*.1)
            self.assertLessEqual(result.speed, .5)
            self.assertLessEqual(result.speed-previous, .080000001)
            previous = result.speed
        self.assertAlmostEqual(result.speed, .5)
        self.assertEqual(result.state, 'CLEAR')

    def test_full_rectangle_includes_front_side_and_rear(self):
        for point in [[.55, 0.], [.1, .22], [-.20, 0.]]:
            with self.subTest(point=point):
                result = self.call(CollisionGuard(), [point])
                self.assertEqual(result.speed, 0.)
                self.assertEqual(result.reason, 'footprint_obstacle')

    def test_normal_speed_profile_passes_after_restart(self):
        guard = CollisionGuard()
        for i in range(10):
            self.call(guard, request=.5, now=i*.1)
        result = self.call(guard, request=1.5, now=1.)
        self.assertEqual(result.speed, 1.5)
        self.assertEqual(result.state, 'CLEAR')

    def test_stopping_envelope_causes_immediate_stop(self):
        result = self.call(CollisionGuard(), [[1., 0.]], request=2., actual=1.)
        self.assertEqual(result.reason, 'stopping_distance_obstacle')
        self.assertLessEqual(result.details['clearance'], .44)
        self.assertAlmostEqual(result.details['stopping_distance'], .7)
        self.assertEqual(result.speed, 0.)

    def test_requested_curve_and_actual_curve_are_both_checked(self):
        point = [[.75, .45]]
        straight = self.call(CollisionGuard(), point, actual=1.)
        self.assertNotEqual(straight.state, 'STOPPED')
        actual_arc = self.call(CollisionGuard(), point, actual=1., yaw=1.)
        self.assertEqual(actual_arc.reason, 'stopping_distance_obstacle')
        requested_arc = self.call(CollisionGuard(), point, actual=1.,
                                  steer=math.atan(.33))
        self.assertEqual(requested_arc.reason, 'stopping_distance_obstacle')

    def test_motion_between_discrete_poses_is_covered(self):
        cfg = GuardConfig(front=.1, rear=.1, half_width=.05, margin=0.,
                          prediction_dt=.5, reaction_time=.5)
        guard = CollisionGuard(cfg)
        # Exact center of an actual-motion arc at t=.25, outside both endpoint
        # boxes at t=0 and .5. Naive endpoint-only boxes would miss this obstacle.
        point = np.array([[math.sin(.5)/2, (1-math.cos(.5))/2]])
        self.assertFalse(guard._hit(point, guard._pose(1., 2., 0.)))
        self.assertFalse(guard._hit(point, guard._pose(1., 2., .5)))
        self.assertTrue(guard._hit(point, guard._pose(1., 2., .25)))
        result = self.call(guard, point, actual=1., yaw=2.)
        self.assertEqual(result.state, 'STOPPED')

    def test_continuous_corner_sweep_at_maximum_steering(self):
        # Many off-grid corner positions on both turning directions, forward
        # and reverse, must fall within the inflated discrete sweep. This
        # checks the rotating rectangle, not just centerline obstacle points.
        cfg = GuardConfig(prediction_dt=.1)
        guard = CollisionGuard(cfg)
        checks = 0
        for speed in [-2., 2.]:
            for steer in [-cfg.max_steer, cfg.max_steer]:
                w = speed*math.tan(steer)/cfg.wheelbase
                for t in [.037, .083, .147, .219, .373]:
                    x, y, angle = guard._pose(speed, w, t)
                    c, s = math.cos(angle), math.sin(angle)
                    for longitudinal in [-cfg.rear, cfg.front]:
                        for lateral in [-cfg.half_width, cfg.half_width]:
                            point = np.array([[x+c*longitudinal-s*lateral,
                                               y+s*longitudinal+c*lateral]])
                            clearance, _ = guard._sweep(point, speed, w, .5)
                            self.assertLessEqual(clearance, abs(speed)*t+1e-9)
                            checks += 1
        self.assertEqual(checks, 80)

    def test_reverse_uses_rear_footprint_and_signed_motion(self):
        result = self.call(CollisionGuard(), [[-.65, 0.]], request=-1., actual=-1.)
        self.assertEqual(result.reason, 'stopping_distance_obstacle')
        guard = CollisionGuard()
        self.call(guard, request=-.5)
        moving = self.call(guard, request=-.5, now=.1)
        self.assertLess(moving.speed, 0.)
        self.assertGreaterEqual(moving.speed, -.5)

    def test_clearance_reduces_speed_before_emergency_threshold(self):
        guard = CollisionGuard()
        for i in range(40):
            result = self.call(guard, [[1.5, 0.]], request=2., actual=.2, now=.1*i)
        self.assertEqual(result.state, 'LIMITED')
        self.assertGreater(result.speed, .2)
        self.assertLess(result.speed, 2.)
        self.assertLessEqual(guard.stopping_distance(result.speed),
                             result.details['clearance']+1e-9)

    def test_zero_request_does_not_clear_latch(self):
        guard = CollisionGuard()
        self.call(guard, [[.7, 0.]], request=.5, actual=.5)
        stopped = self.call(guard, [[.7, 0.]], request=0., actual=0., now=.1)
        self.assertEqual(stopped.state, 'STOPPED')
        self.assertTrue(stopped.details['latched'])
        empty_zero = self.call(guard, request=0., now=1.)
        self.assertEqual(empty_zero.state, 'STOPPED')
        self.assertTrue(empty_zero.details['latched'])
        # The retained intent still sees the obstacle once motion is requested.
        same_obstacle = self.call(guard, [[.7, 0.]], request=.5, now=1.1)
        self.assertEqual(same_obstacle.reason, 'obstacle_latch')

    def test_release_requires_margin_and_hold_then_slow_restart(self):
        guard = CollisionGuard()
        self.call(guard, [[.7, 0.]], request=.5, actual=.5)
        # Distance near .3 m is enough for .5 m/s stopping, not release margin.
        near = self.call(guard, [[.9, 0.]], request=.5, now=.1)
        self.assertEqual(near.reason, 'obstacle_latch')
        first = self.call(guard, request=.5, now=.2)
        self.assertEqual(first.reason, 'release_hold')
        hold = self.call(guard, request=.5, now=.6)
        self.assertEqual(hold.speed, 0.)
        resumed = self.call(guard, request=.5, now=.8)
        self.assertGreater(resumed.speed, 0.)
        self.assertLessEqual(resumed.speed, .080000001)
        self.assertEqual(resumed.state, 'RESTARTING')

    def test_new_obstacle_resets_release_timer(self):
        guard = CollisionGuard()
        self.call(guard, [[.7, 0.]], request=.5, actual=.5)
        self.call(guard, request=.5, now=.1)
        self.call(guard, request=.5, now=.4)
        blocked = self.call(guard, [[.9, 0.]], request=.5, now=.45)
        self.assertEqual(blocked.reason, 'obstacle_latch')
        self.call(guard, request=.5, now=.5)
        too_early = self.call(guard, request=.5, now=.8)
        self.assertEqual(too_early.speed, 0.)
        self.assertEqual(too_early.reason, 'release_hold')
        resumed = self.call(guard, request=.5, now=1.1)
        self.assertGreater(resumed.speed, 0.)

    def test_insufficient_horizon_fails_closed(self):
        result = self.call(CollisionGuard(GuardConfig(max_horizon=3.)),
                           request=6., actual=6.)
        self.assertEqual(result.reason, 'insufficient_prediction_horizon')
        self.assertEqual(result.speed, 0.)

    def test_slow_request_does_not_exhaust_horizon_due_to_release_margin(self):
        guard = CollisionGuard()
        result = self.call(guard, request=.01)
        self.assertNotEqual(result.state, 'STOPPED')

    def test_invalid_input_and_config_fail_closed(self):
        for cfg in [dict(brake_decel=0), dict(front=math.nan),
                    dict(prediction_dt=math.inf), dict(max_steer=2.)]:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                GuardConfig(**cfg)
        for bad in [dict(points=[[math.nan, 0.]]), dict(request=math.nan),
                    dict(steer=math.inf), dict(actual=math.nan), dict(yaw=math.inf),
                    dict(now=math.nan), dict(points=[[0., 0., 0.]])]:
            with self.subTest(bad=bad):
                result = self.call(CollisionGuard(), **bad)
                self.assertEqual(result.speed, 0.)
                self.assertEqual(result.reason, 'invalid_input')
        self.assertEqual(self.call(CollisionGuard(), healthy=False).state, 'STOPPED')

    def test_steering_is_preserved_or_clamped(self):
        result = self.call(CollisionGuard(), steer=.2)
        self.assertEqual(result.steering, .2)
        result = self.call(CollisionGuard(), steer=2.)
        self.assertEqual(result.steering, GuardConfig().max_steer)


if __name__ == '__main__':
    unittest.main()
