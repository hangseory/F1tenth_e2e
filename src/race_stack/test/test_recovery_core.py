"""Map/scan corridor and bounded reverse-recovery regression checks."""
import math
import unittest

import numpy as np

from race_stack.collision_guard_core import GuardConfig
from race_stack.recovery_core import (
    OccupancyGrid, RecoveryConfig, RecoveryController, _arc, _rotation,
)


class RecoveryCoreTest(unittest.TestCase):
    def grid(self, front_wall=True, rear_wall=False, unknown_rear=False):
        data = np.zeros((120, 120), dtype=np.int8)
        if front_wall:
            # Cell centers x=.575, near edge x=.55: front safety margin
            # overlaps the wall, but the physical front at x=.50 is clear.
            data[:, 71] = 100
        if rear_wall or unknown_rear:
            data[:, 49:54] = -1 if unknown_rear else 100
        return OccupancyGrid(data, .05, (-3., -3., 0.))

    def call(self, controller, now=0., pose=(0., 0., 0.), actual=0.,
             request=1., steer=0., points=((.55, 0.),), grid=None,
             healthy=True, permitted=True, guard_state='STOPPED',
             guard_reason='footprint_obstacle', odom_pose=None):
        return controller.update(
            self.grid() if grid is None else grid, pose, points,
            request, steer, actual, now, guard_state, guard_reason,
            healthy=healthy, permitted=permitted,
            odom_pose=pose if odom_pose is None else odom_pose,
        )

    def start(self, controller=None, **kwargs):
        controller = controller or RecoveryController()
        first = self.call(controller, **kwargs)
        self.assertEqual(first.state, 'WAIT_BLOCKED')
        result = self.call(controller, now=1., **kwargs)
        return controller, result

    def finish_reverse(self, controller, start_time=1., start_x=0., grid=None):
        # Feed actual odometry and scan in their respective moving frames.
        for i in range(1, 16):
            x = start_x-.025*i
            result = self.call(controller, now=start_time+i*.1,
                               pose=(x, 0., 0.), actual=-.25,
                               points=((.55-x, 0.),), grid=grid)
            if result.state == 'STOP_HOLD':
                return result, start_time+i*.1, x
            self.assertEqual(result.state, 'REVERSING', result)
        self.fail('reverse distance bound did not stop')

    def test_trigger_requires_continuous_actual_stop_at_obstacle(self):
        c = RecoveryController()
        self.call(c, actual=.1)
        self.call(c, now=2., actual=.1)
        waiting = self.call(c, now=2.1)
        self.assertEqual(waiting.state, 'WAIT_BLOCKED')
        self.assertFalse(waiting.override)
        self.assertEqual(self.call(c, now=2.9).state, 'WAIT_BLOCKED')
        self.assertEqual(self.call(c, now=3.11).state, 'REVERSING')
        d = RecoveryController()
        self.call(d, guard_reason='stale_scan')
        result = self.call(d, now=5., guard_reason='stale_scan')
        self.assertEqual(result.state, 'IDLE')
        self.assertFalse(result.override)

    def test_front_margin_wall_can_be_escaped_but_physical_contact_cannot(self):
        _, result = self.start()
        self.assertEqual(result.state, 'REVERSING')
        self.assertEqual(result.speed, -.25)
        self.assertEqual(result.steering, 0.)
        self.assertGreater(result.details['escape_forward_clearance'], .3)
        self.assertGreater(result.details['checked_reverse_distance'], .35)
        for point in [(.49, 0.), (.1, .16)]:
            with self.subTest(point=point):
                _, locked = self.start(points=(point,))
                self.assertEqual(locked.state, 'LOCKED')
                self.assertEqual(locked.speed, 0.)

    def test_rear_wall_unknown_and_current_scan_each_block_reverse(self):
        cases = [dict(grid=self.grid(rear_wall=True)),
                 dict(grid=self.grid(unknown_rear=True)),
                 dict(points=((.55, 0.), (-.4, 0.)))]
        for kwargs in cases:
            with self.subTest(kwargs=list(kwargs)):
                _, result = self.start(**kwargs)
                self.assertEqual(result.reason, 'no_safe_reverse_candidate')
                self.assertTrue(result.override)
                self.assertEqual(result.speed, 0.)

    def test_outside_map_is_blocked(self):
        data = np.zeros((60, 30), dtype=np.int8)
        grid = OccupancyGrid(data, .05, (-.3, -1.5, 0.))
        _, result = self.start(grid=grid)
        self.assertEqual(result.state, 'LOCKED')

    def test_rechecks_entire_remaining_corridor_and_stopping_extension(self):
        c, _ = self.start()
        # At x=-.025, a new rear obstacle is beyond the .325m remaining
        # nominal movement plus rear body, but inside braking extension.
        result = self.call(c, now=1.1, pose=(-.025, 0., 0.), actual=-.25,
                           points=((.575, 0.), (-.59, 0.)))
        self.assertEqual(result.reason, 'reverse_corridor_blocked')
        self.assertEqual(result.speed, 0.)

    def test_front_swing_of_reverse_turn_is_checked(self):
        c = RecoveryController()
        scene = c._scene(self.grid(False), np.zeros(3), np.array([[.4, -.3]]))
        distance = .478125
        self.assertEqual(c._path_clearance(scene, np.zeros(3), -distance, 0.), distance)
        self.assertLess(c._path_clearance(scene, np.zeros(3), -distance,
                                          c.guard.max_steer), distance)
        self.assertEqual(c._path_clearance(scene, np.zeros(3), -distance,
                                           -c.guard.max_steer), distance)

    def test_continuous_sweep_catches_obstacle_between_sample_endpoints(self):
        guard = GuardConfig(front=.1, rear=.1, half_width=.05, margin=0.)
        c = RecoveryController(guard, RecoveryConfig(sample_step=.6))
        middle = _arc(-.3, guard.max_steer, guard.wheelbase)
        point = middle[:2].reshape(1, 2)
        scene = c._scene(self.grid(False), np.zeros(3), point)
        end = _arc(-.6, guard.max_steer, guard.wheelbase)
        self.assertTrue(scene.clear(np.zeros(3)))
        self.assertTrue(scene.clear(end))
        self.assertFalse(scene.clear(middle))
        self.assertLess(c._path_clearance(scene, np.zeros(3), -.6, guard.max_steer), .3)

    def test_distance_bound_then_stationary_hold_and_one_completion_tick(self):
        c, _ = self.start()
        result, time, x = self.finish_reverse(c)
        self.assertEqual(result.speed, 0.)
        self.assertTrue(result.override)
        rolling = self.call(c, now=time+.1, pose=(x, 0., 0.), actual=-.1,
                            points=((.55-x, 0.),))
        self.assertEqual(rolling.state, 'STOP_HOLD')
        self.assertNotIn('completed', rolling.details)
        self.call(c, now=time+.2, pose=(x, 0., 0.), points=((.55-x, 0.),))
        complete = self.call(c, now=time+.41, pose=(x, 0., 0.), points=((.55-x, 0.),))
        self.assertEqual(complete.state, 'IDLE')
        self.assertFalse(complete.override)
        self.assertTrue(complete.details['completed'])
        next_tick = self.call(c, now=time+.42, pose=(x, 0., 0.),
                              points=((.55-x, 0.),), guard_state='CLEAR')
        self.assertNotIn('completed', next_tick.details)

    def test_spinning_wheels_without_pose_progress_locks(self):
        c, _ = self.start()
        for i in range(1, 12):
            result = self.call(c, now=1.+i*.1, actual=-.25)
            if result.state == 'LOCKED':
                break
        self.assertEqual(result.reason, 'reverse_no_progress')
        self.assertEqual(result.speed, 0.)

    def test_stale_sensor_zero_request_and_manual_cancel_stop_immediately(self):
        for kwargs in [dict(healthy=False), dict(request=0.), dict(permitted=False)]:
            with self.subTest(kwargs=kwargs):
                c, _ = self.start()
                result = self.call(c, now=1.1, **kwargs)
                self.assertTrue(result.override)
                self.assertEqual(result.speed, 0.)
                self.assertEqual(result.state, 'LOCKED')
                # A later manual tick permits normal guarded joystick motion,
                # while an automatic positive request remains locked.
                manual = self.call(c, now=1.2, permitted=False)
                self.assertFalse(manual.override)
                automatic = self.call(c, now=1.3)
                self.assertTrue(automatic.override)
                self.assertEqual(automatic.speed, 0.)

    def test_localization_jump_or_reverse_overspeed_stops_immediately(self):
        for kwargs, reason in [(dict(pose=(-1., 0., 0.), actual=-.25), 'localization_jump'),
                               (dict(actual=-.5), 'reverse_overspeed'),
                               (dict(actual=.1), 'unexpected_forward_motion')]:
            with self.subTest(reason=reason):
                c, _ = self.start()
                result = self.call(c, now=1.1, **kwargs)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.speed, 0.)

    def test_invalid_wait_resets_timer_and_invalid_lock_does_not_pass(self):
        c = RecoveryController()
        self.call(c)
        result = self.call(c, now=.8, pose=(math.nan, 0., 0.), healthy=False)
        self.assertEqual(result.state, 'IDLE')
        self.assertEqual(self.call(c, now=1.).state, 'WAIT_BLOCKED')
        self.assertEqual(self.call(c, now=1.9).state, 'WAIT_BLOCKED')
        self.assertEqual(self.call(c, now=2.1).state, 'REVERSING')
        self.call(c, now=2.2, healthy=False)
        locked = self.call(c, now=2.3, pose=(math.nan, 0., 0.))
        self.assertTrue(locked.override)
        self.assertEqual(locked.speed, 0.)

    def test_disappearing_obstacle_allows_guarded_forward_but_no_new_reversal(self):
        c, first = self.start(grid=self.grid(rear_wall=True))
        self.assertEqual(first.reason, 'no_safe_reverse_candidate')
        for state in ('CLEAR', 'LIMITED', 'RESTARTING'):
            cleared = self.call(c, now=1.1, points=(), grid=self.grid(False),
                                guard_state=state, guard_reason='clear')
            self.assertFalse(cleared.override)
            self.assertEqual(cleared.state, 'LOCKED')
            self.assertEqual(cleared.details['attempts'], 1)
        obstacle_again = self.call(c, now=1.2)
        self.assertTrue(obstacle_again.override)
        # A physical/recovery fault remains zero even if the forward guard
        # reports clear later; only a real manual forward move rearms it.
        d, _ = self.start()
        self.call(d, now=1.1, healthy=False)
        fault = self.call(d, now=1.2, guard_state='CLEAR', points=(), grid=self.grid(False))
        self.assertTrue(fault.override)
        self.assertEqual(fault.speed, 0.)

    def test_attempt_limit_persists_until_actual_forward_distance(self):
        c = RecoveryController(config=RecoveryConfig(max_attempts=1))
        c, _ = self.start(c)
        _, time, x = self.finish_reverse(c)
        self.call(c, now=time+.31, pose=(x, 0., 0.), points=((.55-x, 0.),))
        self.call(c, now=time+.4, pose=(x, 0., 0.), points=((.55-x, 0.),))
        limited = self.call(c, now=time+1.41, pose=(x, 0., 0.), points=((.55-x, 0.),))
        self.assertEqual(limited.reason, 'recovery_attempt_limit')
        # Mode toggling and time passing alone never reset the attempt count.
        self.call(c, now=time+2., pose=(x, 0., 0.), permitted=False)
        locked = self.call(c, now=time+2.1, pose=(x, 0., 0.))
        self.assertEqual(locked.state, 'LOCKED')
        for i in range(1, 107):
            manual = self.call(c, now=time+2.1+.1*i, pose=(x+.01*i, 0., 0.),
                               actual=.1, permitted=False, grid=self.grid(False), points=())
            self.assertFalse(manual.override)
        self.assertEqual(manual.state, 'IDLE')
        self.assertEqual(manual.details['attempts'], 0)

    def test_rotated_grid_matches_known_occupied_not_unknown(self):
        data = np.zeros((10, 10), dtype=np.int8)
        data[3, 2] = 100
        data[3, 4] = -1
        grid = OccupancyGrid(data, .1, (2., -1., math.pi/2))
        xy = np.array([[.25, .35], [.45, .35], [.25, .48], [1.5, 0.]])
        world = xy @ _rotation(math.pi/2).T + np.array([2., -1.])
        self.assertEqual(grid.matches(world, tolerance=.1).tolist(), [True, False, True, False])
        self.assertEqual(grid.contains(world).tolist(), [True, True, True, False])

    def test_batched_map_matches_agree_with_full_occupied_cell_reference(self):
        rng = np.random.default_rng(241)
        data = rng.choice(np.array([-1, 0, 100]), size=(30, 40), p=[.1, .75, .15])
        occupied = np.argwhere(data == 100)[:, ::-1]
        for resolution, tolerance in [(.005, .15), (.05, 0.), (.05, .15), (.005, .5)]:
            grid = OccupancyGrid(data, resolution, (1., -2., .73))
            local = rng.uniform([-.2, -.2], [40*resolution+.2, 30*resolution+.2], (75, 2))
            world = local @ _rotation(.73).T + np.array([1., -2.])
            expected = []
            for p in local:
                q = p/resolution
                distance = np.linalg.norm(np.maximum(np.abs(occupied+.5-q)-.5, 0.)*resolution, axis=1)
                inside = (q >= 0).all() and q[0] < 40 and q[1] < 30
                expected.append(bool(inside and (distance <= tolerance).any()))
            self.assertEqual(grid.matches(world, tolerance).tolist(), expected)

    def test_grid_and_config_reject_invalid_data(self):
        for kwargs in [dict(reverse_speed=float('nan')), dict(reverse_speed=.6),
                       dict(max_attempts=1.5), dict(occupancy_threshold=101),
                       dict(sample_step=0.)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RecoveryConfig(**kwargs)
        for data in [np.array([[101]]), np.array([[-2]]), np.array([[math.nan]])]:
            with self.assertRaises(ValueError):
                OccupancyGrid(data, .05, (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
