"""Exercise the real generator entry point without optimization or vehicle access."""

import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml


class StopBeforeOptimization(Exception):
    """The test has reached the geometry input of the actual optimizer."""


class RacelineGeometryTest(unittest.TestCase):
    def run_tumftm_to_optimizer(self, wheelbase, max_steer, overrides=()):
        # Import the actual module: this also catches missing shared helpers.
        from race_stack import gen_raceline_tumftm as generator

        with tempfile.TemporaryDirectory() as tmp:
            overlay = Path(tmp) / 'vehicle.yaml'
            overlay.write_text(yaml.safe_dump({'pure_pursuit': {
                'wheelbase': wheelbase, 'max_steer': max_steer,
            }}))
            track = np.zeros((8, 4))
            argv = ['gen_raceline_tumftm', '--map', 'test_track', *overrides]
            with patch.dict(os.environ, {'RACE_VEHICLE_OVERLAY': str(overlay)}), \
                    patch.object(sys, 'argv', argv), \
                    patch.object(generator, 'WP_DIR', tmp), \
                    patch.object(generator, 'extract_reftrack', return_value=track), \
                    patch.object(generator, 'interp_track', return_value=track), \
                    patch.object(generator, 'generate_raceline_verified',
                                 side_effect=StopBeforeOptimization) as optimize:
                with self.assertRaises(StopBeforeOptimization):
                    generator.main()
                optimize.assert_called_once()
                self.assertFalse((Path(tmp) / 'test_track_wp_tumftm.csv').exists())
                return optimize.call_args.kwargs['kappa_max']

    def test_current_vehicle_geometry_reaches_optimizer(self):
        curvature = self.run_tumftm_to_optimizer(.33, .3903)
        self.assertAlmostEqual(curvature, math.tan(.3903) / .33)

    def test_alternate_vehicle_geometry_reaches_optimizer(self):
        curvature = self.run_tumftm_to_optimizer(.29, .41)
        self.assertAlmostEqual(curvature, math.tan(.41) / .29)

    def test_cli_geometry_override_reaches_optimizer(self):
        curvature = self.run_tumftm_to_optimizer(
            .33, .3903, ['--wheelbase', '.4', '--max-steer', '.25'])
        self.assertAlmostEqual(curvature, math.tan(.25) / .4)

    def test_speed_sweep_respects_cyclic_acceleration_and_braking(self):
        from race_stack.gen_raceline import apply_accel_brake_sweep

        requested = np.array([6., 8., 1., 7., 9., 5., 8., 7.])
        distances = np.array([.4, .2, .3, .5, .25, .45, .15, .35])
        acc, brake = 1.2, 2.3
        original = requested.copy()
        speeds = apply_accel_brake_sweep(requested, distances, acc, brake)

        np.testing.assert_array_equal(requested, original)
        self.assertTrue(np.isfinite(speeds).all())
        self.assertTrue((speeds >= 0).all())
        self.assertTrue((speeds <= requested).all())
        self.assertTrue((speeds < requested).any())
        next_speeds = np.roll(speeds, -1)
        self.assertTrue((next_speeds**2 <= speeds**2 + 2 * acc * distances + 1e-12).all())
        self.assertTrue((speeds**2 <= next_speeds**2 + 2 * brake * distances + 1e-12).all())
        # Rotating a closed track must not change its speed envelope.
        shifted = apply_accel_brake_sweep(
            np.roll(requested, 3), np.roll(distances, 3), acc, brake)
        np.testing.assert_allclose(shifted, np.roll(speeds, 3), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
