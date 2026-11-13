"""Vehicle selection and ROS workspace regression checks; no vehicle commands."""

import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from race_stack.steering_geometry import load_steering_geometry


class SteeringGeometryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def overlay(self, name, wheelbase, max_steer):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump({'pure_pursuit': {
            'wheelbase': wheelbase, 'max_steer': max_steer}}))
        return path

    def test_explicit_vehicle_beats_repository(self):
        self.overlay('repo/config/.vehicle_overlay.yaml', .33, .3903)
        alternate = self.overlay('car2.yaml', .31, .4)
        with patch.dict(os.environ, {
            'RACE_VEHICLE_OVERLAY': str(alternate),
            'RACE_REPO': str(self.root / 'repo'),
        }, clear=True):
            self.assertEqual(load_steering_geometry(), (.31, .4))

    def test_workspace_does_not_hide_repository(self):
        self.overlay('repo/config/.vehicle_overlay.yaml', .33, .3903)
        with patch.dict(os.environ, {
            'RACE_REPO': str(self.root / 'repo'),
            'RACE_WS': str(self.root / 'different_ros_workspace'),
        }, clear=True):
            self.assertEqual(load_steering_geometry(), (.33, .3903))

    def test_script_selects_repository(self):
        self.overlay('repo/config/.vehicle_overlay.yaml', .33, .3903)
        with patch.dict(os.environ, {
            'RACE_SCRIPT': str(self.root / 'repo/race_script.yaml'),
        }, clear=True):
            self.assertEqual(load_steering_geometry(), (.33, .3903))

    def test_missing_selected_vehicle_does_not_use_another_car(self):
        with patch.dict(os.environ, {
            'RACE_VEHICLE_OVERLAY': str(self.root / 'missing.yaml'),
        }, clear=True):
            with self.assertRaises(FileNotFoundError):
                load_steering_geometry()

    def test_invalid_geometry_is_rejected(self):
        for wheelbase, max_steer in [(0, .4), (.33, math.nan), (.33, 2)]:
            with self.subTest(wheelbase=wheelbase, max_steer=max_steer):
                path = self.overlay('bad.yaml', wheelbase, max_steer)
                with patch.dict(os.environ, {
                    'RACE_VEHICLE_OVERLAY': str(path),
                }, clear=True):
                    with self.assertRaises(ValueError):
                        load_steering_geometry()


if __name__ == '__main__':
    unittest.main()
