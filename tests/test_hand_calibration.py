import unittest

import numpy as np

from hand_control import (
    AutoZeroHysteresisFilter,
    FingerBendCalibration,
    FingerBendHysteresis,
    thumb_opposition,
)


class FingerBendCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.calibration = FingerBendCalibration.create(
            [0.18, 0.08, 0.13, 0.09, 0.13],
            [0.56, 0.92, 0.91, 0.86, 0.87],
            [1.0, 1.0, 1.0, 1.0, 1.0],
        )

    def test_open_and_closed_samples_map_to_endpoints(self):
        opened = self.calibration.apply(
            dict(zip(("thumb", "index", "middle", "ring", "pinky"), self.calibration.open_values))
        )
        closed = self.calibration.apply(
            dict(zip(("thumb", "index", "middle", "ring", "pinky"), self.calibration.closed_values))
        )
        np.testing.assert_allclose(list(opened.values()), np.zeros(5), atol=1e-6)
        np.testing.assert_allclose(list(closed.values()), np.ones(5), atol=1e-6)

    def test_gamma_shapes_midpoint_without_changing_endpoints(self):
        calibration = FingerBendCalibration.create(
            [0.0] * 5,
            [1.0] * 5,
            [2.0] * 5,
        )
        mapped = calibration.apply({name: 0.5 for name in calibration.apply({}).keys()})
        np.testing.assert_allclose(list(mapped.values()), np.full(5, 0.25), atol=1e-6)

    def test_invalid_calibration_span_is_rejected(self):
        with self.assertRaises(ValueError):
            FingerBendCalibration.create([0.2] * 5, [0.2] * 5, [1.0] * 5)

    def test_thumb_opposition_is_independent_from_joint_flexion(self):
        points = np.zeros((21, 2), np.float32)
        points[5] = (0.0, 0.0)
        points[17] = (1.0, 0.0)
        points[4] = (1.05, 0.0)
        self.assertAlmostEqual(thumb_opposition(points), 0.0, places=5)
        points[4] = (0.35, 0.0)
        self.assertAlmostEqual(thumb_opposition(points), 1.0, places=5)


class FingerBendHysteresisTests(unittest.TestCase):
    def test_open_noise_does_not_move_the_finger(self):
        stabilizer = FingerBendHysteresis(0.12, 0.05, 0.025)
        for value in (0.03, 0.08, 0.11, 0.07):
            output = stabilizer.apply({"index": value})
            self.assertEqual(output["index"], 0.0)

    def test_bend_engages_quickly_and_uses_lower_release_threshold(self):
        stabilizer = FingerBendHysteresis(0.12, 0.05, 0.025)
        self.assertAlmostEqual(stabilizer.apply({"index": 0.40})["index"], 0.40)
        self.assertAlmostEqual(stabilizer.apply({"index": 0.08})["index"], 0.12)
        self.assertEqual(stabilizer.apply({"index": 0.04})["index"], 0.0)

    def test_small_active_jitter_is_held(self):
        stabilizer = FingerBendHysteresis(0.12, 0.05, 0.03)
        self.assertAlmostEqual(stabilizer.apply({"middle": 0.50})["middle"], 0.50)
        self.assertAlmostEqual(stabilizer.apply({"middle": 0.52})["middle"], 0.50)


class AutoZeroHysteresisFilterTests(unittest.TestCase):
    def test_straight_baseline_and_noise_are_locked_to_zero(self):
        stabilizer = AutoZeroHysteresisFilter(
            engage_threshold=0.22,
            release_threshold=0.08,
            confirm_frames=3,
            auto_zero_samples=5,
            auto_zero_margin=0.04,
        )
        for value in (0.16, 0.18, 0.17, 0.19, 0.18):
            self.assertEqual(stabilizer.apply(value), 0.0)
        self.assertTrue(stabilizer.baseline_ready)
        self.assertAlmostEqual(stabilizer.baseline, 0.18, places=5)
        for value in (0.19, 0.21, 0.20, 0.23):
            self.assertEqual(stabilizer.apply(value), 0.0)

    def test_real_motion_needs_confirmation_then_releases_at_lower_threshold(self):
        stabilizer = AutoZeroHysteresisFilter(
            engage_threshold=0.25,
            release_threshold=0.10,
            confirm_frames=3,
            jitter_deadband=0.04,
            auto_zero_enabled=False,
        )
        self.assertEqual(stabilizer.apply(0.60), 0.0)
        self.assertEqual(stabilizer.apply(0.60), 0.0)
        self.assertAlmostEqual(stabilizer.apply(0.60), 0.60)
        self.assertAlmostEqual(stabilizer.apply(0.58), 0.60)
        self.assertAlmostEqual(stabilizer.apply(0.20), 0.25)
        self.assertEqual(stabilizer.apply(0.08), 0.0)


if __name__ == "__main__":
    unittest.main()
