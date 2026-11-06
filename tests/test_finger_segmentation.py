import unittest

import cv2
import numpy as np

from vision_backends.finger_segmentation import (
    AdaptiveFingerSegmenter,
    apply_straight_finger_veto,
)


class FingerSegmentationTests(unittest.TestCase):
    SKIN = (80, 140, 190)

    @staticmethod
    def _points():
        points = {
            0: (100, 175),
            5: (72, 116),
            6: (72, 94),
            9: (92, 109),
            10: (92, 87),
            13: (112, 112),
            14: (112, 91),
            17: (132, 120),
            18: (132, 101),
        }
        return {f"right_{index}": value for index, value in points.items()}

    def _hand_image(self, middle_extended=True):
        image = np.full((220, 210, 3), 245, np.uint8)
        palm = np.asarray(
            [(100, 175), (62, 112), (72, 102), (132, 106), (142, 122)],
            np.int32,
        )
        cv2.fillConvexPoly(image, palm, self.SKIN)
        cv2.rectangle(image, (66, 42), (78, 119), self.SKIN, -1)
        middle_top = 34 if middle_extended else 88
        cv2.rectangle(image, (86, middle_top), (98, 112), self.SKIN, -1)
        cv2.rectangle(image, (106, 45), (118, 115), self.SKIN, -1)
        cv2.rectangle(image, (126, 65), (138, 123), self.SKIN, -1)
        return image

    def test_adjacent_fingers_get_independent_extension_measurements(self):
        segmenter = AdaptiveFingerSegmenter(corridor_width_ratio=0.10)
        points = self._points()
        open_evidence = segmenter.analyze(self._hand_image(True), points)
        self.assertIn("right", open_evidence)
        bent_evidence = segmenter.analyze(self._hand_image(False), points)["right"]
        self.assertGreater(bent_evidence["index"][0], 0.80)
        self.assertLess(bent_evidence["middle"][0], 0.70)

    def test_straight_veto_only_changes_the_supported_finger(self):
        bends = {"thumb": 0.1, "index": 0.8, "middle": 0.8, "ring": 0.1, "pinky": 0.1}
        evidence = {
            "index": (0.45, 0.9),
            "middle": (0.96, 0.9),
        }
        corrected, names = apply_straight_finger_veto(bends, evidence)
        self.assertAlmostEqual(corrected["index"], 0.8)
        self.assertLessEqual(corrected["middle"], 0.12)
        self.assertEqual(names, ("middle",))

    def test_low_confidence_mask_cannot_override_pose(self):
        bends = {"index": 0.7}
        corrected, names = apply_straight_finger_veto(
            bends,
            {"index": (1.0, 0.2)},
        )
        self.assertAlmostEqual(corrected["index"], 0.7)
        self.assertEqual(names, ())


if __name__ == "__main__":
    unittest.main()
