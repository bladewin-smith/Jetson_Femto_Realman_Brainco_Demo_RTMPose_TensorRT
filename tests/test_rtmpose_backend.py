import unittest

import numpy as np

from vision_backends.rtmpose_backend import (
    PersonDetection,
    RTMDetDepthPersonSelector,
    RTMPoseLandmarkDetector,
)
from vision_backends.tensorrt_runtime import (
    TensorRTRuntime,
    _decode_simcc,
    _nms,
    _prepare_pose_input,
)


class FakeRuntime:
    def __init__(self):
        self.pose_calls = []

    def detect(self, _image):
        return (
            np.asarray([[10, 20, 210, 380, 0.9], [250, 30, 400, 360, 0.2]], np.float32),
            np.asarray([0, 0], np.int64),
        )

    def pose(self, kind, _image, bboxes):
        self.pose_calls.append((kind, np.asarray(bboxes).copy()))
        if kind == "body":
            body = np.zeros((17, 3), np.float32)
            body[:, 0] = np.linspace(80, 160, 17)
            body[:, 1] = np.linspace(60, 300, 17)
            body[:, 2] = 0.95
            body[7, :2] = (90, 180)
            body[9, :2] = (110, 220)
            body[8, :2] = (150, 180)
            body[10, :2] = (170, 220)
            return body[None, ...]
        hand = np.zeros((21, 3), np.float32)
        hand[:, 0] = np.arange(21) + float(bboxes[0, 0])
        hand[:, 1] = np.arange(21) + float(bboxes[0, 1])
        hand[:, 2] = 0.9
        return hand[None, ...]

    def close(self):
        pass


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeNode:
    def get_logger(self):
        return None

    def declare_parameter(self, _name, default):
        return FakeParameter(default)


class FakePersonDetector:
    def detect_persons(self, _image):
        return [
            PersonDetection((10, 0, 110, 200), 0.8),
            PersonDetection((130, 0, 230, 200), 0.95),
        ]


class RTMPoseBackendTests(unittest.TestCase):
    def setUp(self):
        self.runtime = FakeRuntime()
        self.backend = RTMPoseLandmarkDetector(
            runtime_impl=self.runtime,
            person_score_threshold=0.45,
            hand_inference_interval=2,
            hand_roi_min_size_px=80,
            hand_roi_max_size_px=180,
        )
        self.image = np.zeros((400, 480, 3), np.uint8)

    def test_person_threshold_and_mapping(self):
        detections = self.backend.detect_persons(self.image)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].bbox, (10, 20, 210, 380))

        points = self.backend.collect(self.image, person_bbox=detections[0].bbox)
        self.assertIn("pose_left_shoulder", points)
        self.assertIn("pose_right_wrist", points)
        self.assertEqual(sum(key.startswith("left_") for key in points), 21)
        self.assertEqual(sum(key.startswith("right_") for key in points), 21)
        self.assertEqual([kind for kind, _ in self.runtime.pose_calls], ["body", "hand", "hand"])

    def test_hand_roi_is_square_and_inside_image(self):
        roi = self.backend.hand_roi_from_arm((20, 40), (40, 80), 100, 120)
        x0, y0, x1, y1 = roi
        self.assertEqual(x1 - x0, y1 - y0)
        self.assertGreaterEqual(x0, 0)
        self.assertGreaterEqual(y0, 0)
        self.assertLessEqual(x1, 100)
        self.assertLessEqual(y1, 120)

    def test_hand_interval_reuses_one_frame(self):
        self.backend.collect(self.image, person_bbox=(10, 20, 210, 380))
        calls_after_first = len(self.runtime.pose_calls)
        self.backend.collect(self.image, person_bbox=(10, 20, 210, 380))
        self.assertEqual(len(self.runtime.pose_calls), calls_after_first + 1)

    def test_depth_selector_chooses_nearest_and_never_switches_target(self):
        selector = RTMDetDepthPersonSelector(FakeNode(), FakePersonDetector())
        depth = np.full((200, 240), 4.0, np.float32)
        depth[:, 10:110] = 1.0
        depth[:, 130:230] = 2.0
        color = np.zeros((200, 240, 3), np.uint8)

        nearest = selector.find_nearest(depth, color_image=color)
        self.assertEqual(nearest.bbox, (10, 0, 110, 200))
        tracked = selector.find_nearest(
            depth,
            search_bbox=(130, 0, 230, 200),
            color_image=color,
        )
        self.assertEqual(tracked.bbox, (130, 0, 230, 200))
        self.assertIsNone(
            selector.find_nearest(
                depth,
                search_bbox=(0, 0, 5, 5),
                color_image=color,
            )
        )


class TensorRTRuntimeMathTests(unittest.TestCase):
    def test_nms_suppresses_overlapping_box(self):
        boxes = np.asarray(
            [[0, 0, 100, 100], [5, 5, 95, 95], [150, 150, 200, 200]],
            np.float32,
        )
        scores = np.asarray([0.9, 0.8, 0.7], np.float32)
        kept = _nms(boxes, scores, 0.5, 100)
        np.testing.assert_array_equal(kept, np.asarray([0, 2]))

    def test_simcc_decode_uses_axis_argmax_and_lower_score(self):
        simcc_x = np.full((1, 2, 8), -2.0, np.float32)
        simcc_y = np.full((1, 2, 10), -2.0, np.float32)
        simcc_x[0, 0, 3] = 0.8
        simcc_y[0, 0, 7] = 0.6
        simcc_x[0, 1, 4] = 0.5
        simcc_y[0, 1, 2] = 0.9
        coordinates, scores = _decode_simcc(simcc_x, simcc_y)
        np.testing.assert_array_equal(coordinates[0], [[3, 7], [4, 2]])
        np.testing.assert_allclose(scores[0], [0.6, 0.5])

    def test_pose_preprocess_keeps_fixed_nchw_shape(self):
        image = np.zeros((300, 500, 3), np.uint8)
        tensor, center, scale = _prepare_pose_input(
            image, (50, 40, 450, 280), (192, 256), 1.25
        )
        self.assertEqual(tensor.shape, (1, 3, 256, 192))
        np.testing.assert_allclose(center, [250, 160])
        self.assertAlmostEqual(float(scale[0] / scale[1]), 192.0 / 256.0)

    def test_raw_rtmdet_outputs_are_decoded_without_plugins(self):
        boxes = np.asarray([[[1, 2, 3, 4], [5, 6, 7, 8]]], np.float32)
        class_scores = np.asarray([[[0.1, 0.9], [0.8, 0.2]]], np.float32)
        decoded_boxes, scores, labels = TensorRTRuntime._decode_detector_outputs(
            {"boxes": boxes, "scores": class_scores}
        )
        np.testing.assert_array_equal(decoded_boxes, boxes[0])
        np.testing.assert_allclose(scores, [0.9, 0.8])
        np.testing.assert_array_equal(labels, [1, 0])


if __name__ == "__main__":
    unittest.main()
