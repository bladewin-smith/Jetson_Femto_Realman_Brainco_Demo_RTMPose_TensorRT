"""RTMDet + RTMPose body + dedicated RTMPose hand inference.

The production path uses native TensorRT 10 engines. MMDeploy and PyTorch
remain available as compatibility/reference backends. Imports are lazy so
geometry and depth-selection helpers can be tested without CUDA installed.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from .tensorrt_runtime import TensorRTRuntime


COCO_17_LANDMARKS = {
    "nose": 0,
    "left_eye": 1,
    "right_eye": 2,
    "left_ear": 3,
    "right_ear": 4,
    "left_shoulder": 5,
    "right_shoulder": 6,
    "left_elbow": 7,
    "right_elbow": 8,
    "left_wrist": 9,
    "right_wrist": 10,
    "left_hip": 11,
    "right_hip": 12,
    "left_knee": 13,
    "right_knee": 14,
    "left_ankle": 15,
    "right_ankle": 16,
}

POSE_CONNECTIONS = (
    ("pose_left_shoulder", "pose_right_shoulder"),
    ("pose_left_shoulder", "pose_left_elbow"),
    ("pose_left_elbow", "pose_left_wrist"),
    ("pose_right_shoulder", "pose_right_elbow"),
    ("pose_right_elbow", "pose_right_wrist"),
    ("pose_left_shoulder", "pose_left_hip"),
    ("pose_right_shoulder", "pose_right_hip"),
    ("pose_left_hip", "pose_right_hip"),
    ("pose_left_hip", "pose_left_knee"),
    ("pose_left_knee", "pose_left_ankle"),
    ("pose_right_hip", "pose_right_knee"),
    ("pose_right_knee", "pose_right_ankle"),
)

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)


@dataclass(frozen=True)
class PersonDetection:
    bbox: tuple[int, int, int, int]
    score: float


@dataclass
class DepthPersonCandidate:
    bbox: tuple[int, int, int, int]
    area: int
    median_depth: float
    nearest_depth: float
    centroid: tuple[float, float]
    detection_score: float = 0.0


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _validate_sdk_model_dir(path: str, label: str) -> str:
    model_dir = Path(path).expanduser()
    if not model_dir.is_dir():
        raise FileNotFoundError(f"{label} SDK model directory does not exist: {model_dir}")
    missing = [name for name in ("deploy.json", "pipeline.json") if not (model_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"{label} is not an MMDeploy SDK model directory; missing {', '.join(missing)}: {model_dir}"
        )
    return str(model_dir)


class _MMDeployRuntime:
    def __init__(self, detector_dir: str, body_dir: str, hand_dir: str, device: str, device_id: int):
        try:
            from mmdeploy_runtime import Detector, PoseDetector
        except Exception as exc:
            raise RuntimeError(
                "mmdeploy_runtime is unavailable. Build/install the MMDeploy SDK Python runtime "
                "for this JetPack/TensorRT version."
            ) from exc

        detector_dir = _validate_sdk_model_dir(detector_dir, "RTMDet")
        body_dir = _validate_sdk_model_dir(body_dir, "RTMPose body")
        hand_dir = _validate_sdk_model_dir(hand_dir, "RTMPose hand")
        self.detector = Detector(detector_dir, device_name=device, device_id=int(device_id))
        self.body_pose = PoseDetector(body_dir, device_name=device, device_id=int(device_id))
        self.hand_pose = PoseDetector(hand_dir, device_name=device, device_id=int(device_id))

    def detect(self, image_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        bboxes, labels, _ = self.detector(image_bgr)
        return np.asarray(bboxes, dtype=np.float32), np.asarray(labels, dtype=np.int64)

    def pose(self, kind: str, image_bgr: np.ndarray, bboxes: np.ndarray) -> np.ndarray:
        model = self.body_pose if kind == "body" else self.hand_pose
        poses = model(image_bgr, np.asarray(bboxes, dtype=np.float32).reshape(-1, 4))
        return np.asarray(poses, dtype=np.float32)

    def close(self) -> None:
        # MMDeploy Python objects release their native handles on destruction.
        self.detector = None
        self.body_pose = None
        self.hand_pose = None


class _PyTorchRuntime:
    def __init__(
        self,
        detector_config: str,
        detector_checkpoint: str,
        body_config: str,
        body_checkpoint: str,
        hand_config: str,
        hand_checkpoint: str,
        device: str,
    ):
        required = {
            "rtmdet_config": detector_config,
            "rtmdet_checkpoint": detector_checkpoint,
            "rtmpose_body_config": body_config,
            "rtmpose_body_checkpoint": body_checkpoint,
            "rtmpose_hand_config": hand_config,
            "rtmpose_hand_checkpoint": hand_checkpoint,
        }
        missing = [key for key, value in required.items() if not str(value).strip()]
        if missing:
            raise ValueError("PyTorch runtime requires: " + ", ".join(missing))
        try:
            from mmdet.apis import inference_detector, init_detector
            from mmpose.apis import inference_topdown, init_model
        except Exception as exc:
            raise RuntimeError(
                "PyTorch reference runtime requires compatible mmengine, mmcv, mmdet and mmpose packages."
            ) from exc

        self._inference_detector = inference_detector
        self._inference_topdown = inference_topdown
        self.detector = init_detector(detector_config, detector_checkpoint, device=device)
        self.body_pose = init_model(body_config, body_checkpoint, device=device)
        self.hand_pose = init_model(hand_config, hand_checkpoint, device=device)

    def detect(self, image_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        result = self._inference_detector(self.detector, image_bgr)
        instances = result.pred_instances
        boxes = _as_numpy(instances.bboxes).astype(np.float32)
        scores = _as_numpy(instances.scores).astype(np.float32).reshape(-1, 1)
        labels = _as_numpy(instances.labels).astype(np.int64)
        return np.concatenate((boxes, scores), axis=1), labels

    def pose(self, kind: str, image_bgr: np.ndarray, bboxes: np.ndarray) -> np.ndarray:
        model = self.body_pose if kind == "body" else self.hand_pose
        results = self._inference_topdown(
            model,
            image_bgr,
            bboxes=np.asarray(bboxes, dtype=np.float32).reshape(-1, 4),
        )
        output = []
        for result in results:
            instances = result.pred_instances
            keypoints = _as_numpy(instances.keypoints)
            scores = _as_numpy(instances.keypoint_scores)
            if keypoints.ndim == 3:
                keypoints = keypoints[0]
            if scores.ndim == 2:
                scores = scores[0]
            output.append(np.concatenate((keypoints[:, :2], scores[:, None]), axis=1))
        return np.asarray(output, dtype=np.float32)

    def close(self) -> None:
        self.detector = None
        self.body_pose = None
        self.hand_pose = None


class RTMPoseLandmarkDetector:
    """Expose RTMDet/RTMPose results in the legacy controller's point schema."""

    POSE_LANDMARKS = COCO_17_LANDMARKS
    POSE_CONNECTIONS = POSE_CONNECTIONS
    HAND_CONNECTIONS = HAND_CONNECTIONS

    def __init__(
        self,
        runtime: str = "tensorrt",
        detector_model_dir: str = "",
        body_model_dir: str = "",
        hand_model_dir: str = "",
        detector_engine_path: str = "",
        body_engine_path: str = "",
        hand_engine_path: str = "",
        device_name: str = "cuda",
        device_id: int = 0,
        detector_config: str = "",
        detector_checkpoint: str = "",
        body_config: str = "",
        body_checkpoint: str = "",
        hand_config: str = "",
        hand_checkpoint: str = "",
        person_score_threshold: float = 0.45,
        detector_decode_score_threshold: float = 0.05,
        detector_nms_iou_threshold: float = 0.6,
        detector_max_detections: int = 100,
        pose_bbox_padding: float = 1.25,
        body_keypoint_threshold: float = 0.35,
        hand_keypoint_threshold: float = 0.25,
        hand_roi_scale: float = 2.25,
        hand_roi_wrist_extension: float = 0.35,
        hand_roi_min_size_px: int = 96,
        hand_roi_max_size_px: int = 420,
        hand_inference_interval: int = 1,
        hand_keypoint_hold_frames: int = 2,
        logger: Any = None,
        runtime_impl: Any = None,
        **_ignored: Any,
    ):
        self.logger = logger
        self.runtime_name = str(runtime).strip().lower()
        if runtime_impl is not None:
            self.runtime = runtime_impl
            self.runtime_name = "injected"
        elif self.runtime_name in ("tensorrt", "trt"):
            self.runtime = TensorRTRuntime(
                detector_engine=detector_engine_path or detector_model_dir,
                body_engine=body_engine_path or body_model_dir,
                hand_engine=hand_engine_path or hand_model_dir,
                device_id=device_id,
                detector_score_threshold=detector_decode_score_threshold,
                detector_nms_iou_threshold=detector_nms_iou_threshold,
                detector_max_detections=detector_max_detections,
                pose_bbox_padding=pose_bbox_padding,
                logger=logger,
            )
        elif self.runtime_name == "mmdeploy":
            self.runtime = _MMDeployRuntime(
                detector_model_dir,
                body_model_dir,
                hand_model_dir,
                device_name,
                device_id,
            )
        elif self.runtime_name in ("pytorch", "mmpose"):
            torch_device = f"{device_name}:{device_id}" if device_name == "cuda" else device_name
            self.runtime = _PyTorchRuntime(
                detector_config,
                detector_checkpoint,
                body_config,
                body_checkpoint,
                hand_config,
                hand_checkpoint,
                torch_device,
            )
        else:
            raise ValueError("rtmpose_runtime must be 'tensorrt', 'mmdeploy' or 'pytorch'")

        self.person_score_threshold = max(0.0, min(1.0, float(person_score_threshold)))
        self.body_keypoint_threshold = max(0.0, min(1.0, float(body_keypoint_threshold)))
        self.hand_keypoint_threshold = max(0.0, min(1.0, float(hand_keypoint_threshold)))
        self.hand_roi_scale = max(1.0, float(hand_roi_scale))
        self.hand_roi_wrist_extension = max(-0.25, min(1.5, float(hand_roi_wrist_extension)))
        self.hand_roi_min_size_px = max(32, int(hand_roi_min_size_px))
        self.hand_roi_max_size_px = max(self.hand_roi_min_size_px, int(hand_roi_max_size_px))
        self.hand_inference_interval = max(1, int(hand_inference_interval))
        self.hand_keypoint_hold_frames = max(0, int(hand_keypoint_hold_frames))
        self.frame_index = 0
        self.last_pose_confidences: dict[str, float] = {}
        self.last_hand_confidences: dict[str, float] = {}
        self.last_hand_keypoint_confidences: dict[str, float] = {}
        self._cached_hand_points: dict[str, tuple[int, int]] = {}
        self._hand_keypoint_cache: dict[str, tuple[tuple[int, int], int]] = {}

    def detect_persons(self, image_bgr: np.ndarray) -> list[PersonDetection]:
        bboxes, labels = self.runtime.detect(np.ascontiguousarray(image_bgr))
        detections = []
        for bbox, label in zip(bboxes, labels):
            if int(label) != 0 or len(bbox) < 5:
                continue
            score = float(bbox[4])
            if score < self.person_score_threshold:
                continue
            x0, y0, x1, y1 = self._clamp_bbox(bbox[:4], image_bgr.shape[1], image_bgr.shape[0])
            if x1 - x0 >= 8 and y1 - y0 >= 8:
                detections.append(PersonDetection((x0, y0, x1, y1), score))
        return sorted(detections, key=lambda item: item.score, reverse=True)

    def collect(
        self,
        image_bgr: np.ndarray,
        offset: tuple[int, int] = (0, 0),
        person_bbox: Iterable[float] | None = None,
        include_hands: bool = True,
    ) -> dict[str, tuple[int, int]]:
        height, width = image_bgr.shape[:2]
        bbox = (0, 0, width, height) if person_bbox is None else person_bbox
        bbox = self._clamp_bbox(bbox, width, height)
        self.last_pose_confidences = {}
        self.last_hand_confidences = {}
        self.last_hand_keypoint_confidences = {}
        body = self.runtime.pose("body", np.ascontiguousarray(image_bgr), np.asarray([bbox], np.float32))
        body = self._first_pose(body, expected_keypoints=17)
        if body is None:
            self._cached_hand_points = {}
            self._hand_keypoint_cache = {}
            return {}

        offset_x, offset_y = [int(value) for value in offset]
        points = {}
        for name, index in self.POSE_LANDMARKS.items():
            score = float(body[index, 2])
            self.last_pose_confidences[name] = score
            if score >= self.body_keypoint_threshold:
                points[f"pose_{name}"] = (
                    int(round(float(body[index, 0]))) + offset_x,
                    int(round(float(body[index, 1]))) + offset_y,
                )

        if not include_hands:
            return points

        self.frame_index += 1
        run_hands = (
            (self.frame_index - 1) % self.hand_inference_interval == 0
            or not self._cached_hand_points
        )
        if run_hands:
            hand_points = self._infer_hands(image_bgr, body, offset)
            self._cached_hand_points = hand_points
        else:
            hand_points = dict(self._cached_hand_points)
        points.update(hand_points)
        return points

    def _infer_hands(
        self,
        image_bgr: np.ndarray,
        body: np.ndarray,
        offset: tuple[int, int],
    ) -> dict[str, tuple[int, int]]:
        output = {}
        offset_x, offset_y = [int(value) for value in offset]
        height, width = image_bgr.shape[:2]
        for side in ("left", "right"):
            elbow = body[self.POSE_LANDMARKS[f"{side}_elbow"]]
            wrist = body[self.POSE_LANDMARKS[f"{side}_wrist"]]
            if min(float(elbow[2]), float(wrist[2])) < self.body_keypoint_threshold:
                self._clear_hand_keypoint_cache(side)
                continue
            roi = self.hand_roi_from_arm(elbow[:2], wrist[:2], width, height)
            hand = self.runtime.pose("hand", image_bgr, np.asarray([roi], np.float32))
            hand = self._first_pose(hand, expected_keypoints=21)
            if hand is None:
                self._clear_hand_keypoint_cache(side)
                continue
            valid_scores = hand[:, 2]
            self.last_hand_confidences[side] = float(np.mean(valid_scores))
            for index in range(21):
                key = f"{side}_{index}"
                score = float(hand[index, 2])
                self.last_hand_keypoint_confidences[key] = score
                coordinates = hand[index, :2]
                if score >= self.hand_keypoint_threshold and np.all(np.isfinite(coordinates)):
                    point = (
                        int(round(float(coordinates[0]))) + offset_x,
                        int(round(float(coordinates[1]))) + offset_y,
                    )
                    output[key] = point
                    self._hand_keypoint_cache[key] = (point, self.hand_keypoint_hold_frames)
                    continue
                held_point = self._held_hand_keypoint(key)
                if held_point is not None:
                    output[key] = held_point
        return output

    def _held_hand_keypoint(self, key: str) -> tuple[int, int] | None:
        cached = self._hand_keypoint_cache.get(key)
        if cached is None:
            return None
        point, remaining = cached
        if remaining <= 0:
            self._hand_keypoint_cache.pop(key, None)
            return None
        self._hand_keypoint_cache[key] = (point, remaining - 1)
        return point

    def _clear_hand_keypoint_cache(self, side: str) -> None:
        prefix = f"{side}_"
        for key in [name for name in self._hand_keypoint_cache if name.startswith(prefix)]:
            self._hand_keypoint_cache.pop(key, None)

    def hand_roi_from_arm(
        self,
        elbow_xy: Iterable[float],
        wrist_xy: Iterable[float],
        width: int,
        height: int,
    ) -> tuple[int, int, int, int]:
        elbow = np.asarray(tuple(elbow_xy), dtype=np.float32)
        wrist = np.asarray(tuple(wrist_xy), dtype=np.float32)
        forearm = wrist - elbow
        length = max(1.0, float(np.linalg.norm(forearm)))
        center = wrist + self.hand_roi_wrist_extension * forearm
        size = int(round(np.clip(length * self.hand_roi_scale, self.hand_roi_min_size_px, self.hand_roi_max_size_px)))
        half = 0.5 * size
        return self._clamp_square_bbox(
            (center[0] - half, center[1] - half, center[0] + half, center[1] + half),
            width,
            height,
        )

    @staticmethod
    def _first_pose(poses: np.ndarray, expected_keypoints: int) -> np.ndarray | None:
        poses = np.asarray(poses, dtype=np.float32)
        if poses.size == 0:
            return None
        while poses.ndim > 2 and poses.shape[0] == 1:
            poses = poses[0]
        if poses.ndim != 2 or poses.shape[0] < expected_keypoints or poses.shape[1] < 3:
            return None
        return poses[:expected_keypoints, :3]

    @staticmethod
    def _clamp_bbox(bbox: Iterable[float], width: int, height: int) -> tuple[int, int, int, int]:
        x0, y0, x1, y1 = [int(round(float(value))) for value in bbox]
        return (
            max(0, min(width - 1, x0)),
            max(0, min(height - 1, y0)),
            max(1, min(width, x1)),
            max(1, min(height, y1)),
        )

    @staticmethod
    def _clamp_square_bbox(
        bbox: Iterable[float], width: int, height: int
    ) -> tuple[int, int, int, int]:
        x0, y0, x1, y1 = [float(value) for value in bbox]
        size = min(max(x1 - x0, y1 - y0), float(width), float(height))
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        x0 = min(max(0.0, cx - 0.5 * size), max(0.0, width - size))
        y0 = min(max(0.0, cy - 0.5 * size), max(0.0, height - size))
        return (int(round(x0)), int(round(y0)), int(round(x0 + size)), int(round(y0 + size)))

    def close(self) -> None:
        self.runtime.close()


class RTMDetDepthPersonSelector:
    """Rank RTMDet person boxes using robust Orbbec torso depth."""

    def __init__(self, node: Any, detector: RTMPoseLandmarkDetector):
        self.logger = node.get_logger()
        self.detector = detector
        self.min_depth_m = float(node.declare_parameter("person_min_depth_m", 0.35).value)
        self.max_depth_m = float(node.declare_parameter("person_max_depth_m", 4.0).value)
        self.nearest_percentile = float(node.declare_parameter("person_nearest_percentile", 10.0).value)
        self.nearest_percentile = max(1.0, min(40.0, self.nearest_percentile))
        self.min_area_px = int(node.declare_parameter("person_min_area_px", 2500).value)
        self.roi_margin_px = int(node.declare_parameter("person_roi_margin_px", 30).value)
        self.search_expand_px = int(node.declare_parameter("person_search_expand_px", 160).value)
        self.detector_hz = max(0.5, float(node.declare_parameter("rtmdet_inference_hz", 8.0).value))
        self.torso_top_ratio = float(node.declare_parameter("person_torso_top_ratio", 0.18).value)
        self.torso_bottom_ratio = float(node.declare_parameter("person_torso_bottom_ratio", 0.68).value)
        self.torso_side_ratio = float(node.declare_parameter("person_torso_side_ratio", 0.22).value)
        self.track_min_iou = float(node.declare_parameter("person_track_min_iou", 0.08).value)
        self.last_candidates: list[DepthPersonCandidate] = []
        self._last_detections: list[PersonDetection] = []
        self._last_detection_time = -math.inf

    def find_nearest(
        self,
        depth_meters: np.ndarray,
        search_bbox: Iterable[float] | None = None,
        color_image: np.ndarray | None = None,
    ) -> DepthPersonCandidate | None:
        now = time.monotonic()
        if color_image is not None and now - self._last_detection_time >= 1.0 / self.detector_hz:
            self._last_detections = self.detector.detect_persons(color_image)
            self._last_detection_time = now
        if not self._last_detections:
            self.last_candidates = []
            return None

        candidates = [
            candidate
            for detection in self._last_detections
            for candidate in [self._candidate_from_detection(detection, depth_meters)]
            if candidate is not None
        ]
        if search_bbox is not None:
            tracked = [item for item in candidates if self._iou(item.bbox, search_bbox) >= self.track_min_iou]
            if not tracked:
                self.last_candidates = []
                return None
            candidates = tracked
            candidates.sort(key=lambda item: self._tracking_score(item, search_bbox))
        else:
            candidates.sort(key=lambda item: (item.median_depth, -item.detection_score, -item.area))
        self.last_candidates = candidates
        return candidates[0] if candidates else None

    def _candidate_from_detection(
        self, detection: PersonDetection, depth: np.ndarray
    ) -> DepthPersonCandidate | None:
        height, width = depth.shape[:2]
        x0, y0, x1, y1 = RTMPoseLandmarkDetector._clamp_bbox(detection.bbox, width, height)
        box_width, box_height = x1 - x0, y1 - y0
        if box_width * box_height < self.min_area_px:
            return None
        tx0 = int(round(x0 + self.torso_side_ratio * box_width))
        tx1 = int(round(x1 - self.torso_side_ratio * box_width))
        ty0 = int(round(y0 + self.torso_top_ratio * box_height))
        ty1 = int(round(y0 + self.torso_bottom_ratio * box_height))
        torso = depth[max(0, ty0):min(height, ty1), max(0, tx0):min(width, tx1)]
        valid = np.isfinite(torso) & (torso >= self.min_depth_m) & (torso <= self.max_depth_m)
        if int(np.count_nonzero(valid)) < 20:
            return None
        values = torso[valid]
        return DepthPersonCandidate(
            bbox=(x0, y0, x1, y1),
            area=box_width * box_height,
            median_depth=float(np.median(values)),
            nearest_depth=float(np.percentile(values, self.nearest_percentile)),
            centroid=(0.5 * (x0 + x1), 0.5 * (y0 + y1)),
            detection_score=detection.score,
        )

    def isolate_color_image(
        self, color_image: np.ndarray, _depth: np.ndarray, candidate: DepthPersonCandidate
    ) -> tuple[np.ndarray, tuple[int, int, int, int]]:
        height, width = color_image.shape[:2]
        x0, y0, x1, y1 = candidate.bbox
        roi = RTMPoseLandmarkDetector._clamp_bbox(
            (x0 - self.roi_margin_px, y0 - self.roi_margin_px, x1 + self.roi_margin_px, y1 + self.roi_margin_px),
            width,
            height,
        )
        rx0, ry0, rx1, ry1 = roi
        return color_image[ry0:ry1, rx0:rx1].copy(), roi

    @staticmethod
    def describe(candidate: DepthPersonCandidate | None) -> str:
        if candidate is None:
            return "no candidate"
        x0, y0, x1, y1 = candidate.bbox
        return (
            f"bbox=({x0},{y0},{x1},{y1}), det={candidate.detection_score:.2f}, "
            f"depth={candidate.median_depth:.2f}m"
        )

    @staticmethod
    def _iou(first: Iterable[float], second: Iterable[float]) -> float:
        ax0, ay0, ax1, ay1 = [float(value) for value in first]
        bx0, by0, bx1, by1 = [float(value) for value in second]
        intersection = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(0.0, min(ay1, by1) - max(ay0, by0))
        union = max(1.0, (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - intersection)
        return intersection / union

    @staticmethod
    def _tracking_score(candidate: DepthPersonCandidate, previous_bbox: Iterable[float]) -> float:
        px0, py0, px1, py1 = [float(value) for value in previous_bbox]
        pcx, pcy = 0.5 * (px0 + px1), 0.5 * (py0 + py1)
        return (
            math.hypot(candidate.centroid[0] - pcx, candidate.centroid[1] - pcy)
            + 120.0 * (1.0 - RTMDetDepthPersonSelector._iou(candidate.bbox, previous_bbox))
            + 30.0 * candidate.median_depth
        )
