"""Independent finger-silhouette evidence for top-down hand pose control.

The RTMPose hand network predicts all 21 landmarks jointly.  When adjacent
fingers touch, that joint prior can incorrectly curl both fingers together.
This module adds an image-only, per-finger straightness veto: it learns the
palm chroma in each frame, extracts the connected hand silhouette, and measures
skin extension in a narrow corridor aligned with each finger's proximal bone.
"""

from __future__ import annotations

from typing import Mapping

import cv2
import numpy as np


FINGER_BASE_PIP = {
    "index": (5, 6),
    "middle": (9, 10),
    "ring": (13, 14),
    "pinky": (17, 18),
}
NOMINAL_EXTENSION_RATIOS = {
    "index": 1.00,
    "middle": 1.10,
    "ring": 1.00,
    "pinky": 0.78,
}


def apply_straight_finger_veto(
    bends: Mapping[str, float],
    evidence: Mapping[str, tuple[float, float]],
    *,
    min_confidence: float = 0.45,
    straight_threshold: float = 0.78,
    full_straight_threshold: float = 0.92,
    bend_cap: float = 0.12,
) -> tuple[dict[str, float], tuple[str, ...]]:
    """Reduce only fingers whose independent silhouette is clearly straight."""

    result = {name: float(value) for name, value in bends.items()}
    corrected: list[str] = []
    min_confidence = float(np.clip(min_confidence, 0.0, 1.0))
    straight_threshold = float(np.clip(straight_threshold, 0.0, 0.99))
    full_straight_threshold = float(
        np.clip(full_straight_threshold, straight_threshold + 1e-6, 1.0)
    )
    bend_cap = float(np.clip(bend_cap, 0.0, 1.0))

    for name in FINGER_BASE_PIP:
        measurement = evidence.get(name)
        if measurement is None:
            continue
        extension, confidence = [float(value) for value in measurement]
        if confidence < min_confidence or extension < straight_threshold:
            continue
        strength = float(
            np.clip(
                (extension - straight_threshold)
                / (full_straight_threshold - straight_threshold),
                0.0,
                1.0,
            )
        )
        original = float(np.clip(result.get(name, 0.0), 0.0, 1.0))
        capped = min(original, bend_cap)
        corrected_value = (1.0 - strength) * original + strength * capped
        if corrected_value + 1e-6 < original:
            corrected.append(name)
        result[name] = corrected_value
    return result, tuple(corrected)


class AdaptiveFingerSegmenter:
    """Measure each non-thumb finger in its own narrow silhouette corridor."""

    PALM_INDEXES = (0, 5, 9, 13, 17)

    def __init__(
        self,
        *,
        corridor_width_ratio: float = 0.11,
        color_mad_scale: float = 4.0,
        minimum_chroma_tolerance: float = 9.0,
        minimum_palm_pixels: int = 80,
    ) -> None:
        self.corridor_width_ratio = float(
            np.clip(corridor_width_ratio, 0.05, 0.25)
        )
        self.color_mad_scale = max(1.0, float(color_mad_scale))
        self.minimum_chroma_tolerance = max(
            2.0,
            float(minimum_chroma_tolerance),
        )
        self.minimum_palm_pixels = max(20, int(minimum_palm_pixels))
        self.extension_references: dict[tuple[str, str], float] = {}
        self.last_evidence: dict[str, dict[str, tuple[float, float]]] = {}
        self.last_debug: list[tuple[tuple[int, int], tuple[int, int], str, float]] = []

    def reset(self) -> None:
        self.extension_references = {}
        self.last_evidence = {}
        self.last_debug = []

    def analyze(
        self,
        image_bgr: np.ndarray,
        points: Mapping[str, tuple[int, int]],
        offset: tuple[int, int] = (0, 0),
    ) -> dict[str, dict[str, tuple[float, float]]]:
        self.last_evidence = {}
        self.last_debug = []
        if image_bgr is None or image_bgr.ndim != 3:
            return self.last_evidence
        for side in ("left", "right"):
            side_evidence = self._analyze_side(image_bgr, points, side, offset)
            if side_evidence:
                self.last_evidence[side] = side_evidence
        return self.last_evidence

    def _analyze_side(
        self,
        image_bgr: np.ndarray,
        points: Mapping[str, tuple[int, int]],
        side: str,
        offset: tuple[int, int],
    ) -> dict[str, tuple[float, float]]:
        required = set(self.PALM_INDEXES)
        for base_index, pip_index in FINGER_BASE_PIP.values():
            required.update((base_index, pip_index))
        if any(f"{side}_{index}" not in points for index in required):
            return {}

        offset_xy = np.asarray(offset, dtype=np.float32)
        hand_points = {
            index: np.asarray(points[f"{side}_{index}"], dtype=np.float32) - offset_xy
            for index in required
        }
        palm_width = float(np.linalg.norm(hand_points[5] - hand_points[17]))
        if not np.isfinite(palm_width) or palm_width < 12.0:
            return {}

        height, width = image_bgr.shape[:2]
        all_points = np.asarray(list(hand_points.values()), dtype=np.float32)
        # Expand in every direction because the operator may rotate the hand;
        # assuming that fingers always point upward would reintroduce a strong
        # pose-dependent coupling error.
        margin = max(12.0, 1.8 * palm_width)
        x0 = max(0, int(np.floor(np.min(all_points[:, 0]) - margin)))
        y0 = max(0, int(np.floor(np.min(all_points[:, 1]) - margin)))
        x1 = min(width, int(np.ceil(np.max(all_points[:, 0]) + margin)))
        y1 = min(height, int(np.ceil(np.max(all_points[:, 1]) + margin)))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return {}

        crop = image_bgr[y0:y1, x0:x1]
        crop_origin = np.asarray((x0, y0), dtype=np.float32)
        palm_polygon = np.asarray(
            [hand_points[index] - crop_origin for index in self.PALM_INDEXES],
            dtype=np.int32,
        )
        seed_mask = np.zeros(crop.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(seed_mask, cv2.convexHull(palm_polygon), 255)
        if int(np.count_nonzero(seed_mask)) < self.minimum_palm_pixels:
            return {}

        hand_mask, palm_coverage = self._adaptive_hand_mask(crop, seed_mask)
        if hand_mask is None:
            return {}
        hand_mask = self._component_connected_to_palm(hand_mask, seed_mask)
        if hand_mask is None:
            return {}

        mask_y, mask_x = np.nonzero(hand_mask)
        if mask_x.size < self.minimum_palm_pixels:
            return {}
        mask_points = np.column_stack((mask_x + x0, mask_y + y0)).astype(np.float32)
        corridor_half_width = max(2.0, palm_width * self.corridor_width_ratio)
        max_projection = 1.8 * palm_width
        evidence: dict[str, tuple[float, float]] = {}

        for name, (base_index, pip_index) in FINGER_BASE_PIP.items():
            base = hand_points[base_index]
            direction = hand_points[pip_index] - base
            norm = float(np.linalg.norm(direction))
            if norm < 2.0:
                direction = base - hand_points[0]
                norm = float(np.linalg.norm(direction))
            if norm < 2.0:
                continue
            direction /= norm
            relative = mask_points - base
            projection = relative @ direction
            lateral = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
            in_corridor = (
                (projection >= 0.0)
                & (projection <= max_projection)
                & (lateral <= corridor_half_width)
            )
            corridor_projection = projection[in_corridor]
            if corridor_projection.size < 8:
                continue
            extension_length = float(np.percentile(corridor_projection, 97.0))
            observed_ratio = extension_length / palm_width
            nominal_ratio = NOMINAL_EXTENSION_RATIOS[name]
            reference_key = (side, name)
            reference_ratio = max(
                nominal_ratio,
                self.extension_references.get(reference_key, nominal_ratio),
            )
            support = min(
                1.0,
                float(corridor_projection.size) / max(8.0, 0.08 * palm_width * palm_width),
            )
            confidence = float(np.clip(palm_coverage * support, 0.0, 1.0))
            if confidence >= 0.35 and observed_ratio > reference_ratio:
                reference_ratio = min(observed_ratio, 1.8)
                self.extension_references[reference_key] = reference_ratio
            else:
                self.extension_references.setdefault(reference_key, reference_ratio)
            extension = float(
                np.clip(observed_ratio / max(reference_ratio, 1e-6), 0.0, 1.0)
            )
            evidence[name] = (extension, confidence)

            debug_start = tuple(int(round(value)) for value in base + offset_xy)
            debug_end = tuple(
                int(round(value))
                for value in base + direction * extension_length + offset_xy
            )
            self.last_debug.append((debug_start, debug_end, name, extension))
        return evidence

    def _adaptive_hand_mask(
        self,
        crop_bgr: np.ndarray,
        seed_mask: np.ndarray,
    ) -> tuple[np.ndarray | None, float]:
        ycrcb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2YCrCb).astype(np.float32)
        samples = ycrcb[seed_mask > 0]
        if samples.shape[0] < self.minimum_palm_pixels:
            return None, 0.0
        median = np.median(samples, axis=0)
        mad = np.median(np.abs(samples - median), axis=0)
        tolerance = np.maximum(
            np.asarray((35.0, self.minimum_chroma_tolerance, self.minimum_chroma_tolerance)),
            self.color_mad_scale * 1.4826 * mad,
        )
        tolerance = np.minimum(tolerance, np.asarray((80.0, 38.0, 38.0)))
        normalized = np.abs(ycrcb - median) / np.maximum(tolerance, 1.0)
        mask = np.all(normalized <= 1.0, axis=2).astype(np.uint8) * 255
        kernel_size = max(3, int(round(np.sqrt(np.count_nonzero(seed_mask)) * 0.025)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel_size = min(kernel_size, 7)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        palm_pixels = seed_mask > 0
        palm_coverage = float(np.mean(mask[palm_pixels] > 0))
        if palm_coverage < 0.45:
            return None, palm_coverage
        return mask, palm_coverage

    @staticmethod
    def _component_connected_to_palm(
        mask: np.ndarray,
        seed_mask: np.ndarray,
    ) -> np.ndarray | None:
        count, labels = cv2.connectedComponents(mask, connectivity=8)
        if count <= 1:
            return None
        seed_labels = labels[seed_mask > 0]
        seed_labels = seed_labels[seed_labels > 0]
        if seed_labels.size == 0:
            return None
        label_counts = np.bincount(seed_labels, minlength=count)
        selected = int(np.argmax(label_counts[1:]) + 1)
        return (labels == selected).astype(np.uint8)

    def draw_debug(self, image_bgr: np.ndarray) -> None:
        for start, end, name, extension in self.last_debug:
            color = (0, 220, 0) if extension >= 0.78 else (0, 140, 255)
            cv2.line(image_bgr, start, end, color, 2, cv2.LINE_AA)
            cv2.putText(
                image_bgr,
                f"{name[0].upper()}:{extension:.2f}",
                end,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )
