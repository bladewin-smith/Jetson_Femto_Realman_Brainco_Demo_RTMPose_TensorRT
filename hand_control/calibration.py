"""Calibration and geometry helpers for vision-driven dexterous-hand control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def _five_floats(values: Sequence[float], label: str) -> tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != len(FINGER_NAMES):
        raise ValueError(f"{label} must contain five values in {FINGER_NAMES} order")
    if not all(np.isfinite(value) for value in converted):
        raise ValueError(f"{label} must contain only finite values")
    return converted


@dataclass(frozen=True)
class FingerBendCalibration:
    """Map operator-specific raw bend values onto a stable zero-to-one range."""

    open_values: tuple[float, ...]
    closed_values: tuple[float, ...]
    gamma_values: tuple[float, ...]
    enabled: bool = True
    minimum_span: float = 0.05

    @classmethod
    def create(
        cls,
        open_values: Sequence[float],
        closed_values: Sequence[float],
        gamma_values: Sequence[float],
        enabled: bool = True,
        minimum_span: float = 0.05,
    ) -> "FingerBendCalibration":
        open_tuple = _five_floats(open_values, "hand_bend_open_calibration")
        closed_tuple = _five_floats(closed_values, "hand_bend_closed_calibration")
        gamma_tuple = _five_floats(gamma_values, "hand_bend_gamma")
        minimum_span = max(1e-6, float(minimum_span))
        for name, opened, closed, gamma in zip(
            FINGER_NAMES,
            open_tuple,
            closed_tuple,
            gamma_tuple,
        ):
            if closed - opened < minimum_span:
                raise ValueError(
                    f"{name} closed calibration must exceed open calibration by "
                    f"at least {minimum_span:.3f}"
                )
            if gamma <= 0.0:
                raise ValueError(f"{name} hand_bend_gamma must be greater than zero")
        return cls(
            open_values=open_tuple,
            closed_values=closed_tuple,
            gamma_values=gamma_tuple,
            enabled=bool(enabled),
            minimum_span=minimum_span,
        )

    def apply(self, bends: Mapping[str, float]) -> dict[str, float]:
        calibrated = dict(bends)
        if not self.enabled:
            return calibrated
        for name, opened, closed, gamma in zip(
            FINGER_NAMES,
            self.open_values,
            self.closed_values,
            self.gamma_values,
        ):
            raw_value = float(bends.get(name, opened))
            normalized = np.clip((raw_value - opened) / (closed - opened), 0.0, 1.0)
            calibrated[name] = float(normalized**gamma)
        return calibrated

    def describe(self) -> str:
        pairs = ", ".join(
            f"{name}={opened:.3f}->{closed:.3f},gamma={gamma:.2f}"
            for name, opened, closed, gamma in zip(
                FINGER_NAMES,
                self.open_values,
                self.closed_values,
                self.gamma_values,
            )
        )
        return f"enabled={self.enabled}; {pairs}"


def thumb_opposition(
    hand_points: np.ndarray,
    open_ratio: float = 1.05,
    closed_ratio: float = 0.35,
    gamma: float = 1.0,
) -> float | None:
    """Estimate thumb opposition independently from thumb joint flexion."""

    points = np.asarray(hand_points, dtype=np.float32)
    if points.ndim != 2 or points.shape[0] < 21 or points.shape[1] not in (2, 3):
        return None
    if not np.all(np.isfinite(points[[4, 5, 17]])):
        return None
    palm_width = float(np.linalg.norm(points[5] - points[17]))
    if palm_width < 1e-6:
        return None
    open_ratio = float(open_ratio)
    closed_ratio = float(closed_ratio)
    gamma = float(gamma)
    if open_ratio <= closed_ratio or gamma <= 0.0:
        return None
    tip_ratio = float(np.linalg.norm(points[4] - points[5])) / palm_width
    normalized = np.clip(
        (open_ratio - tip_ratio) / (open_ratio - closed_ratio),
        0.0,
        1.0,
    )
    return float(normalized**gamma)
