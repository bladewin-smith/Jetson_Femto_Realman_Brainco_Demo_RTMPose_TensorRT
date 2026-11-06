"""Temporal stabilizers used by vision-driven dexterous-hand control."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from .calibration import FINGER_NAMES


@dataclass
class AutoZeroHysteresisFilter:
    """Scalar auto-zero plus confirmed Schmitt trigger for a noisy actuator DOF."""

    engage_threshold: float
    release_threshold: float
    confirm_frames: int = 3
    jitter_deadband: float = 0.04
    auto_zero_enabled: bool = True
    auto_zero_samples: int = 5
    auto_zero_margin: float = 0.04
    enabled: bool = True
    baseline: float = 0.0
    active: bool = False
    candidate_frames: int = 0
    last_value: float = 0.0
    samples: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.engage_threshold = float(np.clip(self.engage_threshold, 0.0, 1.0))
        self.release_threshold = float(np.clip(self.release_threshold, 0.0, 1.0))
        self.jitter_deadband = float(np.clip(self.jitter_deadband, 0.0, 1.0))
        self.auto_zero_samples = max(1, int(self.auto_zero_samples))
        self.auto_zero_margin = float(np.clip(self.auto_zero_margin, 0.0, 0.5))
        self.confirm_frames = max(1, int(self.confirm_frames))
        if self.release_threshold >= self.engage_threshold:
            raise ValueError("release_threshold must be lower than engage_threshold")
        self.reset()

    @property
    def baseline_ready(self) -> bool:
        return not self.auto_zero_enabled or len(self.samples) >= self.auto_zero_samples

    def reset(self) -> None:
        self.baseline = 0.0
        self.active = False
        self.candidate_frames = 0
        self.last_value = 0.0
        self.samples = []

    def apply(self, value: float) -> float:
        value = float(np.clip(float(value), 0.0, 1.0))
        if not self.enabled:
            return value

        if self.auto_zero_enabled and not self.baseline_ready:
            self.samples.append(value)
            if self.baseline_ready:
                self.baseline = float(np.median(np.asarray(self.samples, dtype=np.float32)))
            return 0.0

        if self.auto_zero_enabled:
            zero_edge = min(0.95, self.baseline + self.auto_zero_margin)
            value = float(np.clip((value - zero_edge) / (1.0 - zero_edge), 0.0, 1.0))

        if not self.active:
            if value < self.engage_threshold:
                self.candidate_frames = 0
                self.last_value = 0.0
                return 0.0
            self.candidate_frames += 1
            if self.candidate_frames < self.confirm_frames:
                return 0.0
            self.active = True
            self.candidate_frames = 0
        elif value <= self.release_threshold:
            self.active = False
            self.candidate_frames = 0
            self.last_value = 0.0
            return 0.0
        elif value < self.engage_threshold:
            value = self.engage_threshold

        if abs(value - self.last_value) < self.jitter_deadband:
            value = self.last_value
        self.last_value = float(np.clip(value, 0.0, 1.0))
        return self.last_value


@dataclass
class FingerBendHysteresis:
    """Schmitt-trigger filter that keeps an open finger from chattering.

    ``engage_threshold`` is crossed once to enter the bent state.  The finger
    then remains active until it falls below the lower ``release_threshold``.
    Small changes inside an active state are held to suppress actuator chatter.
    """

    engage_threshold: float = 0.12
    release_threshold: float = 0.05
    jitter_deadband: float = 0.025
    active: dict[str, bool] = field(default_factory=dict)
    last_values: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.engage_threshold = float(np.clip(self.engage_threshold, 0.0, 1.0))
        self.release_threshold = float(np.clip(self.release_threshold, 0.0, 1.0))
        self.jitter_deadband = float(np.clip(self.jitter_deadband, 0.0, 1.0))
        if self.release_threshold >= self.engage_threshold:
            raise ValueError(
                "hand_bend_release_threshold must be lower than hand_bend_deadzone"
            )
        self.reset()

    def reset(self) -> None:
        self.active = {name: False for name in FINGER_NAMES}
        self.last_values = {name: 0.0 for name in FINGER_NAMES}

    def configure(
        self,
        engage_threshold: float,
        release_threshold: float,
        jitter_deadband: float,
    ) -> None:
        self.engage_threshold = float(np.clip(engage_threshold, 0.0, 1.0))
        self.release_threshold = float(np.clip(release_threshold, 0.0, 1.0))
        self.jitter_deadband = float(np.clip(jitter_deadband, 0.0, 1.0))
        if self.release_threshold >= self.engage_threshold:
            raise ValueError(
                "hand_bend_release_threshold must be lower than hand_bend_deadzone"
            )
        self.reset()

    def apply(self, bends: Mapping[str, float]) -> dict[str, float]:
        filtered: dict[str, float] = {}
        for name in FINGER_NAMES:
            value = float(np.clip(float(bends.get(name, 0.0)), 0.0, 1.0))
            is_active = self.active[name]
            previous = self.last_values[name]

            if not is_active:
                if value < self.engage_threshold:
                    value = 0.0
                else:
                    self.active[name] = True
            elif value <= self.release_threshold:
                self.active[name] = False
                value = 0.0
            elif value < self.engage_threshold:
                # Stay on the active side of the Schmitt trigger until the
                # lower release threshold is crossed.  Without this clamp an
                # open-looking hand can still alternate between small motor
                # commands on consecutive noisy frames.
                value = self.engage_threshold
            elif abs(value - previous) < self.jitter_deadband:
                value = previous

            self.last_values[name] = value
            filtered[name] = value
        return filtered
