#!/usr/bin/env python3
"""Run RTMDet + RTMPose TensorRT vision without ROS or robot hardware."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parent
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_backends.rtmpose_backend import RTMPoseLandmarkDetector


def _parse_args() -> argparse.Namespace:
    model_root = REPOSITORY_ROOT / "models" / "tensorrt"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        default="0",
        help="image/video path or OpenCV camera index (default: 0)",
    )
    parser.add_argument("--output", type=Path, help="optional annotated image/video path")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--detector-engine",
        type=Path,
        default=model_root / "rtmdet_nano.engine",
    )
    parser.add_argument(
        "--body-engine",
        type=Path,
        default=model_root / "rtmpose_body.engine",
    )
    parser.add_argument(
        "--hand-engine",
        type=Path,
        default=model_root / "rtmpose_hand.engine",
    )
    parser.add_argument("--person-threshold", type=float, default=0.45)
    parser.add_argument("--body-threshold", type=float, default=0.35)
    parser.add_argument("--hand-threshold", type=float, default=0.25)
    parser.add_argument("--nms-iou", type=float, default=0.60)
    parser.add_argument("--hand-interval", type=int, default=1)
    parser.add_argument("--device-id", type=int, default=0)
    return parser.parse_args()


def _draw_points(frame, detector, points, bbox, score, fps):
    x0, y0, x1, y1 = bbox
    cv2.rectangle(frame, (x0, y0), (x1, y1), (20, 220, 20), 2)
    cv2.putText(
        frame,
        f"person {score:.2f}  {fps:.1f} FPS",
        (x0, max(20, y0 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (20, 220, 20),
        2,
        cv2.LINE_AA,
    )
    for first, second in detector.POSE_CONNECTIONS:
        if first in points and second in points:
            cv2.line(frame, points[first], points[second], (0, 210, 255), 2)
    for side, color in (("left", (255, 80, 40)), ("right", (40, 80, 255))):
        for first, second in detector.HAND_CONNECTIONS:
            first_name, second_name = f"{side}_{first}", f"{side}_{second}"
            if first_name in points and second_name in points:
                cv2.line(frame, points[first_name], points[second_name], color, 2)
    for name, point in points.items():
        if name.startswith("pose_"):
            color = (0, 255, 255)
        elif name.startswith("left_"):
            color = (255, 80, 40)
        else:
            color = (40, 80, 255)
        cv2.circle(frame, point, 3, color, -1, cv2.LINE_AA)


def _image_input(path_text: str):
    path = Path(path_text).expanduser()
    if not path.is_file():
        return None
    image = cv2.imread(str(path))
    return image


def main() -> int:
    args = _parse_args()
    detector = RTMPoseLandmarkDetector(
        runtime="tensorrt",
        detector_engine_path=str(args.detector_engine),
        body_engine_path=str(args.body_engine),
        hand_engine_path=str(args.hand_engine),
        device_id=args.device_id,
        person_score_threshold=args.person_threshold,
        detector_nms_iou_threshold=args.nms_iou,
        body_keypoint_threshold=args.body_threshold,
        hand_keypoint_threshold=args.hand_threshold,
        hand_inference_interval=args.hand_interval,
    )
    still_image = _image_input(args.input)
    capture = None
    writer = None
    try:
        if still_image is None:
            source = int(args.input) if args.input.lstrip("-").isdigit() else args.input
            capture = cv2.VideoCapture(source)
            if not capture.isOpened():
                raise RuntimeError(f"Could not open video/camera input: {args.input}")

        fps = 0.0
        previous_time = time.perf_counter()
        while True:
            if still_image is not None:
                frame = still_image.copy()
                ok = True
            else:
                ok, frame = capture.read()
            if not ok:
                break

            detections = detector.detect_persons(frame)
            if detections:
                person = detections[0]
                points = detector.collect(frame, person_bbox=person.bbox, include_hands=True)
                now = time.perf_counter()
                instant_fps = 1.0 / max(now - previous_time, 1e-6)
                fps = instant_fps if fps == 0.0 else 0.9 * fps + 0.1 * instant_fps
                previous_time = now
                _draw_points(frame, detector, points, person.bbox, person.score, fps)
            else:
                previous_time = time.perf_counter()
                cv2.putText(
                    frame,
                    "no person",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                if still_image is not None:
                    if not cv2.imwrite(str(args.output), frame):
                        raise RuntimeError(f"Could not write image: {args.output}")
                else:
                    if writer is None:
                        source_fps = capture.get(cv2.CAP_PROP_FPS)
                        source_fps = source_fps if source_fps > 0 else 20.0
                        writer = cv2.VideoWriter(
                            str(args.output),
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            source_fps,
                            (frame.shape[1], frame.shape[0]),
                        )
                        if not writer.isOpened():
                            raise RuntimeError(f"Could not open output video: {args.output}")
                    writer.write(frame)

            if not args.no_display:
                cv2.imshow("RTMDet + RTMPose TensorRT", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            if still_image is not None:
                break
    finally:
        if capture is not None:
            capture.release()
        if writer is not None:
            writer.release()
        detector.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
