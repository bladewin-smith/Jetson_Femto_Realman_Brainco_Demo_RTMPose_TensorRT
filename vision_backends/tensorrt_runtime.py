"""Native TensorRT 10 runtime for RTMDet and RTMPose.

TensorRT and CUDA Python are imported lazily so preprocessing and unit tests
remain usable on development machines without a Jetson GPU.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np


_DETECTOR_MEAN_BGR = np.asarray((103.53, 116.28, 123.675), dtype=np.float32)
_DETECTOR_STD_BGR = np.asarray((57.375, 57.12, 58.395), dtype=np.float32)
_POSE_MEAN_RGB = np.asarray((123.675, 116.28, 103.53), dtype=np.float32)
_POSE_STD_RGB = np.asarray((58.395, 57.12, 57.375), dtype=np.float32)


def _log(logger: Any, level: str, message: str) -> None:
    if logger is None:
        return
    method = getattr(logger, level, None) or getattr(logger, "info", None)
    if method is not None:
        method(message)


def _resolve_engine_path(path: str, label: str) -> Path:
    candidate = Path(str(path)).expanduser()
    if candidate.is_dir():
        for name in ("end2end.engine", "model.engine", f"{candidate.name}.engine"):
            engine = candidate / name
            if engine.is_file():
                return engine.resolve()
        raise FileNotFoundError(
            f"{label} directory does not contain end2end.engine or model.engine: {candidate}"
        )
    if not candidate.is_file():
        raise FileNotFoundError(f"{label} TensorRT engine does not exist: {candidate}")
    return candidate.resolve()


def _nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float,
    max_detections: int,
) -> np.ndarray:
    """Return score-sorted indices kept by class-agnostic NMS."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    if boxes.shape[0] == 0:
        return np.empty((0,), dtype=np.int64)
    x0, y0, x1, y1 = boxes.T
    areas = np.maximum(0.0, x1 - x0) * np.maximum(0.0, y1 - y0)
    order = np.argsort(scores)[::-1]
    keep: list[int] = []
    while order.size and len(keep) < max(1, int(max_detections)):
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break
        rest = order[1:]
        ix0 = np.maximum(x0[index], x0[rest])
        iy0 = np.maximum(y0[index], y0[rest])
        ix1 = np.minimum(x1[index], x1[rest])
        iy1 = np.minimum(y1[index], y1[rest])
        intersection = np.maximum(0.0, ix1 - ix0) * np.maximum(0.0, iy1 - iy0)
        union = np.maximum(areas[index] + areas[rest] - intersection, 1e-6)
        order = rest[intersection / union <= float(iou_threshold)]
    return np.asarray(keep, dtype=np.int64)


def _decode_simcc(simcc_x: np.ndarray, simcc_y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decode raw SimCC axes into split-space coordinates and scores."""
    simcc_x = np.asarray(simcc_x, dtype=np.float32)
    simcc_y = np.asarray(simcc_y, dtype=np.float32)
    if simcc_x.ndim == 2:
        simcc_x = simcc_x[None]
    if simcc_y.ndim == 2:
        simcc_y = simcc_y[None]
    if simcc_x.ndim != 3 or simcc_y.ndim != 3 or simcc_x.shape[:2] != simcc_y.shape[:2]:
        raise ValueError(f"Invalid SimCC output shapes: x={simcc_x.shape}, y={simcc_y.shape}")
    x_index = np.argmax(simcc_x, axis=-1)
    y_index = np.argmax(simcc_y, axis=-1)
    coordinates = np.stack((x_index, y_index), axis=-1).astype(np.float32)
    scores = np.minimum(np.max(simcc_x, axis=-1), np.max(simcc_y, axis=-1))
    coordinates[scores <= 0.0] = -1.0
    return coordinates, scores.astype(np.float32)


def _bbox_center_scale(
    bbox: Iterable[float],
    input_size: tuple[int, int],
    padding: float,
) -> tuple[np.ndarray, np.ndarray]:
    x0, y0, x1, y1 = [float(value) for value in bbox]
    center = np.asarray(((x0 + x1) * 0.5, (y0 + y1) * 0.5), dtype=np.float32)
    scale = np.asarray((max(1.0, x1 - x0), max(1.0, y1 - y0)), dtype=np.float32)
    scale *= float(padding)
    input_width, input_height = input_size
    aspect = float(input_width) / float(input_height)
    if scale[0] > scale[1] * aspect:
        scale[1] = scale[0] / aspect
    else:
        scale[0] = scale[1] * aspect
    return center, scale


def _third_point(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    direction = first - second
    return second + np.asarray((-direction[1], direction[0]), dtype=np.float32)


def _pose_warp_matrix(
    center: np.ndarray,
    scale: np.ndarray,
    input_size: tuple[int, int],
) -> np.ndarray:
    input_width, input_height = input_size
    source = np.zeros((3, 2), dtype=np.float32)
    source[0] = center
    source[1] = center + np.asarray((0.0, -0.5 * scale[0]), dtype=np.float32)
    source[2] = _third_point(source[0], source[1])
    destination = np.zeros((3, 2), dtype=np.float32)
    destination[0] = (0.5 * input_width, 0.5 * input_height)
    destination[1] = destination[0] + np.asarray((0.0, -0.5 * input_width), dtype=np.float32)
    destination[2] = _third_point(destination[0], destination[1])
    return cv2.getAffineTransform(source, destination)


def _prepare_pose_input(
    image_bgr: np.ndarray,
    bbox: Iterable[float],
    input_size: tuple[int, int],
    padding: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    center, scale = _bbox_center_scale(bbox, input_size, padding)
    matrix = _pose_warp_matrix(center, scale, input_size)
    width, height = input_size
    crop = cv2.warpAffine(image_bgr, matrix, (width, height), flags=cv2.INTER_LINEAR)
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32)
    crop = (crop - _POSE_MEAN_RGB) / _POSE_STD_RGB
    tensor = np.ascontiguousarray(crop.transpose(2, 0, 1)[None], dtype=np.float32)
    return tensor, center, scale


def _letterbox_detector_input(
    image_bgr: np.ndarray,
    input_size: tuple[int, int],
) -> tuple[np.ndarray, float]:
    width, height = input_size
    image_height, image_width = image_bgr.shape[:2]
    ratio = min(float(width) / image_width, float(height) / image_height)
    resized_width = max(1, min(width, int(round(image_width * ratio))))
    resized_height = max(1, min(height, int(round(image_height * ratio))))
    resized = cv2.resize(image_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((height, width, 3), 114.0, dtype=np.float32)
    canvas[:resized_height, :resized_width] = resized.astype(np.float32)
    canvas = (canvas - _DETECTOR_MEAN_BGR) / _DETECTOR_STD_BGR
    tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32)
    return tensor, ratio


def _squeeze_batch(array: np.ndarray) -> np.ndarray:
    array = np.asarray(array)
    return array[0] if array.ndim >= 1 and array.shape[0] == 1 else array


@dataclass
class _DeviceBuffer:
    pointer: Any
    size: int


class _TensorRTEngine:
    """Small TensorRT 10 runner using the name-based I/O API."""

    def __init__(self, engine_path: str, device_id: int = 0, logger: Any = None):
        try:
            import tensorrt as trt
            from cuda import cudart
        except Exception as exc:
            raise RuntimeError(
                "The native TensorRT runtime requires JetPack's tensorrt module and "
                "cuda-python (from cuda import cudart)."
            ) from exc

        self.trt = trt
        self.cudart = cudart
        self.logger = logger
        self.path = _resolve_engine_path(engine_path, "TensorRT")
        self._buffers: dict[str, _DeviceBuffer] = {}
        self._closed = False
        self._cuda_check(cudart.cudaSetDevice(int(device_id)), "cudaSetDevice")
        self.stream = self._cuda_check(cudart.cudaStreamCreate(), "cudaStreamCreate")

        trt_logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(trt_logger)
        with self.path.open("rb") as handle:
            self.engine = self.runtime.deserialize_cuda_engine(handle.read())
        if self.engine is None:
            self.close()
            raise RuntimeError(f"TensorRT could not deserialize engine: {self.path}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            self.close()
            raise RuntimeError(f"TensorRT could not create execution context: {self.path}")

        self.input_names: list[str] = []
        self.output_names: list[str] = []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)
        if len(self.input_names) != 1 or not self.output_names:
            self.close()
            raise RuntimeError(
                f"Expected one input and at least one output in {self.path}; "
                f"got inputs={self.input_names}, outputs={self.output_names}"
            )
        _log(logger, "info", self.describe())

    def _cuda_check(self, result: Any, operation: str) -> Any:
        values = result if isinstance(result, tuple) else (result,)
        error = values[0]
        if error != self.cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f"{operation} failed with CUDA error {error}")
        if len(values) == 1:
            return None
        return values[1] if len(values) == 2 else values[1:]

    def input_shape_nchw(self) -> tuple[int, int, int, int]:
        shape = tuple(int(value) for value in self.engine.get_tensor_shape(self.input_names[0]))
        if len(shape) != 4 or any(value <= 0 for value in shape):
            raise ValueError(
                f"Engine must use a fixed positive NCHW input shape for this backend: "
                f"{self.path} has {shape}"
            )
        return shape

    def _ensure_buffer(self, name: str, size: int) -> _DeviceBuffer:
        current = self._buffers.get(name)
        if current is not None and current.size >= size:
            return current
        if current is not None:
            self._cuda_check(self.cudart.cudaFree(current.pointer), f"cudaFree({name})")
        pointer = self._cuda_check(self.cudart.cudaMalloc(int(size)), f"cudaMalloc({name})")
        buffer = _DeviceBuffer(pointer=pointer, size=int(size))
        self._buffers[name] = buffer
        return buffer

    def infer(self, input_tensor: np.ndarray) -> dict[str, np.ndarray]:
        input_name = self.input_names[0]
        expected_dtype = np.dtype(self.trt.nptype(self.engine.get_tensor_dtype(input_name)))
        host_input = np.ascontiguousarray(input_tensor, dtype=expected_dtype)
        if not self.context.set_input_shape(input_name, tuple(host_input.shape)):
            raise RuntimeError(
                f"TensorRT rejected input shape {host_input.shape} for {self.path.name}"
            )

        input_buffer = self._ensure_buffer(input_name, host_input.nbytes)
        if not self.context.set_tensor_address(input_name, int(input_buffer.pointer)):
            raise RuntimeError(f"set_tensor_address failed for input {input_name}")
        self._cuda_check(
            self.cudart.cudaMemcpyAsync(
                input_buffer.pointer,
                host_input.ctypes.data,
                host_input.nbytes,
                self.cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                self.stream,
            ),
            f"cudaMemcpyAsync(H2D:{input_name})",
        )

        outputs: dict[str, np.ndarray] = {}
        for name in self.output_names:
            shape = tuple(int(value) for value in self.context.get_tensor_shape(name))
            if not shape or any(value < 0 for value in shape):
                raise RuntimeError(f"Unresolved output shape for {name}: {shape}")
            dtype = np.dtype(self.trt.nptype(self.engine.get_tensor_dtype(name)))
            host_output = np.empty(shape, dtype=dtype)
            output_buffer = self._ensure_buffer(name, host_output.nbytes)
            if not self.context.set_tensor_address(name, int(output_buffer.pointer)):
                raise RuntimeError(f"set_tensor_address failed for output {name}")
            outputs[name] = host_output

        if not self.context.execute_async_v3(self.stream):
            raise RuntimeError(f"execute_async_v3 failed for {self.path}")
        for name, host_output in outputs.items():
            output_buffer = self._buffers[name]
            self._cuda_check(
                self.cudart.cudaMemcpyAsync(
                    host_output.ctypes.data,
                    output_buffer.pointer,
                    host_output.nbytes,
                    self.cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                    self.stream,
                ),
                f"cudaMemcpyAsync(D2H:{name})",
            )
        self._cuda_check(self.cudart.cudaStreamSynchronize(self.stream), "cudaStreamSynchronize")
        return outputs

    def describe(self) -> str:
        input_name = self.input_names[0]
        input_shape = tuple(int(value) for value in self.engine.get_tensor_shape(input_name))
        return (
            f"Loaded TensorRT engine {self.path.name}: input={input_name}{input_shape}, "
            f"outputs={self.output_names}"
        )

    def close(self) -> None:
        if getattr(self, "_closed", True):
            return
        self._closed = True
        for buffer in getattr(self, "_buffers", {}).values():
            try:
                self._cuda_check(self.cudart.cudaFree(buffer.pointer), "cudaFree")
            except Exception:
                pass
        self._buffers = {}
        stream = getattr(self, "stream", None)
        if stream is not None:
            try:
                self._cuda_check(self.cudart.cudaStreamDestroy(stream), "cudaStreamDestroy")
            except Exception:
                pass
        self.context = None
        self.engine = None
        self.runtime = None

    def __del__(self) -> None:
        self.close()


class TensorRTRuntime:
    """RTMDet + RTMPose inference implemented directly on TensorRT 10."""

    def __init__(
        self,
        detector_engine: str,
        body_engine: str,
        hand_engine: str,
        device_id: int = 0,
        detector_score_threshold: float = 0.05,
        detector_nms_iou_threshold: float = 0.6,
        detector_max_detections: int = 100,
        pose_bbox_padding: float = 1.25,
        logger: Any = None,
    ):
        self.detector = _TensorRTEngine(detector_engine, device_id, logger)
        self.body_pose = _TensorRTEngine(body_engine, device_id, logger)
        self.hand_pose = _TensorRTEngine(hand_engine, device_id, logger)
        self.detector_score_threshold = float(detector_score_threshold)
        self.detector_nms_iou_threshold = float(detector_nms_iou_threshold)
        self.detector_max_detections = max(1, int(detector_max_detections))
        self.pose_bbox_padding = max(1.0, float(pose_bbox_padding))

    def detect(self, image_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        _, _, input_height, input_width = self.detector.input_shape_nchw()
        tensor, resize_ratio = _letterbox_detector_input(
            image_bgr, (input_width, input_height)
        )
        outputs = self.detector.infer(tensor)
        boxes, scores, labels = self._decode_detector_outputs(outputs)
        valid = np.isfinite(scores) & (scores >= self.detector_score_threshold)
        boxes, scores, labels = boxes[valid], scores[valid], labels[valid]
        if boxes.size == 0:
            return np.empty((0, 5), np.float32), np.empty((0,), np.int64)

        keep_parts = []
        for label in np.unique(labels):
            indices = np.flatnonzero(labels == label)
            kept = _nms(
                boxes[indices],
                scores[indices],
                self.detector_nms_iou_threshold,
                self.detector_max_detections,
            )
            keep_parts.extend(indices[kept].tolist())
        keep = np.asarray(keep_parts, dtype=np.int64)
        keep = keep[np.argsort(scores[keep])[::-1]][: self.detector_max_detections]
        boxes = boxes[keep] / max(resize_ratio, 1e-8)
        image_height, image_width = image_bgr.shape[:2]
        boxes[:, 0::2] = np.clip(boxes[:, 0::2], 0.0, float(image_width))
        boxes[:, 1::2] = np.clip(boxes[:, 1::2], 0.0, float(image_height))
        detections = np.concatenate((boxes, scores[keep, None]), axis=1).astype(np.float32)
        return detections, labels[keep].astype(np.int64)

    @staticmethod
    def _decode_detector_outputs(
        outputs: dict[str, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        named = {name.lower(): _squeeze_batch(value) for name, value in outputs.items()}
        box_item = next(
            ((name, value) for name, value in named.items() if "box" in name and value.ndim == 2),
            None,
        )
        score_item = next(
            ((name, value) for name, value in named.items() if "score" in name and value.ndim == 2),
            None,
        )
        label_item = next(
            ((name, value) for name, value in named.items() if "label" in name),
            None,
        )
        if box_item is not None and score_item is not None:
            boxes = np.asarray(box_item[1], dtype=np.float32).reshape(-1, 4)
            class_scores = np.asarray(score_item[1], dtype=np.float32)
            if class_scores.shape[0] != boxes.shape[0]:
                raise ValueError(
                    f"RTMDet boxes/scores disagree: {boxes.shape} vs {class_scores.shape}"
                )
            labels = np.argmax(class_scores, axis=1).astype(np.int64)
            scores = class_scores[np.arange(class_scores.shape[0]), labels]
            return boxes, scores, labels

        arrays = [(name, _squeeze_batch(value)) for name, value in outputs.items()]
        detection_item = next(
            ((name, value) for name, value in arrays if value.ndim == 2 and value.shape[1] in (5, 6)),
            None,
        )
        if detection_item is None:
            shapes = {name: tuple(value.shape) for name, value in outputs.items()}
            raise ValueError(
                "Unsupported RTMDet outputs. Expected boxes+scores or an Nx5/Nx6 tensor; "
                f"got {shapes}"
            )
        detections = np.asarray(detection_item[1], dtype=np.float32)
        boxes = detections[:, :4]
        scores = detections[:, 4]
        if detections.shape[1] == 6:
            labels = detections[:, 5].astype(np.int64)
        elif label_item is not None:
            labels = np.asarray(label_item[1], dtype=np.int64).reshape(-1)[: len(boxes)]
        else:
            labels = np.zeros((len(boxes),), dtype=np.int64)
        return boxes, scores, labels

    def pose(self, kind: str, image_bgr: np.ndarray, bboxes: np.ndarray) -> np.ndarray:
        runner = self.body_pose if kind == "body" else self.hand_pose
        _, _, input_height, input_width = runner.input_shape_nchw()
        results = []
        for bbox in np.asarray(bboxes, dtype=np.float32).reshape(-1, 4):
            tensor, center, scale = _prepare_pose_input(
                image_bgr,
                bbox,
                (input_width, input_height),
                self.pose_bbox_padding,
            )
            outputs = runner.infer(tensor)
            simcc_x, simcc_y = self._find_simcc_outputs(
                outputs, input_width, input_height
            )
            coordinates, scores = _decode_simcc(simcc_x, simcc_y)
            coordinates, scores = coordinates[0], scores[0]
            split_x = float(simcc_x.shape[-1]) / float(input_width)
            split_y = float(simcc_y.shape[-1]) / float(input_height)
            coordinates[:, 0] /= split_x
            coordinates[:, 1] /= split_y
            coordinates = (
                coordinates / np.asarray((input_width, input_height), np.float32) * scale
                + center
                - scale * 0.5
            )
            results.append(np.concatenate((coordinates, scores[:, None]), axis=1))
        if not results:
            return np.empty((0, 0, 3), dtype=np.float32)
        return np.asarray(results, dtype=np.float32)

    @staticmethod
    def _find_simcc_outputs(
        outputs: dict[str, np.ndarray], input_width: int, input_height: int
    ) -> tuple[np.ndarray, np.ndarray]:
        lowered = {name.lower(): np.asarray(value) for name, value in outputs.items()}
        x_value = next((value for name, value in lowered.items() if "simcc_x" in name), None)
        y_value = next((value for name, value in lowered.items() if "simcc_y" in name), None)
        if x_value is None or y_value is None:
            candidates = [value for value in lowered.values() if value.ndim in (2, 3)]
            if len(candidates) != 2:
                shapes = {name: tuple(value.shape) for name, value in outputs.items()}
                raise ValueError(f"Could not identify SimCC x/y outputs: {shapes}")
            first, second = candidates
            direct_error = abs(first.shape[-1] / float(input_width) - second.shape[-1] / float(input_height))
            swapped_error = abs(second.shape[-1] / float(input_width) - first.shape[-1] / float(input_height))
            if direct_error <= swapped_error:
                x_value, y_value = first, second
            else:
                x_value, y_value = second, first
        return np.asarray(x_value, np.float32), np.asarray(y_value, np.float32)

    def close(self) -> None:
        self.detector.close()
        self.body_pose.close()
        self.hand_pose.close()
