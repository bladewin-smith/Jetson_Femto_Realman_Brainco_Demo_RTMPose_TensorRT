#!/usr/bin/env python3
"""Export RTMDet and RTMPose models to plain ONNX without MMDeploy.

Run this in a separate OpenMMLab/PyTorch environment. The resulting ONNX
files contain only standard operators and are intended to be converted to
TensorRT engines on the target Jetson.
"""

from __future__ import annotations

import argparse
import inspect
from pathlib import Path

import torch


PERSON_CHECKPOINT = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmpose/"
    "rtmdet_nano_8xb32-100e_coco-obj365-person-05d8511e.pth"
)
BODY_CHECKPOINT = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/"
    "rtmpose-s_simcc-coco_pt-aic-coco_420e-256x192-8edcf0d7_20230127.pth"
)
HAND_CHECKPOINT = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/"
    "rtmpose-m_simcc-hand5_pt-aic-coco_210e-256x256-74fb594_20230320.pth"
)


class RTMDetONNX(torch.nn.Module):
    """Return decoded boxes and sigmoid class scores, without NMS."""

    def __init__(self, model: torch.nn.Module):
        super().__init__()
        self.model = model

    def forward(self, image: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.model.extract_feat(image)
        class_maps, box_maps = self.model.bbox_head(features)
        feature_sizes = [feature.shape[-2:] for feature in class_maps]
        priors = self.model.bbox_head.prior_generator.grid_priors(
            feature_sizes,
            dtype=box_maps[0].dtype,
            device=box_maps[0].device,
            with_stride=True,
        )
        class_scores = []
        decoded_boxes = []
        for class_map, box_map, level_priors in zip(class_maps, box_maps, priors):
            scores = class_map.permute(0, 2, 3, 1).reshape(
                class_map.shape[0], -1, class_map.shape[1]
            )
            distances = box_map.permute(0, 2, 3, 1).reshape(
                box_map.shape[0], -1, 4
            )
            centers = level_priors[:, :2].unsqueeze(0)
            top_left = centers - distances[..., :2]
            bottom_right = centers + distances[..., 2:]
            decoded_boxes.append(torch.cat((top_left, bottom_right), dim=-1))
            class_scores.append(scores.sigmoid())
        return torch.cat(decoded_boxes, dim=1), torch.cat(class_scores, dim=1)


class RTMPoseONNX(torch.nn.Module):
    """Return the two raw SimCC distributions."""

    def __init__(self, model: torch.nn.Module):
        super().__init__()
        self.model = model

    def forward(self, image: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.model.extract_feat(image)
        simcc_x, simcc_y = self.model.head(features)
        return simcc_x, simcc_y


def _export(
    model: torch.nn.Module,
    sample: torch.Tensor,
    output_path: Path,
    output_names: tuple[str, str],
    opset: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    kwargs = dict(
        input_names=["input"],
        output_names=list(output_names),
        opset_version=opset,
        do_constant_folding=True,
        export_params=True,
    )
    if "dynamo" in inspect.signature(torch.onnx.export).parameters:
        kwargs["dynamo"] = False
    with torch.inference_mode():
        torch.onnx.export(model, sample, str(output_path), **kwargs)

    import onnx

    graph = onnx.load(str(output_path))
    onnx.checker.check_model(graph)
    print(f"validated: {output_path}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mmpose-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("models/onnx"))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--person-config", type=Path)
    parser.add_argument("--person-checkpoint", default=PERSON_CHECKPOINT)
    parser.add_argument("--body-config", type=Path)
    parser.add_argument("--body-checkpoint", default=BODY_CHECKPOINT)
    parser.add_argument("--hand-config", type=Path)
    parser.add_argument("--hand-checkpoint", default=HAND_CHECKPOINT)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    root = args.mmpose_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    person_config = args.person_config or (
        root / "projects/rtmpose/rtmdet/person/rtmdet_nano_320-8xb32_coco-person.py"
    )
    body_config = args.body_config or (
        root
        / "configs/body_2d_keypoint/rtmpose/coco/"
        "rtmpose-s_8xb256-420e_coco-256x192.py"
    )
    hand_config = args.hand_config or (
        root
        / "configs/hand_2d_keypoint/rtmpose/hand5/"
        "rtmpose-m_8xb256-210e_hand5-256x256.py"
    )
    for config in (person_config, body_config, hand_config):
        if not Path(config).is_file():
            raise FileNotFoundError(f"OpenMMLab config not found: {config}")

    from mmdet.apis import init_detector
    from mmpose.apis import init_model

    device = torch.device(args.device)
    detector = init_detector(
        str(person_config), args.person_checkpoint, device=str(device)
    ).eval()
    body = init_model(
        str(body_config), args.body_checkpoint, device=str(device)
    ).eval()
    hand = init_model(
        str(hand_config), args.hand_checkpoint, device=str(device)
    ).eval()

    _export(
        RTMDetONNX(detector).eval(),
        torch.randn(1, 3, 320, 320, device=device),
        output_dir / "rtmdet_nano.onnx",
        ("boxes", "scores"),
        args.opset,
    )
    _export(
        RTMPoseONNX(body).eval(),
        torch.randn(1, 3, 256, 192, device=device),
        output_dir / "rtmpose_body.onnx",
        ("simcc_x", "simcc_y"),
        args.opset,
    )
    _export(
        RTMPoseONNX(hand).eval(),
        torch.randn(1, 3, 256, 256, device=device),
        output_dir / "rtmpose_hand.onnx",
        ("simcc_x", "simcc_y"),
        args.opset,
    )
    print(f"ONNX models are ready in {output_dir}")


if __name__ == "__main__":
    main()
