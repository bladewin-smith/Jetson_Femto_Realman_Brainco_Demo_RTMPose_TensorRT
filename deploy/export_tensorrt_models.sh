#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 /path/to/mmdeploy /path/to/mmpose [output_dir]" >&2
  exit 2
fi

MMDEPLOY_DIR="$(realpath "$1")"
MMPOSE_DIR="$(realpath "$2")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "${SCRIPT_DIR}/..")"
OUTPUT_DIR="$(realpath -m "${3:-${REPO_ROOT}/models/mmdeploy}")"

[[ -f "${MMDEPLOY_DIR}/tools/deploy.py" ]] || { echo "Invalid MMDeploy checkout: ${MMDEPLOY_DIR}" >&2; exit 3; }
[[ -d "${MMPOSE_DIR}/mmpose" ]] || { echo "Invalid MMPose checkout: ${MMPOSE_DIR}" >&2; exit 3; }
mkdir -p "${OUTPUT_DIR}"

PERSON_CONFIG="${MMPOSE_DIR}/projects/rtmpose/rtmdet/person/rtmdet_nano_320-8xb32_coco-person.py"
BODY_CONFIG="${MMPOSE_DIR}/configs/body_2d_keypoint/rtmpose/coco/rtmpose-s_8xb256-420e_coco-256x192.py"
HAND_CONFIG="${MMPOSE_DIR}/configs/hand_2d_keypoint/rtmpose/hand5/rtmpose-m_8xb256-210e_hand5-256x256.py"
PERSON_CHECKPOINT="https://download.openmmlab.com/mmpose/v1/projects/rtmpose/rtmdet_nano_8xb32-100e_coco-obj365-person-05d8511e.pth"
BODY_CHECKPOINT="https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-s_simcc-coco_pt-aic-coco_420e-256x192-8edcf0d7_20230127.pth"
HAND_CHECKPOINT="https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-m_simcc-hand5_pt-aic-coco_210e-256x256-74fb594_20230320.pth"
HUMAN_IMAGE="${MMDEPLOY_DIR}/demo/resources/human-pose.jpg"
HAND_IMAGE="${MMPOSE_DIR}/tests/data/onehand10k/9.jpg"

python3 "${MMDEPLOY_DIR}/tools/deploy.py" \
  "${SCRIPT_DIR}/mmdeploy_configs/rtmdet_tensorrt-fp16_static-320x320.py" \
  "${PERSON_CONFIG}" "${PERSON_CHECKPOINT}" "${HUMAN_IMAGE}" \
  --work-dir "${OUTPUT_DIR}/rtmdet_nano" --device cuda:0 --dump-info

python3 "${MMDEPLOY_DIR}/tools/deploy.py" \
  "${SCRIPT_DIR}/mmdeploy_configs/rtmpose_simcc_tensorrt-fp16_static-256x192.py" \
  "${BODY_CONFIG}" "${BODY_CHECKPOINT}" "${HUMAN_IMAGE}" \
  --work-dir "${OUTPUT_DIR}/rtmpose_body" --device cuda:0 --dump-info

python3 "${MMDEPLOY_DIR}/tools/deploy.py" \
  "${SCRIPT_DIR}/mmdeploy_configs/rtmpose_simcc_tensorrt-fp16_static-256x256.py" \
  "${HAND_CONFIG}" "${HAND_CHECKPOINT}" "${HAND_IMAGE}" \
  --work-dir "${OUTPUT_DIR}/rtmpose_hand" --device cuda:0 --dump-info

for model in rtmdet_nano rtmpose_body rtmpose_hand; do
  for file in deploy.json pipeline.json end2end.engine; do
    [[ -f "${OUTPUT_DIR}/${model}/${file}" ]] || { echo "Missing ${model}/${file}" >&2; exit 4; }
  done
done

echo "TensorRT SDK models are ready in ${OUTPUT_DIR}"
