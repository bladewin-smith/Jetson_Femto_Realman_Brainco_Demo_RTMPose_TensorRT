#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
if [[ $# -ge 1 ]]; then
  ONNX_DIR="$(realpath "$1")"
else
  ONNX_DIR="$REPO_ROOT/models/onnx"
fi
if [[ $# -ge 2 ]]; then
  ENGINE_DIR="$(realpath -m "$2")"
else
  ENGINE_DIR="$REPO_ROOT/models/tensorrt"
fi

if command -v trtexec >/dev/null 2>&1; then
  TRTEXEC="$(command -v trtexec)"
elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
  TRTEXEC=/usr/src/tensorrt/bin/trtexec
else
  echo "trtexec was not found. Install the JetPack libnvinfer-bin package." >&2
  exit 2
fi

mkdir -p "$ENGINE_DIR"

build_engine() {
  local model_name="$1"
  local onnx_path="$ONNX_DIR/$model_name.onnx"
  local engine_path="$ENGINE_DIR/$model_name.engine"
  [[ -f "$onnx_path" ]] || {
    echo "Missing ONNX model: $onnx_path" >&2
    exit 3
  }
  echo "Building $engine_path"
  "$TRTEXEC" \
    --onnx="$onnx_path" \
    --saveEngine="$engine_path" \
    --fp16 \
    --memPoolSize=workspace:1024 \
    --builderOptimizationLevel=3 \
    --skipInference
  [[ -s "$engine_path" ]] || {
    echo "TensorRT did not create $engine_path" >&2
    exit 4
  }
}

build_engine rtmdet_nano
build_engine rtmpose_body
build_engine rtmpose_hand

echo "TensorRT engines are ready in $ENGINE_DIR"
