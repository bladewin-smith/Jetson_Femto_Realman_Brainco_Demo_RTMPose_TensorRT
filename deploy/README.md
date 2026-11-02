# Native TensorRT 10 deployment

The production controller consumes three bare TensorRT engines and does not
import mmdeploy_runtime:

    models/tensorrt/
    ├── rtmdet_nano.engine
    ├── rtmpose_body.engine
    └── rtmpose_hand.engine

The engine inputs are fixed batch-1 NCHW tensors:

| model | input | outputs |
|---|---|---|
| RTMDet-nano | 1x3x320x320 | decoded boxes, sigmoid scores (no NMS) |
| RTMPose-S body | 1x3x256x192 | simcc_x, simcc_y |
| RTMPose-M Hand5 | 1x3x256x256 | simcc_x, simcc_y |

Image normalization, NMS, SimCC decoding and ROI mapping are implemented in
vision_backends/tensorrt_runtime.py.

## 1. Export ONNX

Use a separate PyTorch/OpenMMLab environment, preferably on an x86 workstation.
It needs compatible MMEngine, MMCV, MMDetection and a MMPose source checkout,
plus ONNX. It does not need MMDeploy.

    python3 deploy/export_onnx_models.py \
      --mmpose-root /path/to/mmpose \
      --output-dir models/onnx \
      --device cuda:0

The script loads the three official checkpoints, exports opset 17 and runs the
ONNX checker. Copy models/onnx/*.onnx to the Jetson if export was performed on
another machine.

## 2. Build FP16 engines on the target Jetson

Always build engines on the board and software image that will run them:

    cd /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
    bash deploy/build_tensorrt_engines.sh models/onnx models/tensorrt

The script uses JetPack's trtexec, FP16, a 1 GiB builder workspace and fixed
input shapes. Rebuild all engines after changing JetPack, TensorRT, the GPU
board, model inputs or ONNX files.

## 3. Runtime dependencies

Keep TensorRT and OpenCV from JetPack/Ubuntu. Add only CUDA Python and a NumPy
1.x version in a system-site-packages virtual environment:

    python3 -m venv --system-site-packages .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip wheel
    python -m pip install -r deploy/requirements-jetson-runtime.txt

Verify that all modules resolve in the same interpreter:

    python -c "import cv2, numpy, tensorrt; from cuda import cudart; print(cv2.__version__, numpy.__version__, tensorrt.__version__)"

The legacy export_tensorrt_models.sh and models/mmdeploy/ path are retained
only for compatibility with the optional rtmpose_runtime:=mmdeploy backend.
