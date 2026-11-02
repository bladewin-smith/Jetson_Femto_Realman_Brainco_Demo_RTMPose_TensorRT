# Model artifacts

The native runtime expects:

    models/onnx/rtmdet_nano.onnx
    models/onnx/rtmpose_body.onnx
    models/onnx/rtmpose_hand.onnx
    models/tensorrt/rtmdet_nano.engine
    models/tensorrt/rtmpose_body.engine
    models/tensorrt/rtmpose_hand.engine

Create ONNX with deploy/export_onnx_models.py, then build the FP16 engines on
the target Jetson with deploy/build_tensorrt_engines.sh. ONNX, checkpoint and
engine binaries are ignored by Git. TensorRT engines are not portable across
arbitrary JetPack/TensorRT/GPU combinations.
