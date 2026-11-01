# Model artifacts

Run `deploy/export_tensorrt_models.sh` on the target Jetson. The generated
MMDeploy SDK directories will be placed under `models/mmdeploy/` and are
ignored by Git because TensorRT engines are platform-specific binary files.
