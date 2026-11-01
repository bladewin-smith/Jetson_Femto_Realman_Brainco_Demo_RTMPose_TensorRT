# Jetson TensorRT model export

The controller consumes **MMDeploy SDK model directories**, not bare TensorRT
engine files. Each of these directories must contain `deploy.json`,
`pipeline.json`, and `end2end.engine`:

```text
models/mmdeploy/
├── rtmdet_nano/
├── rtmpose_body/
└── rtmpose_hand/
```

The export script uses FP16 and fixed batch size 1, which matches this
application: one selected operator and one hand ROI per pose call.

## Prerequisites

Build MMPose/MMDetection/MMDeploy against the CUDA and TensorRT versions from
the JetPack image. Do not install an x86 prebuilt MMDeploy runtime on Jetson.
Confirm that these checkouts are mutually compatible before exporting.

## Export

```bash
cd /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
bash deploy/export_tensorrt_models.sh \
  /home/jetson/ws/mmdeploy \
  /home/jetson/ws/mmpose
```

The three selected official checkpoints are:

- RTMDet-nano 320×320, person class only.
- RTMPose-S 256×192, COCO 17-point body skeleton.
- RTMPose-M 256×256, Hand5 21-point dedicated hand model.

TensorRT engines are tied to the target TensorRT/GPU environment. Export or
rebuild them on the Jetson that will run the controller whenever JetPack or
TensorRT changes.
