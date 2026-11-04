# RTMDet + RTMPose + 专用手部模型机械臂控制

这是从原项目独立复制并优化的仓库。当前生产视觉链已经切换为原生
TensorRT 10 Python 后端，不再要求 MMDeploy SDK：

    Orbbec RGB-D
        │
        ├─ RTMDet-nano 320×320（全图、限频）
        │      └─ 人体框 + 躯干深度排序 + 最近操作者锁定
        │
        └─ 锁定操作者
               ├─ RTMPose-S 256×192：COCO 17 点身体骨架
               ├─ 左肘/腕 → 左手 ROI → RTMPose-M Hand5 21 点
               └─ 右肘/腕 → 右手 ROI → RTMPose-M Hand5 21 点
                        │
                        ├─ 肩、肘角度
                        ├─ 双手五指弯曲度
                        ├─ 手掌 RGB-D 三维位移
                        └─ RM65-B 六向运动 + 灵巧手

## 当前实现

- 使用 TensorRT 10 名称式 I/O API：set_input_shape、set_tensor_address 和
  execute_async_v3。
- RTMDet engine 输出未做 NMS 的 boxes 和 scores；预处理、阈值过滤、分类 NMS
  与原图坐标恢复由 Python 实现。
- RTMPose 身体和手部 engine 输出 simcc_x 与 simcc_y；SimCC 解码、仿射 ROI
  和坐标回映射由 Python 实现。
- 人体检测与 Orbbec 躯干深度融合，按最近距离选操作者，并通过 IoU/中心距离
  连续跟踪目标。
- 只对锁定操作者运行身体模型；左右手 ROI 根据对应肘腕方向生成。
- 保留既有 RGB-D 反投影、手指弯曲度、肩肘角度、ROS 话题、机械臂/灵巧手
  通信和安全门控。
- MMDeploy 与 PyTorch 后端仍作为兼容和参考路径保留，但默认 runtime 为
  tensorrt。

## 主要文件

| 路径 | 作用 |
|---|---|
| scripts/camera_arm_rm65b_control_rtmpose.py | 推荐的完整控制入口 |
| scripts/test_tensorrt_vision.py | 不连接 ROS、Orbbec 和机械臂的视觉独立测试 |
| vision_backends/tensorrt_runtime.py | TensorRT 10 执行、预处理、NMS 与 SimCC 解码 |
| vision_backends/rtmpose_backend.py | 人体/身体/双手编排及深度选人 |
| deploy/export_onnx_models.py | 从 OpenMMLab 模型导出三个普通 ONNX |
| deploy/build_tensorrt_engines.sh | 在目标 Jetson 上生成三个 FP16 engine |
| deploy/requirements-jetson-runtime.txt | Jetson 额外 Python 运行时依赖 |
| tests/test_rtmpose_backend.py | 视觉几何与解码单元测试 |
| jetson_autostart/ | systemd/桌面自启动配置 |

## 模型产物

程序默认读取：

    models/tensorrt/rtmdet_nano.engine
    models/tensorrt/rtmpose_body.engine
    models/tensorrt/rtmpose_hand.engine

先在独立的 PyTorch/OpenMMLab 环境导出 ONNX。这个环境可以放在 x86 工作站，
不需要 MMDeploy：

    python3 deploy/export_onnx_models.py \
      --mmpose-root /path/to/mmpose \
      --output-dir models/onnx \
      --device cuda:0

把三个 ONNX 复制到 Jetson 后，必须在运行它们的 Jetson/JetPack/TensorRT 环境
生成 engine：

    cd /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
    bash deploy/build_tensorrt_engines.sh models/onnx models/tensorrt

详细说明见 deploy/README.md。TensorRT engine 不应跨不同 JetPack、TensorRT 或
GPU 环境直接复用。

## Jetson 运行时

TensorRT、CUDA 和 OpenCV 继续使用 JetPack/Ubuntu 自带版本。建议使用能够读取
系统包的虚拟环境，只额外安装 CUDA Python 与 NumPy 1.x：

    sudo apt update
    sudo apt install -y python3-venv python3-pip python3-opencv libnvinfer-bin
    python3 -m venv --system-site-packages .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip wheel
    python -m pip install -r deploy/requirements-jetson-runtime.txt

验证当前解释器：

    python -c "import cv2, numpy, tensorrt; from cuda import cudart; print(cv2.__version__, numpy.__version__, tensorrt.__version__)"

不要使用 pip 覆盖 JetPack 的 tensorrt、CUDA 或 cuDNN 包。

## 独立测试视觉部分

图片测试：

    source .venv/bin/activate
    python scripts/test_tensorrt_vision.py \
      --input /path/to/person.jpg \
      --output /tmp/rtmpose_result.jpg \
      --no-display

USB/CSI 相机或视频测试：

    python scripts/test_tensorrt_vision.py --input 0
    python scripts/test_tensorrt_vision.py --input /path/to/test.mp4

该入口只验证 RTMDet、身体姿态、左右手 ROI 和 Hand5 关键点，不初始化 ROS、
Orbbec SDK、机械臂或灵巧手。

## Orbbec 视觉链安全测试

需要验证深度选人时运行完整入口，但明确关闭所有动作输出：

    python scripts/camera_arm_rm65b_control_rtmpose.py \
      --ros-args \
      -p vision_backend:=rtmpose \
      -p rtmpose_runtime:=tensorrt \
      -p rtmdet_engine_path:=$PWD/models/tensorrt/rtmdet_nano.engine \
      -p rtmpose_body_engine_path:=$PWD/models/tensorrt/rtmpose_body.engine \
      -p rtmpose_hand_engine_path:=$PWD/models/tensorrt/rtmpose_hand.engine \
      -p robot_motion_enabled:=false \
      -p hand_control_enabled:=false \
      -p auto_start_on_ready:=false \
      -p show_window:=true

只有确认最近操作者、左右手标签、深度和坐标方向都稳定后，才显式开启机械臂和
灵巧手。默认不会自动发送动作；GUI 中按 s 开始，按 h 回安全位，按 q 或 Esc
退出。

## 关键参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| vision_backend | rtmpose | rtmpose 或 mediapipe 回退 |
| rtmpose_runtime | tensorrt | 原生 TensorRT；也保留 mmdeploy、pytorch |
| rtmdet_engine_path | models/tensorrt/rtmdet_nano.engine | 人体检测 engine |
| rtmpose_body_engine_path | models/tensorrt/rtmpose_body.engine | 身体 engine |
| rtmpose_hand_engine_path | models/tensorrt/rtmpose_hand.engine | 手部 engine |
| rtmdet_inference_hz | 8.0 | 全图人体检测频率 |
| rtmdet_nms_iou_threshold | 0.60 | RTMDet NMS IoU 阈值 |
| rtmdet_max_detections | 100 | NMS 后最大框数 |
| rtmpose_bbox_padding | 1.25 | 身体/手部姿态 ROI 外扩比例 |
| rtmpose_person_score_threshold | 0.45 | 人体框阈值 |
| rtmpose_body_keypoint_threshold | 0.35 | 身体关键点阈值 |
| rtmpose_hand_keypoint_threshold | 0.25 | 手部关键点阈值 |
| rtmpose_hand_roi_scale | 2.25 | 手 ROI 边长与前臂长度比例 |
| rtmpose_hand_roi_wrist_extension | 0.35 | ROI 沿肘到腕向外延伸比例 |
| rtmpose_hand_inference_interval | 2 | 双手推理间隔 |
| rtmpose_hand_keypoint_hold_frames | 2 | 单个低置信度手点最多复用的手部推理帧数；持续丢失后停止更新并触发安全逻辑 |
| person_min_depth_m / person_max_depth_m | 0.35 / 4.0 | 操作者深度范围 |
| person_track_min_iou | 0.08 | 目标锁定最小 IoU |
| control_hand_side | right | left、right 或 auto |
| robot_motion_enabled | false | 必须显式开启机械臂动作 |
| hand_control_enabled | false | 必须显式开启灵巧手动作 |
| hand_bend_calibration_enabled | true | 在发送硬件命令前启用逐指开合标定；ROS 话题仍保留原始弯曲度 |
| hand_bend_open_calibration | `[0.18,0.08,0.13,0.09,0.13]` | 完全张开标定，顺序为 thumb/index/middle/ring/pinky |
| hand_bend_closed_calibration | `[0.56,0.92,0.91,0.86,0.87]` | 完全握拳标定，顺序同上 |
| hand_bend_gamma | `[1,1,1,1,1]` | 逐指响应曲线；大于 1 会减弱中段，小于 1 会增强中段 |
| hand_thumb_rotate_mode | opposition | `opposition` 根据拇指尖相对掌宽独立计算旋转；`scaled-bend` 保留旧映射 |
| hand_thumb_rotate_scale | 0.35 | 拇指旋转轴最大比例，首次标定建议保持 0.35 |

## 测试

不连接 GPU、相机和机械臂即可执行纯 NumPy/OpenCV 几何测试：

    python3 -m unittest discover -s tests -v

这些测试覆盖阈值与关键点映射、手 ROI、推理降频、NMS、SimCC 解码和原始
RTMDet 输出解码。真实 engine 兼容性、FPS、Orbbec 深度质量和机械臂动作仍需在
目标 Jetson 上分阶段验收。

## ROS 输出

- operator_torso_depth：锁定操作者的躯干深度。
- operator_skeleton_3d：COCO 17 点，每点为 x、y、z、score。
- operator_arm_joint_angles：左右肩、肘角度。
- operator_dual_hand_bends：左右手各五指弯曲度。
- operator_dual_palm_xyz：左右掌心三维位置。
- hand_finger_bends：当前控制手五指弯曲度。
- hand_palm_direction_distances：当前控制手六方向位移。
