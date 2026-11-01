# RTMDet + RTMPose + 专用手部模型机械臂控制

这是从原项目独立复制并优化的新仓库。原仓库和
`Jetson_Femto_Realman_Brainco_Demo_MediaPipe_Pose` 均不会被本仓库修改。

本版本把视觉主链替换为面向 Jetson Orin Nano 8GB 的长期部署方案：

```text
Orbbec RGB-D
    │
    ├─ RTMDet-nano 320×320（全图、限频）
    │      └─ 人体候选框 + 躯干深度排序 + 目标锁定
    │
    └─ 最近操作者
           ├─ RTMPose-S 256×192：COCO 17 点身体骨架
           ├─ 左肘/腕 → 左手 ROI → RTMPose-M Hand5 21 点
           └─ 右肘/腕 → 右手 ROI → RTMPose-M Hand5 21 点
                    │
                    ├─ 肩、肘角度
                    ├─ 双手五指弯曲度
                    ├─ 手掌 RGB-D 三维位移
                    └─ RM65-B 六向运动 + BrainCo/RealMan 灵巧手
```

## 已落地功能

- RTMDet 人体检测与 Orbbec 躯干深度融合，按最近距离选操作者。
- 目标框 IoU/中心距离连续跟踪，避免多人交叉时只按瞬时最近深度跳人。
- 只对锁定操作者运行 RTMPose 身体模型，避免逐人重复姿态推理。
- 根据左右肘腕方向生成手部方形 ROI，不再额外运行全图手部检测器。
- 双手使用独立 Hand5 21 点 RTMPose 模型，可通过超参数控制 ROI 大小、腕外延和手部推理间隔。
- 保留既有的 RGB-D 三维反投影、手指弯曲度、肩肘角度、ROS 话题、机械臂/灵巧手通信和安全门控。
- 生产后端为 MMDeploy TensorRT FP16；同时保留 MMPose PyTorch 参考后端和 MediaPipe 回退后端。
- 提供三模型一键导出脚本、Jetson 自启动配置和无硬件单元测试。

## 目录

| 路径 | 作用 |
|---|---|
| `scripts/camera_arm_rm65b_control_rtmpose.py` | 推荐启动入口 |
| `scripts/camera_arm_rm65b_control_pose_hands.py` | 完整控制实现，已接入可切换视觉后端 |
| `vision_backends/rtmpose_backend.py` | RTMDet、RTMPose、手部 ROI、深度选人和运行时适配 |
| `deploy/export_tensorrt_models.sh` | 导出三套 MMDeploy TensorRT FP16 SDK 模型 |
| `deploy/mmdeploy_configs/` | 固定 batch=1 的 320×320、256×192、256×256 部署配置 |
| `tests/test_rtmpose_backend.py` | 检测阈值、关键点映射、手 ROI 与降频缓存测试 |
| `jetson_autostart/` | systemd/桌面自启动脚本与环境配置 |
| `scripts/camera_arm_rm65b_control_neural.py` | 从原仓库复制的对照基线 |

## 运行依赖

除原项目已有的 ROS 2、PyOrbbecSDK、OpenCV/NumPy、RealMan RM_API2 和可选
BrainCo SDK 外，RTMPose 生产模式还需要针对目标 JetPack 构建的
`mmdeploy_runtime` Python 模块。PyTorch 参考模式需要版本相互兼容的 MMEngine、
MMCV、MMDetection、MMPose 和 PyTorch。由于 Jetson 的 CUDA/TensorRT 版本与普通
x86 wheel 不同，本仓库没有提供一条会盲目覆盖系统 CUDA 依赖的 `pip install`
命令。

## 模型准备

控制程序读取的是 MMDeploy SDK 模型目录，每个目录至少应有：

```text
deploy.json
pipeline.json
end2end.engine
```

不能只把 `end2end.engine` 路径传给程序。TensorRT engine 与 JetPack、TensorRT
版本和 GPU 环境绑定，建议直接在目标 Orin Nano 上生成。

```bash
cd /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
bash deploy/export_tensorrt_models.sh \
  /home/jetson/ws/mmdeploy \
  /home/jetson/ws/mmpose
```

详细依赖和产物结构见 `deploy/README.md`。

## 先以识别模式启动

在机械臂上电前先关闭动作输出，检查三套视觉模型、操作者选择、左右手标签和深度：

```bash
cd /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
python3 scripts/camera_arm_rm65b_control_rtmpose.py \
  --ros-args \
  -p vision_backend:=rtmpose \
  -p rtmpose_runtime:=mmdeploy \
  -p rtmdet_model_dir:=$PWD/models/mmdeploy/rtmdet_nano \
  -p rtmpose_body_model_dir:=$PWD/models/mmdeploy/rtmpose_body \
  -p rtmpose_hand_model_dir:=$PWD/models/mmdeploy/rtmpose_hand \
  -p robot_motion_enabled:=false \
  -p hand_control_enabled:=false \
  -p show_window:=true
```

确认安全后，再显式打开机械臂与灵巧手控制：

```bash
python3 scripts/camera_arm_rm65b_control_rtmpose.py \
  --ros-args \
  -p vision_backend:=rtmpose \
  -p rtmpose_runtime:=mmdeploy \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p hand_control_enabled:=true \
  -p hand_transport:=brainco-rs485 \
  -p control_hand_side:=right
```

默认不会自动开始动作。在 GUI 中确认最近操作者稳定后按 `s`；按 `h` 回安全位，
按 `q` 或 `Esc` 退出。无界面自启动只有在完成现场验证后，才应设置
`AUTO_START_ON_READY=true`。

## 关键超参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `vision_backend` | `rtmpose` | `rtmpose` 或 `mediapipe` 回退 |
| `rtmpose_runtime` | `mmdeploy` | Jetson 生产使用 `mmdeploy`；桌面校验可用 `pytorch` |
| `rtmdet_inference_hz` | `8.0` | 全图人体检测频率；身体姿态仍按相机循环更新 |
| `rtmpose_person_score_threshold` | `0.45` | RTMDet 人体框阈值 |
| `rtmpose_body_keypoint_threshold` | `0.35` | 身体关键点阈值 |
| `rtmpose_hand_keypoint_threshold` | `0.25` | 手部关键点阈值 |
| `rtmpose_hand_roi_scale` | `2.25` | 手 ROI 边长与前臂长度的比例 |
| `rtmpose_hand_roi_wrist_extension` | `0.35` | ROI 中心沿肘到腕方向向外延伸比例 |
| `rtmpose_hand_roi_min_size_px` | `96` | 手 ROI 最小边长 |
| `rtmpose_hand_roi_max_size_px` | `420` | 手 ROI 最大边长 |
| `rtmpose_hand_inference_interval` | `2` | 每两帧更新一次双手，空结果会立即重试 |
| `person_min_depth_m` / `person_max_depth_m` | `0.35` / `4.0` | 可选操作者深度范围 |
| `person_track_min_iou` | `0.08` | 目标锁定最小 IoU |
| `control_hand_side` | `right` | `left`、`right` 或 `auto` |
| `robot_motion_enabled` | `false` | 必须显式开启机械臂动作 |
| `hand_control_enabled` | `false` | 必须显式开启灵巧手动作 |

`pytorch` 参考模式还需要传入 `rtmdet_config/checkpoint`、
`rtmpose_body_config/checkpoint` 和 `rtmpose_hand_config/checkpoint` 六个参数。

## ROS 输出

- `operator_torso_depth`：锁定操作者的躯干深度。
- `operator_skeleton_3d`：COCO 17 点，每点 `[x,y,z,score]`。
- `operator_arm_joint_angles`：左右肩、肘角度。
- `operator_dual_hand_bends`：左、右手各五指弯曲度。
- `operator_dual_palm_xyz`：左右掌心三维位置。
- `hand_finger_bends`：当前控制手五指弯曲度。
- `hand_palm_direction_distances`：当前控制手六方向位移。

## 测试

不连接相机和机械臂即可执行视觉几何测试：

```bash
python3 -m unittest discover -s tests -v
```

在已安装 NumPy/OpenCV 的开发或 Jetson 环境中，可验证人体阈值、COCO/Hand5
映射、手 ROI 边界和推理降频逻辑；真实 FPS、TensorRT 兼容性、Orbbec 深度质量
和机械臂动作仍必须在目标 Jetson 与实际硬件上分阶段验收。

## 官方参考

- [MMPose RTMPose 模型](https://github.com/open-mmlab/mmpose/tree/main/configs/body_2d_keypoint/rtmpose)
- [MMPose 手部关键点示例](https://github.com/open-mmlab/mmpose/blob/main/demo/docs/en/2d_hand_demo.md)
- [MMPose MMDeploy 部署说明](https://github.com/open-mmlab/mmpose/blob/main/docs/en/user_guides/how_to_deploy.md)
- [MMDeploy Detector/PoseDetector Python 示例](https://github.com/open-mmlab/mmdeploy/blob/main/demo/python/det_pose.py)
