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
- 手部关键点采用紧凑跟踪 ROI、自适应关键点平滑和低置信度连续跟踪；四指再经
  自适应肤色掩膜与独立窄通道校验，降低相邻手指误联动。
- 拇指弯曲轴和旋转轴使用独立自动归零、双阈值迟滞、连续帧确认和静止区，
  抑制绷直状态下的非零偏差与往复抖动。
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
| vision_backends/finger_segmentation.py | 掌心自适应掩膜、逐指独立轮廓通道与伸直证据融合 |
| hand_control/calibration.py | 五指开合标定与拇指对掌几何 |
| hand_control/stabilization.py | 五指迟滞防抖及拇指自动归零状态机 |
| deploy/export_onnx_models.py | 从 OpenMMLab 模型导出三个普通 ONNX |
| deploy/build_tensorrt_engines.sh | 在目标 Jetson 上生成三个 FP16 engine |
| deploy/requirements-jetson-runtime.txt | Jetson 额外 Python 运行时依赖 |
| tests/test_rtmpose_backend.py | 视觉几何与解码单元测试 |
| tests/test_finger_segmentation.py | 相邻手指独立分割和误弯曲否决测试 |
| tests/test_hand_calibration.py | 五指标定、迟滞和拇指自动归零测试 |
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

## Jetson 分阶段测试与运行

以下流程使用当前 Jetson 上已经验证的 engine 布局。每次重新打开终端，都必须先
重新执行变量定义；否则 `$DET_ENGINE` 等空变量会展开成错误的
`/rtmdet_nano.engine`。

### 0. 定义路径和运行环境

```bash
REPO="$HOME/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT"
DET_ENGINE="$REPO/models/tensorrt_orin_nano_20261102_170600/rtmdet_nano.engine"
BODY_ENGINE="$REPO/models/tensorrt_pose_fp32/rtmpose_body.engine"
HAND_ENGINE="$REPO/models/tensorrt_pose_fp32/rtmpose_hand.engine"
BRAINCO_SDK="$HOME/ws/brainco-hand-sdk-main/python"

cd "$REPO"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/ws/pyorbbecsdk-2-main:$HOME/ws/RM_API2-main/Demo/RMDemo_Python:$REPO:$REPO/scripts:${PYTHONPATH:-}"
```

可以先确认路径没有展开为空：

```bash
printf 'DET=%s\nBODY=%s\nHAND=%s\nBRAINCO=%s\n' \
  "$DET_ENGINE" "$BODY_ENGINE" "$HAND_ENGINE" "$BRAINCO_SDK"
```

### 1. 纯 TensorRT 视觉测试

该入口只验证 RTMDet、身体姿态、左右手 ROI 和 Hand5 关键点，不加载 Orbbec
深度、ROS、机械臂或灵巧手：

```bash
cd "$REPO"

python scripts/test_tensorrt_vision.py \
  --input 0 \
  --detector-engine "$DET_ENGINE" \
  --body-engine "$BODY_ENGINE" \
  --hand-engine "$HAND_ENGINE" \
  --body-threshold 0.25 \
  --hand-threshold 0.25
```

图片或视频也可以作为输入：

```bash
python scripts/test_tensorrt_vision.py \
  --input /path/to/person.jpg \
  --detector-engine "$DET_ENGINE" \
  --body-engine "$BODY_ENGINE" \
  --hand-engine "$HAND_ENGINE" \
  --body-threshold 0.25 \
  --hand-threshold 0.25 \
  --output /tmp/rtmpose_result.jpg \
  --no-display

python scripts/test_tensorrt_vision.py \
  --input /path/to/test.mp4 \
  --detector-engine "$DET_ENGINE" \
  --body-engine "$BODY_ENGINE" \
  --hand-engine "$HAND_ENGINE" \
  --body-threshold 0.25 \
  --hand-threshold 0.25
```

### 2. 完整 RGB-D 视觉链，关闭所有动作

该阶段加载 Orbbec 深度和完整 ROS 节点，但明确关闭机械臂和灵巧手输出：

```bash
cd "$REPO"

python scripts/camera_arm_rm65b_control_rtmpose.py \
  --ros-args \
  -p vision_backend:=rtmpose \
  -p rtmpose_runtime:=tensorrt \
  -p rtmdet_engine_path:="$DET_ENGINE" \
  -p rtmpose_body_engine_path:="$BODY_ENGINE" \
  -p rtmpose_hand_engine_path:="$HAND_ENGINE" \
  -p rtmpose_body_keypoint_threshold:=0.25 \
  -p rtmpose_hand_keypoint_threshold:=0.25 \
  -p rtmpose_hand_roi_max_size_px:=700 \
  -p rtmpose_hand_inference_interval:=1 \
  -p control_hand_side:=right \
  -p robot_motion_enabled:=false \
  -p hand_control_enabled:=false \
  -p auto_start_on_ready:=false \
  -p show_window:=true
```

最近操作者稳定后按 `s`。另开一个终端查看五指弯曲度：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hand_finger_bends
```

`hand_finger_bends` 数据顺序固定为：

```text
thumb, index, middle, ring, pinky
```

### 3. 只连接 BrainCo 灵巧手，不发送运动

该阶段用于验证 Revo2 独立 USB-RS485、SDK 路径、串口和从站地址。不要按 `s`：

```bash
cd "$REPO"

python scripts/camera_arm_rm65b_control_rtmpose.py \
  --ros-args \
  -p vision_backend:=rtmpose \
  -p rtmpose_runtime:=tensorrt \
  -p rtmdet_engine_path:="$DET_ENGINE" \
  -p rtmpose_body_engine_path:="$BODY_ENGINE" \
  -p rtmpose_hand_engine_path:="$HAND_ENGINE" \
  -p rtmpose_body_keypoint_threshold:=0.25 \
  -p rtmpose_hand_keypoint_threshold:=0.25 \
  -p rtmpose_hand_roi_max_size_px:=700 \
  -p rtmpose_hand_inference_interval:=1 \
  -p control_hand_side:=right \
  -p robot_motion_enabled:=false \
  -p hand_control_enabled:=true \
  -p hand_transport:=brainco-rs485 \
  -p hand_rs485_sdk_path:="$BRAINCO_SDK" \
  -p hand_rs485_port:=auto \
  -p hand_rs485_slave_id:=0 \
  -p hand_rs485_baudrate:=460800 \
  -p hand_configure_tool_voltage:=false \
  -p hand_open_on_start:=false \
  -p hand_open_on_home:=false \
  -p hand_open_on_lost:=false \
  -p hand_open_on_exit:=false \
  -p auto_start_on_ready:=false \
  -p show_window:=true
```

通信正常时应看到类似日志：

```text
BrainCo Revo2 RS485 device OK
BrainCo Revo2 RS485 finger unit mode set to Normalized
Dexterous hand follower started
```

### 4. 当前完整灵巧手随动命令

下面是当前代码对应的完整调优命令。机械臂运动仍保持关闭，只驱动 BrainCo 灵巧
手。该命令使用 `hand_position_min:=0`、`hand_position_max:=1000`，属于完整行程；
首次硬件验收如需限幅，可临时把 `hand_position_max` 改成 `350`。

```bash
cd "$REPO"

python scripts/camera_arm_rm65b_control_rtmpose.py \
  --ros-args \
  -p vision_backend:=rtmpose \
  -p rtmpose_runtime:=tensorrt \
  -p rtmdet_engine_path:="$DET_ENGINE" \
  -p rtmpose_body_engine_path:="$BODY_ENGINE" \
  -p rtmpose_hand_engine_path:="$HAND_ENGINE" \
  -p rtmpose_body_keypoint_threshold:=0.25 \
  -p rtmpose_hand_keypoint_threshold:=0.28 \
  -p rtmpose_hand_keypoint_tracking_threshold:=0.08 \
  -p rtmpose_hand_keypoint_hold_frames:=0 \
  -p rtmpose_hand_keypoint_smoothing:=0.25 \
  -p rtmpose_hand_keypoint_fast_motion_px:=14.0 \
  -p rtmpose_hand_roi_scale:=2.25 \
  -p rtmpose_hand_roi_wrist_extension:=0.35 \
  -p rtmpose_hand_tracking_roi_scale:=1.55 \
  -p rtmpose_hand_roi_min_size_px:=96 \
  -p rtmpose_hand_roi_max_size_px:=700 \
  -p rtmpose_hand_inference_interval:=1 \
  -p finger_segmentation_enabled:=true \
  -p finger_segmentation_corridor_width_ratio:=0.08 \
  -p finger_segmentation_color_mad_scale:=4.0 \
  -p finger_segmentation_minimum_chroma_tolerance:=9.0 \
  -p finger_segmentation_debug_overlay:=true \
  -p finger_segmentation_fusion_enabled:=true \
  -p finger_segmentation_min_confidence:=0.45 \
  -p finger_segmentation_straight_threshold:=0.78 \
  -p finger_segmentation_full_straight_threshold:=0.92 \
  -p finger_segmentation_bend_cap:=0.12 \
  -p control_hand_side:=right \
  -p control_hand_fallback_enabled:=false \
  -p robot_motion_enabled:=false \
  -p hand_control_enabled:=true \
  -p hand_transport:=brainco-rs485 \
  -p hand_rs485_sdk_path:="$BRAINCO_SDK" \
  -p hand_rs485_port:=auto \
  -p hand_rs485_slave_id:=0 \
  -p hand_rs485_baudrate:=460800 \
  -p hand_rs485_duration_ms:=200 \
  -p hand_rs485_speed:=300 \
  -p hand_position_min:=0 \
  -p hand_position_max:=1000 \
  -p hand_bend_calibration_enabled:=true \
  -p 'hand_bend_open_calibration:=[0.18,0.08,0.13,0.09,0.13]' \
  -p 'hand_bend_closed_calibration:=[0.56,0.92,0.91,0.86,0.87]' \
  -p 'hand_bend_gamma:=[1.0,1.0,1.0,1.0,1.0]' \
  -p hand_bend_deadzone:=0.12 \
  -p hand_bend_release_threshold:=0.05 \
  -p hand_bend_jitter_deadband:=0.025 \
  -p hand_send_hz:=10.0 \
  -p hand_smoothing:=0.50 \
  -p hand_bend_smoothing:=0.60 \
  -p hand_bend_max_delta:=0.30 \
  -p hand_deadband:=10 \
  -p hand_require_3d_bends_for_control:=true \
  -p hand_thumb_rotate_mode:=opposition \
  -p hand_thumb_opposition_open_ratio:=1.05 \
  -p hand_thumb_opposition_closed_ratio:=0.35 \
  -p hand_thumb_opposition_gamma:=1.0 \
  -p hand_thumb_rotate_scale:=0.35 \
  -p hand_thumb_straight_lock_enabled:=true \
  -p hand_thumb_auto_zero_enabled:=true \
  -p hand_thumb_auto_zero_samples:=5 \
  -p hand_thumb_auto_zero_margin:=0.05 \
  -p hand_thumb_bend_engage_threshold:=0.25 \
  -p hand_thumb_bend_release_threshold:=0.08 \
  -p hand_thumb_rotate_engage_threshold:=0.32 \
  -p hand_thumb_rotate_release_threshold:=0.12 \
  -p hand_thumb_state_confirm_frames:=3 \
  -p hand_thumb_jitter_deadband:=0.05 \
  -p hand_reject_single_finger_spikes:=true \
  -p hand_single_finger_spike_delta:=0.22 \
  -p hand_single_finger_spike_peer_max:=0.22 \
  -p hand_spike_confirm_frames:=2 \
  -p hand_reject_multi_finger_spikes:=true \
  -p hand_multi_finger_spike_delta:=0.30 \
  -p hand_multi_finger_spike_min_count:=2 \
  -p hand_multi_finger_spike_confirm_frames:=2 \
  -p hand_closure_confirm_frames:=2 \
  -p hand_closure_confirm_delta:=0.25 \
  -p hand_closure_confirm_value:=0.45 \
  -p hand_closure_confirm_min_fingers:=2 \
  -p hand_open_on_start:=true \
  -p hand_open_on_home:=true \
  -p hand_open_on_lost:=true \
  -p hand_lost_open_delay_s:=1.0 \
  -p hand_open_on_exit:=true \
  -p hand_configure_tool_voltage:=false \
  -p auto_start_on_ready:=false \
  -p show_window:=true
```

开始跟踪前将拇指完全绷直并保持静止，按 `s` 后继续保持至少 5 个有效识别帧，
直到终端出现：

```text
Thumb auto-zero ready: bend_baseline=..., rotate_baseline=...
```

之后再测试拇指弯曲和对掌。GUI 中按 `h` 返回安全状态，按 `q` 或 `Esc` 退出。
只有确认最近操作者、左右手标签、深度、手指分割线和坐标方向都稳定后，才应考虑
开启机械臂运动。

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
| rtmpose_hand_tracking_roi_scale | 1.55 | 首次识别后按上一帧手点生成紧凑 ROI，提高单指弯曲的输入分辨率 |
| rtmpose_hand_inference_interval | 1 | 双手推理间隔；逐帧推理可避免快速屈指被跳过 |
| rtmpose_hand_keypoint_hold_frames | 0 | 低置信度点复用帧数；手指控制默认禁用旧姿态滞留 |
| rtmpose_hand_keypoint_tracking_threshold | 0.08 | 已检测手点的低置信度连续跟踪阈值，低于主阈值时仍使用当前坐标而不是旧直指坐标 |
| rtmpose_hand_keypoint_smoothing | 0.25 | 小幅关键点抖动的自适应平滑系数 |
| rtmpose_hand_keypoint_fast_motion_px | 14 | 超过该位移时快速释放平滑，保证真实屈指及时响应 |
| finger_segmentation_enabled | true | 启用掌心自适应颜色掩膜与逐指独立窄通道测量 |
| finger_segmentation_corridor_width_ratio | 0.11 | 单指分割通道半宽相对掌宽；减小可降低相邻手指串扰 |
| finger_segmentation_color_mad_scale | 4.0 | 掌心颜色鲁棒偏差倍数，光照变化较大时可适当增大 |
| finger_segmentation_minimum_chroma_tolerance | 9.0 | YCrCb 色度最小容差 |
| finger_segmentation_debug_overlay | true | 在画面中显示 I/M/R/P 独立延伸线与比例 |
| finger_segmentation_fusion_enabled | true | 使用独立轮廓的伸直证据否决对应手指的误弯曲 |
| finger_segmentation_min_confidence | 0.45 | 分割证据参与关键点结果融合的最低可信度 |
| finger_segmentation_straight_threshold | 0.78 | 从该延伸比例开始逐步抑制该手指的误弯曲 |
| finger_segmentation_full_straight_threshold | 0.92 | 达到该比例时把该手指弯曲量限制到 bend cap |
| finger_segmentation_bend_cap | 0.12 | 明确伸直手指允许保留的最大原始弯曲量 |
| person_min_depth_m / person_max_depth_m | 0.35 / 4.0 | 操作者深度范围 |
| person_track_min_iou | 0.08 | 目标锁定最小 IoU |
| control_hand_side | right | left、right 或 auto |
| robot_motion_enabled | false | 必须显式开启机械臂动作 |
| hand_control_enabled | false | 必须显式开启灵巧手动作 |
| max_arm_axis_offset_m | `[0.020,0.045,0.020]` | 相对启动 TCP 的 XYZ 分轴最大位移；当前映射中第二项 Y 是左右方向 |
| arm_axis_max_velocity_m_s | `[0.035,0.050,0.035]` | MIT 外环 XYZ 分轴最大速度，单位 m/s |
| arm_axis_max_accel_m_s2 | `[0.160,0.240,0.160]` | MIT 外环 XYZ 分轴最大加速度，单位 m/s² |
| max_arm_target_delta_axis_m | `[0.004,0.006,0.004]` | 每条机械臂命令允许的 XYZ 分轴最大目标变化，单位 m |
| hand_bend_calibration_enabled | true | 在发送硬件命令前启用逐指开合标定；ROS 话题仍保留原始弯曲度 |
| hand_bend_open_calibration | `[0.18,0.08,0.13,0.09,0.13]` | 完全张开标定，顺序为 thumb/index/middle/ring/pinky |
| hand_bend_closed_calibration | `[0.56,0.92,0.91,0.86,0.87]` | 完全握拳标定，顺序同上 |
| hand_bend_gamma | `[1,1,1,1,1]` | 逐指响应曲线；大于 1 会减弱中段，小于 1 会增强中段 |
| hand_bend_deadzone | 0.10 | 标定后弯曲量进入弯曲状态的阈值 |
| hand_bend_release_threshold | 0.04 | 已弯曲手指返回伸直状态的较低阈值，和进入阈值形成迟滞区 |
| hand_bend_jitter_deadband | 0.025 | 已激活手指的微小弯曲量变化保持不动，抑制电机抖动 |
| hand_spike_confirm_frames | 2 | 单指大幅弯曲连续两帧即确认，避免把真实食指/中指动作长期当成尖峰 |
| hand_thumb_rotate_mode | opposition | `opposition` 根据拇指尖相对掌宽独立计算旋转；`scaled-bend` 保留旧映射 |
| hand_thumb_rotate_scale | 0.35 | 拇指旋转轴最大比例，首次标定建议保持 0.35 |
| hand_thumb_straight_lock_enabled | true | 仅对拇指启用自动归零、迟滞和连续帧确认，不影响其余四指 |
| hand_thumb_auto_zero_enabled | true | 每次开始跟踪时学习操作者拇指绷直状态的非零视觉偏差 |
| hand_thumb_auto_zero_samples | 5 | 开始跟踪后用于自动归零的有效帧数；这几帧必须保持拇指绷直 |
| hand_thumb_auto_zero_margin | 0.04 | 在学习基线之上额外忽略的拇指噪声范围 |
| hand_thumb_bend_engage_threshold / release_threshold | 0.22 / 0.08 | 拇指弯曲轴进入/退出动作状态的双阈值 |
| hand_thumb_rotate_engage_threshold / release_threshold | 0.30 / 0.12 | 拇指旋转轴进入/退出动作状态的双阈值 |
| hand_thumb_state_confirm_frames | 3 | 拇指动作必须连续超过进入阈值的帧数 |
| hand_thumb_jitter_deadband | 0.04 | 拇指已动作状态下保持不变的微小波动范围 |

四个分轴参数均可通过 ROS 2 终端参数覆盖，数组顺序固定为机械臂
`[X（前后）, Y（左右）, Z（上下）]`：

```bash
-p 'max_arm_axis_offset_m:=[0.020,0.045,0.020]' \
-p 'arm_axis_max_velocity_m_s:=[0.035,0.050,0.035]' \
-p 'arm_axis_max_accel_m_s2:=[0.160,0.240,0.160]' \
-p 'max_arm_target_delta_axis_m:=[0.004,0.006,0.004]'
```

这些参数采用逐轴裁剪，不再使用单个球形范数限制；因此某一方向达到上限时，
不会再按比例压缩另外两个方向的有效范围。

## 测试

不连接 GPU、相机和机械臂即可执行纯 NumPy/OpenCV 几何测试：

    python3 -m unittest discover -s tests -v

这些测试覆盖阈值与关键点映射、手 ROI、推理降频、NMS、SimCC 解码和原始
RTMDet 输出解码，以及相邻手指的独立轮廓分割。真实 engine 兼容性、FPS、
Orbbec 深度质量和机械臂动作仍需在
目标 Jetson 上分阶段验收。

## ROS 输出

- operator_torso_depth：锁定操作者的躯干深度。
- operator_skeleton_3d：COCO 17 点，每点为 x、y、z、score。
- operator_arm_joint_angles：左右肩、肘角度。
- operator_dual_hand_bends：左右手各五指弯曲度。
- operator_dual_palm_xyz：左右掌心三维位置。
- hand_finger_bends：当前控制手五指弯曲度。
- hand_palm_direction_distances：当前控制手六方向位移。
