'''
# RM65-B Neural-Accelerated Hand Palm Control

This demo combines the uploaded Orbbec RGB-D + MediaPipe hand tracking code with
the RealMan Python SDK. It uses the detected palm 3D displacement to move the
RM65-B TCP in Cartesian space, and can also map detected finger bends to the
dexterous hand mounted on the robot flange.

This neural-accelerated variant keeps the original GUI/ROS parameters/key
controls, but uses a lighter MediaPipe inference profile, optional nearest-ROI
resizing before neural landmark inference, and a vectorized neural-kinematics
adapter that can load a trained ONNX landmark-to-bend regressor when available.

## Control rule

- The first valid palm position is used as the hand reference point.
- The robot moves to `safe_home_joints` at startup, then that TCP pose is used
  as the robot reference pose.
- Palm displacement is transformed by `camera_to_arm_matrix`.
- The robot target offset is quantized by `arm_motion_unit_m`.
- The target pose is sent with the selected control protocol, while the TCP
  orientation is kept unchanged from startup.
- `arm_control_protocol:=rs485` uses planned Cartesian motion through `rm_movel`.
- `arm_control_protocol:=canfd` uses pose pass-through through `rm_movep_canfd`.
- Finger bend control runs in a separate worker thread from arm motion control.
- RealMan hand command order is `[pinky, ring, middle, index, thumb_bend,
  thumb_rotate]`, matching the official six-DOF hand interface.
- By default the hand uses `rm_set_hand_follow_pos`; set
  `hand_use_follow_pos:=false` to use `rm_set_hand_angle` instead.
- For a BrainCo Revo2 Basic hand that is wired to the RM tool RS485 port, set
  `hand_transport:=raw-modbus` to write the Revo2 fast-position registers
  directly through the RM_API2 Modbus RTU helpers.
- For a BrainCo Revo2 Basic hand wired to a Linux SocketCAN CANFD adapter, set
  `hand_transport:=brainco-canfd` so the hand no longer shares the RM tool
  RS485/Modbus channel with arm control.
- For a BrainCo Revo2 Basic hand wired to an independent Jetson USB-RS485
  adapter, set `hand_transport:=brainco-rs485` to control it through BrainCo's
  official RS485 SDK while the RM65-B arm uses RealMan CANFD motion.
- Press `h` in the display window to return to `safe_home_joints`.
- Before sending a palm target pose, the demo reads the official joint limits
  from the controller, solves IK for the target pose, and refuses targets whose
  current or target joint angles exceed those limits with the configured margin.

Default camera coordinates are:

- `x`: image right
- `y`: image down
- `z`: away from the camera

Default robot mapping is:

```text
arm_x = camera_z
arm_y = -camera_x
arm_z = -camera_y
```

Adjust `camera_to_arm_matrix` after camera-to-robot calibration.

## Run

From the repository root:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py
```

The demo starts in detection-only mode. To allow robot motion:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=rs485 \
  -p safe_home_joints:="[-2.349,-91.303,99.533,2.447,84.25,0.032]" \
  -p arm_motion_unit_m:=0.01 \
  -p max_arm_offset_m:=0.15 \
  -p arm_speed:=10
```

To use CANFD pose pass-through:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p canfd_follow:=false \
  -p canfd_trajectory_mode:=0 \
  -p canfd_radio:=0 \
  -p arm_command_interval_s:=0.05
```

To control both the RM65-B arm through CANFD and the mounted dexterous hand
through the RealMan hand interface:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p arm_command_interval_s:=0.05 \
  -p hand_control_enabled:=true \
  -p hand_position_min:=0 \
  -p hand_position_max:=1000
```

To control a BrainCo Revo2 Basic hand through the RM tool RS485 port with raw
Modbus RTU register writes:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p arm_command_interval_s:=0.05 \
  -p hand_control_enabled:=true \
  -p hand_transport:=raw-modbus \
  -p hand_raw_modbus_generation:=auto \
  -p hand_raw_modbus_slave_id:=0 \
  -p hand_raw_modbus_baudrate:=460800 \
  -p hand_tool_voltage_type:=3
```

To control both the RM65-B arm and BrainCo Revo2 Basic hand through CANFD:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p canfd_follow:=false \
  -p canfd_trajectory_mode:=2 \
  -p canfd_radio:=50 \
  -p arm_command_interval_s:=0.04 \
  -p hand_control_enabled:=true \
  -p hand_transport:=brainco-canfd \
  -p hand_canfd_iface:=can0 \
  -p hand_canfd_slave_id:=127 \
  -p hand_canfd_duration_ms:=70 \
  -p hand_wait_for_first_arm_command:=false \
  -p hand_pause_while_arm_sdk_busy:=false \
  -p hand_configure_tool_voltage:=true \
  -p hand_tool_voltage_type:=3
```

To control the RM65-B arm through RealMan CANFD and the BrainCo Revo2 Basic
hand through BrainCo's official RS485 SDK on an independent USB-RS485 adapter:

```bash
python Demo/RMDemo_Python/RMDemo_HandPalmControl/src/core/hand_palm_rm65_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p canfd_follow:=false \
  -p canfd_trajectory_mode:=2 \
  -p canfd_radio:=50 \
  -p arm_command_interval_s:=0.04 \
  -p hand_control_enabled:=true \
  -p hand_transport:=brainco-rs485 \
  -p hand_rs485_sdk_path:=/home/jetson/ws/brainco-hand-sdk-main/python \
  -p hand_rs485_port:=/dev/ttyUSB0 \
  -p hand_rs485_slave_id:=127 \
  -p hand_rs485_baudrate:=460800 \
  -p hand_rs485_duration_ms:=60 \
  -p hand_wait_for_first_arm_command:=false \
  -p hand_pause_while_arm_sdk_busy:=false \
  -p hand_configure_tool_voltage:=true \
  -p hand_tool_voltage_type:=3
```

## Important parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `robot_motion_enabled` | `false` | Must be `true` before sending RM65-B motion commands. |
| `robot_ip` | `192.168.1.18` | RM65-B controller IP. |
| `robot_port` | `8080` | RM65-B controller port. |
| `color_width` | `1280` | Requested Orbbec color stream width. Falls back automatically if unsupported. |
| `color_height` | `720` | Requested Orbbec color stream height. |
| `color_fps` | `30` | Requested Orbbec color stream FPS. |
| `depth_width` | `0` | Requested Orbbec depth stream width. `0` keeps device/default profile selection. |
| `depth_height` | `0` | Requested Orbbec depth stream height. |
| `depth_fps` | `0` | Requested Orbbec depth stream FPS. `0` means wildcard/default. |
| `mp_hands_model_complexity` | `0` | MediaPipe Hands model complexity used by this neural-accelerated script. Use `1` if accuracy is more important than speed. |
| `mp_pose_enabled` | `false` | Enable MediaPipe Pose landmarks. Keep this off for fastest hand/palm control and to avoid downloading pose models into system site-packages. |
| `mp_pose_backend` | `auto` | Pose backend: `auto`, `tasks` for new `.task` PoseLandmarker models, or `solutions` for legacy `.tflite` models. |
| `mp_pose_model_complexity` | `1` | MediaPipe Pose model complexity when `mp_pose_enabled` is true. |
| `mp_pose_model_path` | empty | Optional local Pose `.tflite` file. When set, the script installs it into MediaPipe's expected model filename before Pose initialization. |
| `mp_pose_task_model_path` | empty | Optional local PoseLandmarker `.task` file for the new MediaPipe Tasks backend. |
| `nn_inference_resize_enabled` | `true` | Resize the isolated nearest-person ROI before neural landmark inference. |
| `nn_inference_max_side_px` | `384` | Maximum ROI side length sent into MediaPipe neural inference. |
| `nn_hand_bend_model_path` | empty | Optional ONNX model path. Input must be normalized 21 hand landmarks flattened to 63 floats; output first 5 values are thumb/index/middle/ring/pinky bends. |
| `nn_hand_bend_blend` | `0.85` | Blend ratio for ONNX bend prediction versus vectorized geometric fallback. |
| `nn_depth_bend_reject_enabled` | `true` | Compare 3D and same-frame 2D finger bends; use 2D when depth makes multiple fingers falsely look closed. |
| `nn_depth_bend_reject_delta` | `0.35` | Minimum 3D-minus-2D bend gap for depth-noise rejection. |
| `nn_depth_bend_reject_2d_max` | `0.35` | Only reject 3D bend spikes when the same finger remains mostly open in 2D. |
| `nn_depth_bend_reject_min_count` | `2` | Number of non-thumb fingers that must disagree before rejecting noisy 3D bends. |
| `nn_skip_arm_angle_log` | `true` | Skip elbow-angle logging during tracking to reduce non-control CPU work. |
| `arm_control_protocol` | `rs485` | Motion command protocol, either `rs485` or `canfd`. |
| `arm_rs485_motion_mode` | `movej_p` | RS485 arm backend: `movej_p` uses absolute pose targets through `rm_movej_p`; `movel` uses `rm_movel`; `step` uses `rm_set_pos_step`. |
| `arm_block` | `0` | Whether each palm-follow `rm_movel` waits for completion. Keep this `0` for continuous tracking; `1` can block hand commands and keyboard recovery. |
| `arm_movel_redirect_to_canfd` | `false` | Optional debug fallback: redirect RS485/`movel` targets to `rm_movep_canfd`. Keep this `false` when the arm must be controlled through RS485. |
| `arm_recover_motion_state_on_start` | `true` | Clear controller motion state before startup home. |
| `arm_recover_motion_state_on_tracking_start` | `false` | Optional debug recovery before pressing `s` starts palm following. Keep this off if it stalls on your controller. |
| `arm_slow_stop_on_pause` | `true` | Send `rm_set_arm_slow_stop` when target tracking pauses. |
| `arm_log_command_start` | `true` | Log the exact target pose before entering the blocking RM_API motion call. Useful when diagnosing SDK stalls. |
| `sdk_busy_log_interval_s` | `5.0` | Minimum interval for shared RM SDK busy warnings. |
| `safe_home_joints` | `[-2.349,-91.303,99.533,2.447,84.25,0.032]` | Safe startup/home joint angles for joints 1-6, in degrees. Keep J5 away from 0 deg to avoid the RM65 wrist singularity. |
| `auto_home_on_start` | `true` | Return to `safe_home_joints` after connecting and before palm tracking controls the arm. |
| `home_key_enabled` | `true` | Enable pressing `h` in the OpenCV window to return home. |
| `home_speed` | `20` | Speed percentage for the safe-home `movej`. |
| `home_verify_timeout_s` | `8.0` | After `h`/startup home, wait up to this long for joints to reach `safe_home_joints` before refreshing the TCP reference. |
| `home_verify_tolerance_deg` | `1.5` | Joint tolerance used by the safe-home verification wait. |
| `joint_safety_enabled` | `true` | Read official joint limits and block unsafe commands. |
| `target_ik_safety_enabled` | `true` | Run IK for target poses and check the target joint angles before sending motion. |
| `singularity_safety_enabled` | `true` | Refuse startup/current/target joints that are too close to the wrist singularity. |
| `wrist_singularity_joint5_min_abs_deg` | `15.0` | Minimum allowed absolute J5 angle. RM65 wrist singularity occurs near J5=0 deg. |
| `singularity_safe_step_command_interval_s` | `0.8` | Minimum command interval enforced for RS485 step mode while singularity safety is enabled. |
| `target_safety_check_interval_s` | `1.0` | Minimum interval between expensive target safety checks in RS485 step mode. Avoids dropping many tracking commands when `rm_get_joint_degree` occasionally times out. |
| `target_safety_allow_cached_on_read_error` | `true` | In bounded RS485 step tracking, keep using the last successful safety state when a current-joint read times out. |
| `joint_limit_margin_deg` | `2.0` | Safety margin inside the official min/max joint limits. |
| `arm_motion_unit_m` | `0.01` | Robot motion base length unit, in meters. Example: `0.01` means 1 cm quantization. |
| `palm_to_arm_scale` | `1.0` | Scale from palm displacement to robot displacement. |
| `arm_mit_control_enabled` | `true` | Enable a Cartesian outer-loop spring-damper filter before sending RS485/CANFD targets. This is MIT-style trajectory shaping, not low-level torque control. |
| `arm_mit_kp` | `70.0` | Virtual spring gain for the outer-loop target filter. Larger values track faster. |
| `arm_mit_kd` | `16.0` | Virtual damping gain for the outer-loop target filter. Larger values suppress overshoot. |
| `arm_mit_max_velocity_m_s` | `0.22` | Maximum filtered Cartesian target velocity. |
| `arm_mit_max_accel_m_s2` | `1.5` | Maximum filtered Cartesian target acceleration. |
| `p` key | n/a | While waiting for `s`, save GUI trackbar values to `camera_arm_rm65b_control.defaults.json`; omit those GUI-tuned parameters from the next launch command to reuse the saved defaults. |
| `arm_axis_scale` | `[1.0,1.4,1.4]` | Extra per-axis robot-base scale after `camera_to_arm_matrix`; useful for making left/right or up/down tracking more sensitive. |
| `arm_axis_deadband_m` | `[0.003,0.003,0.003]` | Per-axis robot-base deadband before quantization. |
| `hand_deadband_m` | `0.005` | Palm displacement below this value is treated as zero. |
| `palm_filter_alpha` | `0.55` | Anti-drift palm filter movement alpha. Higher values respond faster. |
| `palm_filter_hold_deadband_m` | `0.008` | Hold the arm still when filtered palm motion stays inside this range. |
| `palm_filter_release_deadband_m` | `0.014` | Release the anti-drift hold when palm motion exceeds this range. |
| `max_arm_offset_m` | `0.15` | Maximum distance from startup TCP pose. |
| `max_arm_target_delta_m` | `0.02` | Maximum Cartesian target change per command. Limits sudden camera/landmark jumps. |
| `arm_command_interval_s` | `0.15` | Minimum interval between robot target updates. |
| `arm_rs485_step_min_command_interval_s` | `0.10` | Minimum effective command interval for RS485 `rm_set_pos_step`, even if GUI `Cmd_ms` is lower. Prevents flooding the controller. |
| `arm_rs485_step_axis_interval_s` | `0.02` | Small spacing between X/Y/Z `rm_set_pos_step` commands in one Cartesian update. |
| `arm_rs485_step_max_axes_per_update` | `1` | Maximum Cartesian axes sent in one `rm_set_pos_step` update. `1` sends only the dominant axis and is safer for shared Revo2 raw-Modbus control. |
| `arm_rs485_step_axis_switch_ratio` | `1.4` | Hysteresis for switching the dominant step axis. Larger values reduce jitter from small palm-noise axis changes. |
| `arm_rs485_step_axis_reverse_deadband_m` | `0.003` | Ignore tiny immediate reversals on the same step axis to reduce visible shaking. |
| `arm_rs485_step_fallback_to_movel` | `true` | Automatically switch future RS485 arm commands to `rm_movel` if `rm_set_pos_step` stalls or returns an error. This is forced off when Revo2 raw-Modbus hand control is enabled. |
| `arm_rs485_step_fallback_after_stall_s` | `1.5` | Step-command elapsed-time threshold that triggers the automatic `movel` fallback. |
| `canfd_follow` | `false` | CANFD follow mode. `true` is high-follow and needs a very stable fast cycle. |
| `canfd_trajectory_mode` | `0` | CANFD high-follow trajectory mode: `0` pass-through, `1` curve fitting, `2` filtering. |
| `canfd_radio` | `0` | CANFD smoothing/filtering parameter for trajectory modes. |
| `rs485_configure_on_connect` | `false` | Whether to configure controller RS485 mode at startup. |
| `rs485_mode` | `0` | Controller RS485 mode when configuring: `0` serial, `1` Modbus RTU master, `2` Modbus RTU slave. |
| `rs485_baudrate` | `115200` | Controller RS485 baudrate when configuring. |
| `camera_to_arm_matrix` | `[0,0,1,-1,0,0,0,-1,0]` | 3x3 row-major transform from camera displacement to robot base displacement. |
| `hand_control_enabled` | `false` | Must be `true` before sending RealMan dexterous hand commands. It reuses the RM65-B SDK connection, so `robot_motion_enabled` must also be `true`. |
| `hand_use_dedicated_connection` | `false` | Use the same RM_API2 connection as the arm by default. For Revo2 `raw-modbus` plus arm motion, a dedicated second connection is automatically overridden unless `hand_force_shared_connection_for_raw_modbus:=false`. |
| `hand_force_shared_connection_for_raw_modbus` | `true` | Force Revo2 raw-Modbus hand commands and RM65-B arm motion through one serialized RM_API connection to avoid controller/tool-RS485 contention. |
| `hand_robot_ip` | `robot_ip` | RM controller IP for the dedicated hand connection. |
| `hand_robot_port` | `robot_port` | RM controller port for the dedicated hand connection. |
| `hand_transport` | `realman-hand` | Hand transport backend. `realman-hand` keeps the original RM generic hand API; `raw-modbus` writes BrainCo Revo2 fast-position registers through the RM tool RS485 port; `brainco-canfd` sends Revo2 commands through BrainCo SocketCAN CANFD; `brainco-rs485` sends Revo2 commands through BrainCo's official RS485 SDK on an independent serial adapter. |
| `hand_use_follow_pos` | `true` | Use `rm_set_hand_follow_pos`; set `false` to use `rm_set_hand_angle`. |
| `hand_position_min` | `0` | Fully open position value sent to each hand DOF. |
| `hand_position_max` | `1000` | Fully closed position value sent to each hand DOF. |
| `hand_thumb_rotate_scale` | `1.0` | Scale applied from thumb bend to the thumb rotation DOF. |
| `hand_send_hz` | `35.0` | Maximum dexterous hand command rate. |
| `hand_wait_for_first_arm_command` | `true` | During each `s` tracking cycle, hold hand writes until the first arm tracking command succeeds. Helps avoid startup contention on the controller/tool RS485 path. |
| `hand_wait_for_first_arm_command_timeout_s` | `1.0` | If the palm stays still and no arm command is pending, release the startup hand gate after this many seconds. |
| `hand_pause_while_arm_sdk_busy` | `true` | Temporarily hold new hand writes while an arm RM_API motion call is in flight. Disable only after confirming the arm no longer stalls. |
| `hand_pause_while_arm_motion_pending` | `false` | Also hold hand writes when an arm target is queued but not yet sent. Keep this `false` for shared raw-Modbus control, otherwise continuous palm targets can starve the hand. |
| `hand_smoothing` | `0.90` | Exponential smoothing alpha for hand positions. Higher values respond faster. |
| `hand_deadband` | `1` | Minimum position change before sending another hand command. |
| `Hand_hz` GUI | n/a | GUI trackbar for `hand_send_hz`, only editable before pressing `s`. |
| `Hand_alpha_pct` GUI | n/a | GUI trackbar for `hand_smoothing`. Higher values make the hand respond faster but may pass more vision noise. |
| `Hand_db` GUI | n/a | GUI trackbar for `hand_deadband`. Lower values make small finger changes respond sooner. |
| `hand_bend_deadzone` | `0.10` | Finger bend values below this are treated as open-hand noise. |
| `hand_bend_smoothing` | `0.65` | Low-pass alpha applied to detected finger bends before converting to hand positions. Higher values respond faster. |
| `hand_bend_max_delta` | `0.60` | Maximum bend-value change per detection update. Larger values allow quicker finger closing/opening. |
| `hand_require_3d_bends_for_control` | `true` | Only send hand commands when finger bends come from reliable 3D landmarks. |
| `hand_reject_single_finger_spikes` | `true` | Reject isolated non-thumb finger spikes unless they persist for `hand_spike_confirm_frames`. |
| `hand_spike_confirm_frames` | `3` | Consecutive frames needed to accept an isolated finger-bend spike. |
| `hand_reject_multi_finger_spikes` | `true` | Hold sudden multi-finger bend jumps until they persist for several frames. |
| `hand_multi_finger_spike_delta` | `0.30` | Per-finger bend jump threshold used by the multi-finger spike rejector. |
| `hand_multi_finger_spike_min_count` | `2` | Minimum number of non-thumb fingers that must jump together before multi-finger spike filtering applies. |
| `hand_multi_finger_spike_confirm_frames` | `6` | Consecutive frames needed to accept a sudden multi-finger closure. |
| `hand_closure_confirm_frames` | `5` | Consecutive frames needed before accepting large apparent finger closure from vision landmarks. |
| `hand_sdk_lock_timeout_s` | `0.25` | How long the hand worker waits for the shared RM SDK connection before retrying later. |
| `hand_block` | `false` | Whether RealMan hand commands wait for controller acknowledgement. |
| `hand_open_on_start` | `false` | Queue an open-hand command at startup. |
| `hand_open_on_home` | `false` | Open the hand when `h` returns the arm to safe home. |
| `hand_open_on_exit` | `true` | Open the hand during shutdown. |
| `hand_open_on_lost` | `false` | Open the hand after camera tracking is lost for `hand_lost_open_delay_s`. |
| `hand_configure_speed_force` | `false` | Configure RealMan hand speed and force at startup. |
| `hand_speed` | `500` | Hand speed value used when `hand_configure_speed_force` is enabled. |
| `hand_force` | `500` | Hand force value used when `hand_configure_speed_force` is enabled. |
| `hand_configure_tool_voltage` | `true` | Configure tool-end power output for the dexterous hand at startup. |
| `hand_tool_voltage_type` | `3` | Tool-end power output type. RealMan SDK uses `0` for 0V, `1` for 5V, `2` for 12V, and `3` for 24V. |
| `hand_configure_rm_plus_mode` | `false` | Configure the RM Plus end-effector protocol at startup. Leave this off if the robot is already configured correctly. |
| `hand_rm_plus_mode` | `115200` | RM Plus end-effector protocol mode/baudrate when `hand_configure_rm_plus_mode` is enabled. |
| `hand_configure_tool_rs485` | `false` | Configure the tool RS485 port for dexterous hand mode at startup. |
| `hand_tool_rs485_mode` | `1` | Tool RS485 mode used when configuring the dexterous hand link. |
| `hand_tool_rs485_baudrate` | `115200` | Tool RS485 baudrate used when configuring the dexterous hand link. |
| `hand_raw_modbus_generation` | `auto` | RM controller generation for raw Revo2 Modbus: `auto`, `gen3`, or `gen4`. |
| `hand_raw_modbus_slave_id` | `0` | Revo2 Modbus slave ID. `0` auto-probes the factory IDs `127` and `126`. |
| `hand_raw_modbus_baudrate` | `460800` | Baudrate for raw Revo2 Modbus through the tool RS485 port. |
| `hand_raw_modbus_force_arm_step_mode` | `true` | When Revo2 raw-Modbus hand control and RM65-B RS485 arm motion are both enabled, force the arm backend to `rm_set_pos_step` to avoid `rm_movej_p`/`rm_movel` planner calls stalling after tool-Modbus setup. |
| `hand_raw_modbus_configure_tool_port` | `true` | Configure the RM tool RS485 port as Modbus RTU master before probing the Revo2. |
| `hand_raw_modbus_probe_on_start` | `true` | Probe the configured Revo2 slave ID at startup when `hand_raw_modbus_slave_id` is non-zero. |
| `hand_raw_modbus_read_position_on_start` | `true` | Read Revo2 actual-position registers once at startup to verify communication. |
| `hand_canfd_sdk_path` | auto | Optional path to `brainco-hand-sdk/python`; auto-detects common workspace locations. |
| `hand_canfd_iface` | `can0` | Linux SocketCAN interface used by BrainCo Revo2 CANFD. Configure it before launching, e.g. arbitration 1 Mbps and data 5 Mbps. |
| `hand_canfd_slave_id` | `127` | Revo2 CANFD slave ID. Use `126` if your hand is configured as the other factory ID. |
| `hand_canfd_master_id` | `1` | CANFD master ID passed to BrainCo SDK. |
| `hand_canfd_use_builtin_socketcan` | `true` | Prefer BrainCo SDK built-in SocketCAN CANFD support; falls back to the Python callback adapter if unavailable. |
| `hand_canfd_use_durations` | `true` | Send Revo2 position-duration commands for smooth fast motion. |
| `hand_canfd_duration_ms` | `70` | Target duration for each CANFD hand position command. Lower values respond faster; too low may look jerky. |
| `hand_canfd_speed` | `1000` | Speed value used when duration commands are disabled or unavailable. |
| `hand_canfd_command_timeout_s` | `0.20` | Timeout for one BrainCo CANFD SDK command. |
| `hand_canfd_verify_on_start` | `true` | Read Revo2 device info and baudrate during startup to verify the CANFD link. |
| `hand_rs485_sdk_path` | auto | Optional path to `brainco-hand-sdk/python`; auto-detects common workspace locations. |
| `hand_rs485_port` | `auto` | Independent serial port for BrainCo Revo2 RS485, e.g. `/dev/ttyUSB0`; `auto` uses BrainCo SDK auto-detect. |
| `hand_rs485_slave_id` | `127` | Revo2 RS485 slave ID. Set `0` to use the ID returned by BrainCo SDK auto-detect. |
| `hand_rs485_baudrate` | `460800` | BrainCo Revo2 RS485 baudrate when the port is specified manually. |
| `hand_rs485_quick_detect` | `true` | Use fast BrainCo Modbus auto-detection when `hand_rs485_port:=auto` or `hand_rs485_slave_id:=0`. |
| `hand_rs485_use_durations` | `true` | Send Revo2 position-duration commands for smooth fast motion over RS485. |
| `hand_rs485_duration_ms` | `60` | Target duration for each RS485 hand position command. Lower values respond faster; too low may look jerky. |
| `hand_rs485_speed` | `1000` | Speed value used when duration commands are disabled or unavailable. |
| `hand_rs485_command_timeout_s` | `0.20` | Timeout for one BrainCo RS485 SDK command. |
| `hand_rs485_verify_on_start` | `true` | Read Revo2 device info during startup to verify the RS485 link. |

For first real-machine tests, use a small `arm_motion_unit_m`, low `arm_speed`,
and a conservative `max_arm_offset_m`.


python /home/jetson/ws/pyorbbecsdk-2-main/camera_arm_rm65b_control.py \
  --ros-args \
  -p robot_motion_enabled:=true \
  -p robot_ip:=192.168.1.18 \
  -p arm_control_protocol:=canfd \
  -p arm_command_interval_s:=0.05 \
  -p hand_control_enabled:=true \
  -p hand_position_min:=0 \
  -p hand_position_max:=1000


python /home/jetson/ws/pyorbbecsdk-2-main/camera_arm_rm65b_control_neural.py   \
    --ros-args  \
    -p robot_motion_enabled:=true   \
    -p robot_ip:=192.168.1.18   \
    -p arm_control_protocol:=canfd   \
    -p canfd_follow:=false   \
    -p canfd_trajectory_mode:=2   \
    -p canfd_radio:=50   \
    -p arm_command_interval_s:=0.04   \
    -p hand_control_enabled:=true   \
    -p hand_transport:=brainco-rs485   \
    -p hand_rs485_sdk_path:=/home/jetson/ws/brainco-hand-sdk-main/python   \
    -p hand_rs485_port:=auto   \
    -p hand_rs485_slave_id:=0   \
    -p hand_rs485_baudrate:=460800   \
    -p hand_rs485_duration_ms:=60   \
    -p hand_wait_for_first_arm_command:=false   \
    -p hand_pause_while_arm_sdk_busy:=false   \
    -p hand_configure_tool_voltage:=true   \
    -p hand_tool_voltage_type:=3


'''

import collections
import asyncio
import json
import math
import os
import shutil
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

_SCRIPT_DIR = Path(__file__).resolve().parent
for _DEPTH_PACKAGE_ROOT in (_SCRIPT_DIR, _SCRIPT_DIR.parent):
    if (_DEPTH_PACKAGE_ROOT / "lingbot_depth_feature_pack").exists():
        _DEPTH_PACKAGE_ROOT_TEXT = str(_DEPTH_PACKAGE_ROOT)
        if _DEPTH_PACKAGE_ROOT_TEXT not in sys.path:
            sys.path.insert(0, _DEPTH_PACKAGE_ROOT_TEXT)
        break

try:
    from lingbot_depth_feature_pack import DepthEnhancerConfig, LingbotDepthEnhancer
except Exception:
    DepthEnhancerConfig = None
    LingbotDepthEnhancer = None

from pyorbbecsdk import (
    AlignFilter,
    Config,
    Context,
    OBFormat,
    OBFrameAggregateOutputMode,
    OBSensorType,
    OBStreamType,
    Pipeline,
)

try:
    from mediapipe.tasks import python as mp_tasks_python
    from mediapipe.tasks.python import vision as mp_tasks_vision
except Exception:
    mp_tasks_python = None
    mp_tasks_vision = None


GUI_TUNING_KEYS = (
    "arm_mit_control_enabled",
    "arm_mit_kp",
    "arm_mit_kd",
    "arm_mit_max_velocity_m_s",
    "arm_mit_max_accel_m_s2",
    "max_arm_target_delta_m",
    "arm_command_interval_s",
    "hand_deadband_m",
    "arm_axis_deadband_m",
    "palm_filter_alpha",
    "palm_filter_hold_deadband_m",
    "palm_filter_release_deadband_m",
    "hand_bend_deadzone",
    "hand_bend_smoothing",
    "hand_bend_max_delta",
    "hand_send_hz",
    "hand_smoothing",
    "hand_deadband",
)
PERSISTENT_DEFAULTS_PATH = Path(__file__).with_name(
    "camera_arm_rm65b_control.defaults.json"
)


def load_persistent_defaults(logger=None):
    if not PERSISTENT_DEFAULTS_PATH.exists():
        return {}
    try:
        with PERSISTENT_DEFAULTS_PATH.open("r", encoding="utf-8") as defaults_file:
            data = json.load(defaults_file)
    except Exception as exc:
        if logger is not None:
            logger.warn(f"Failed to load persistent defaults: {exc}")
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def save_persistent_defaults(values):
    defaults = load_persistent_defaults()
    defaults.update({key: values[key] for key in GUI_TUNING_KEYS if key in values})
    with PERSISTENT_DEFAULTS_PATH.open("w", encoding="utf-8") as defaults_file:
        json.dump(defaults, defaults_file, indent=2, sort_keys=True)
        defaults_file.write("\n")


def persistent_default(node, name, fallback):
    defaults = getattr(node, "persistent_defaults", {})
    return defaults.get(name, fallback)


def mediapipe_pose_model_filename(model_complexity):
    complexity = int(model_complexity)
    if complexity == 0:
        return "pose_landmark_lite.tflite"
    if complexity == 2:
        return "pose_landmark_heavy.tflite"
    return "pose_landmark_full.tflite"


def install_local_mediapipe_pose_model(
    model_path,
    model_complexity,
    logger=None,
):
    source = Path(str(model_path)).expanduser()
    if not source.exists():
        raise FileNotFoundError(f"mp_pose_model_path does not exist: {source}")
    if not source.is_file():
        raise ValueError(f"mp_pose_model_path is not a file: {source}")

    target_dir = (
        Path(mp.__file__).resolve().parent
        / "modules"
        / "pose_landmark"
    )
    target = target_dir / mediapipe_pose_model_filename(model_complexity)

    if target.exists() and source.resolve() == target.resolve():
        if logger is not None:
            logger.info(f"Using existing MediaPipe Pose model: {target}")
        return target

    target_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    if logger is not None:
        logger.info(f"Installed local MediaPipe Pose model: {source} -> {target}")
    return target


# def _add_rm_sdk_to_path():
#     current = Path(__file__).resolve()
#     for parent in current.parents:
#         sdk_dir = parent / "Python"
#         if (sdk_dir / "Robotic_Arm" / "rm_robot_interface.py").exists():
#             sys.path.insert(0, str(sdk_dir))
#             return
#     raise RuntimeError("Could not find RM_API2/Python/Robotic_Arm SDK directory")


# _add_rm_sdk_to_path()

from Robotic_Arm import rm_robot_interface as rm_api
from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_api_version,
    rm_inverse_kinematics_params_t,
    rm_pos_teach_type_e,
    rm_thread_mode_e,
)


class LatestFrameBuffer:
    def __init__(self, maxsize=2):
        self._queue = collections.deque(maxlen=maxsize)
        self._lock = threading.Lock()

    def put(self, frame_set):
        with self._lock:
            if len(self._queue) == self._queue.maxlen:
                self._queue.popleft()
            self._queue.append(frame_set)

    def get_latest(self):
        with self._lock:
            if not self._queue:
                return None
            latest = self._queue.pop()
            self._queue.clear()
            return latest

    def clear(self):
        with self._lock:
            self._queue.clear()


class OrbbecRgbdSource:
    def __init__(
        self,
        logger,
        frame_buffer,
        color_width=1280,
        color_height=720,
        color_fps=30,
        depth_width=0,
        depth_height=0,
        depth_fps=0,
    ):
        self.logger = logger
        self.frame_buffer = frame_buffer
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.color_fps = int(color_fps)
        self.depth_width = int(depth_width)
        self.depth_height = int(depth_height)
        self.depth_fps = int(depth_fps)
        self.ctx = Context()
        self.pipeline = None
        self.align_filter = None
        self.camera_intrinsic = None
        self.last_frame_time = time.monotonic()
        self.lock = threading.Lock()

    def start(self):
        if not self._device_available():
            raise RuntimeError("No Orbbec camera detected")

        pipeline = Pipeline()
        config = Config()

        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = self._select_video_profile(
            color_profiles,
            self.color_width,
            self.color_height,
            OBFormat.RGB,
            self.color_fps,
            "color",
        )
        config.enable_stream(color_profile)

        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = self._select_video_profile(
            depth_profiles,
            self.depth_width,
            self.depth_height,
            OBFormat.Y16,
            self.depth_fps,
            "depth",
        )
        config.enable_stream(depth_profile)

        try:
            config.set_frame_aggregate_output_mode(
                OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
            )
        except Exception as exc:
            self.logger.warn(f"Full-frame aggregate mode unavailable: {exc}")

        try:
            pipeline.enable_frame_sync()
        except Exception as exc:
            self.logger.warn(f"Frame sync is unavailable: {exc}")

        try:
            align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        except Exception as exc:
            align_filter = None
            self.logger.warn(f"Depth-to-color align is unavailable: {exc}")

        pipeline.start(config, self._on_new_frame)

        try:
            camera_intrinsic = pipeline.get_camera_param().rgb_intrinsic
        except Exception as exc:
            camera_intrinsic = None
            self.logger.warn(f"Failed to read camera intrinsics, using fallback: {exc}")

        with self.lock:
            self.pipeline = pipeline
            self.align_filter = align_filter
            self.camera_intrinsic = camera_intrinsic
            self.last_frame_time = time.monotonic()

        self.frame_buffer.clear()
        self.logger.info(
            "Orbbec camera started "
            f"(color={color_profile.get_width()}x{color_profile.get_height()}, "
            f"depth={depth_profile.get_width()}x{depth_profile.get_height()})"
        )

    def stop(self):
        with self.lock:
            pipeline = self.pipeline
            self.pipeline = None
            self.align_filter = None
            self.camera_intrinsic = None

        self.frame_buffer.clear()
        if pipeline is None:
            return

        try:
            pipeline.stop()
        except Exception as exc:
            self.logger.warn(f"Failed to stop camera cleanly: {exc}")

    def seconds_since_last_frame(self):
        return time.monotonic() - self.last_frame_time

    def get_camera_intrinsic(self):
        with self.lock:
            return self.camera_intrinsic

    def latest_aligned_frames(self):
        frames = self.frame_buffer.get_latest()
        if frames is None:
            return None

        with self.lock:
            align_filter = self.align_filter

        if align_filter is None:
            return frames

        aligned = align_filter.process(frames)
        if aligned is None:
            self.logger.warn("Failed to align frames")
            return None
        return aligned

    def frames_to_images(self, frames):
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            self.logger.warn("Missing color or depth frame")
            return None, None

        color_image = self._color_frame_to_bgr(color_frame)
        depth_meters = self._depth_frame_to_meters(depth_frame)
        return color_image, depth_meters

    def get_3d_point(self, x_pixel, y_pixel, depth_meters):
        height, width = depth_meters.shape[:2]
        if x_pixel < 0 or y_pixel < 0 or x_pixel >= width or y_pixel >= height:
            return None

        depth = float(depth_meters[y_pixel, x_pixel])
        if depth <= 0.0 or math.isnan(depth):
            return None

        fx = self._intrinsic_value("fx", 615.0)
        fy = self._intrinsic_value("fy", 615.0)
        cx = self._intrinsic_value("cx", width / 2.0)
        cy = self._intrinsic_value("cy", height / 2.0)

        x = (x_pixel - cx) * depth / fx
        y = (y_pixel - cy) * depth / fy
        z = depth
        return np.array([x, y, z], dtype=np.float32)

    def _on_new_frame(self, frame_set):
        if frame_set is None:
            return
        self.last_frame_time = time.monotonic()
        self.frame_buffer.put(frame_set)

    def _device_available(self):
        try:
            return self.ctx.query_devices().get_count() > 0
        except Exception as exc:
            self.logger.warn(f"Failed to query Orbbec devices: {exc}")
            return False

    def _select_video_profile(self, profiles, width, height, fmt, fps, label):
        if width > 0 and height > 0:
            try:
                profile = profiles.get_video_stream_profile(width, height, fmt, fps)
                self.logger.info(
                    f"Selected Orbbec {label} stream: "
                    f"{profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} fps"
                )
                return profile
            except Exception as exc:
                self.logger.warn(
                    f"Requested Orbbec {label} stream "
                    f"{width}x{height}@{fps or 'any'} is unavailable: {exc}"
                )

            try:
                profile = profiles.get_video_stream_profile(width, height, fmt, 0)
                self.logger.info(
                    f"Selected Orbbec {label} stream with fallback fps: "
                    f"{profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} fps"
                )
                return profile
            except Exception as exc:
                self.logger.warn(
                    f"Requested Orbbec {label} resolution {width}x{height} "
                    f"is unavailable: {exc}"
                )

        try:
            profile = profiles.get_video_stream_profile(0, 0, fmt, fps)
            self.logger.info(
                f"Selected Orbbec {label} stream by format/default size: "
                f"{profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} fps"
            )
            return profile
        except Exception as exc:
            self.logger.warn(
                f"Could not select Orbbec {label} stream by format; using device default: {exc}"
            )
            return profiles.get_default_video_stream_profile()

    def _intrinsic_value(self, name, fallback):
        with self.lock:
            intrinsic = self.camera_intrinsic
        if intrinsic is None:
            return fallback
        return float(getattr(intrinsic, name, fallback))

    @staticmethod
    def _color_frame_to_bgr(color_frame):
        width = color_frame.get_width()
        height = color_frame.get_height()
        fmt = color_frame.get_format()
        data = np.asanyarray(color_frame.get_data())

        if fmt == OBFormat.RGB:
            image = data.reshape((height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if fmt == OBFormat.BGR:
            return data.reshape((height, width, 3))
        if fmt == OBFormat.YUYV:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
        if hasattr(OBFormat, "YUY2") and fmt == OBFormat.YUY2:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
        if hasattr(OBFormat, "UYVY") and fmt == OBFormat.UYVY:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
        if hasattr(OBFormat, "NV12") and fmt == OBFormat.NV12:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
        if hasattr(OBFormat, "NV21") and fmt == OBFormat.NV21:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV21)
        if hasattr(OBFormat, "I420") and fmt == OBFormat.I420:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
        if fmt == OBFormat.MJPG:
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is not None:
                return image

        raise RuntimeError(f"Unsupported color format: {fmt}")

    @staticmethod
    def _depth_frame_to_meters(depth_frame):
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        depth_scale = depth_frame.get_depth_scale()
        depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
            (height, width)
        )
        depth_mm = depth_raw.astype(np.float32) * depth_scale
        return depth_mm * 0.001


@dataclass
class DepthPersonCandidate:
    bbox: tuple[int, int, int, int]
    area: int
    median_depth: float
    nearest_depth: float
    centroid: tuple[float, float]


class NearestPersonDepthSelector:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.min_depth_m = float(node.declare_parameter("person_min_depth_m", 0.35).value)
        self.max_depth_m = float(node.declare_parameter("person_max_depth_m", 4.0).value)
        self.depth_band_m = float(node.declare_parameter("person_depth_band_m", 1.2).value)
        self.nearest_percentile = float(
            node.declare_parameter("person_nearest_percentile", 1.0).value
        )
        self.nearest_percentile = max(0.0, min(25.0, self.nearest_percentile))
        self.min_area_px = int(node.declare_parameter("person_min_area_px", 2500).value)
        self.roi_margin_px = int(node.declare_parameter("person_roi_margin_px", 90).value)
        self.search_expand_px = int(
            node.declare_parameter("person_search_expand_px", 140).value
        )
        self.mask_background = bool(
            node.declare_parameter("person_mask_background", True).value
        )

    def find_nearest(self, depth_meters, search_bbox=None):
        height, width = depth_meters.shape[:2]
        x0, y0, x1, y1 = self._expanded_bbox(
            search_bbox or (0, 0, width, height),
            self.search_expand_px if search_bbox else 0,
            width,
            height,
        )
        depth_roi = depth_meters[y0:y1, x0:x1]
        valid = self._valid_depth_mask(depth_roi)
        if int(np.count_nonzero(valid)) < self.min_area_px:
            return None

        nearest_depth = float(
            np.percentile(depth_roi[valid], self.nearest_percentile)
        )
        depth_limit = min(self.max_depth_m, nearest_depth + self.depth_band_m)
        mask = valid & (depth_roi <= depth_limit)
        mask = self._clean_mask(mask)

        component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8),
            connectivity=8,
        )
        candidates = []
        for label in range(1, component_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area_px:
                continue

            component_mask = labels == label
            component_depth = depth_roi[component_mask]
            if component_depth.size == 0:
                continue

            left = int(stats[label, cv2.CC_STAT_LEFT]) + x0
            top = int(stats[label, cv2.CC_STAT_TOP]) + y0
            comp_width = int(stats[label, cv2.CC_STAT_WIDTH])
            comp_height = int(stats[label, cv2.CC_STAT_HEIGHT])
            bbox = (left, top, left + comp_width, top + comp_height)
            centroid = (float(centroids[label][0] + x0), float(centroids[label][1] + y0))
            candidates.append(
                DepthPersonCandidate(
                    bbox=bbox,
                    area=area,
                    median_depth=float(np.median(component_depth)),
                    nearest_depth=float(
                        np.percentile(component_depth, self.nearest_percentile)
                    ),
                    centroid=centroid,
                )
            )

        if not candidates:
            return None
        if search_bbox:
            return min(candidates, key=lambda item: self._tracking_score(item, search_bbox))
        return min(candidates, key=lambda item: (item.median_depth, -item.area))

    def isolate_color_image(self, color_image, depth_meters, candidate):
        height, width = depth_meters.shape[:2]
        roi = self._expanded_bbox(candidate.bbox, self.roi_margin_px, width, height)
        x0, y0, x1, y1 = roi
        color_roi = color_image[y0:y1, x0:x1].copy()
        if not self.mask_background:
            return color_roi, roi

        depth_roi = depth_meters[y0:y1, x0:x1]
        valid = self._valid_depth_mask(depth_roi)
        depth_limit = min(self.max_depth_m, candidate.nearest_depth + self.depth_band_m)
        mask = valid & (depth_roi <= depth_limit)
        mask = self._clean_mask(mask)
        color_roi[~mask] = 0
        return color_roi, roi

    def describe(self, candidate):
        if candidate is None:
            return "no candidate"
        x0, y0, x1, y1 = candidate.bbox
        return (
            f"bbox=({x0},{y0},{x1},{y1}), area={candidate.area}, "
            f"depth={candidate.median_depth:.2f}m"
        )

    def _valid_depth_mask(self, depth):
        return (
            np.isfinite(depth)
            & (depth >= self.min_depth_m)
            & (depth <= self.max_depth_m)
        )

    @staticmethod
    def _clean_mask(mask):
        mask_u8 = mask.astype(np.uint8) * 255
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask_u8 > 0

    @staticmethod
    def _expanded_bbox(bbox, margin, width, height):
        x0, y0, x1, y1 = [int(round(value)) for value in bbox]
        return (
            max(0, x0 - margin),
            max(0, y0 - margin),
            min(width, x1 + margin),
            min(height, y1 + margin),
        )

    @staticmethod
    def _tracking_score(candidate, previous_bbox):
        px0, py0, px1, py1 = previous_bbox
        pcx = 0.5 * (px0 + px1)
        pcy = 0.5 * (py0 + py1)
        cx, cy = candidate.centroid
        center_distance = math.hypot(cx - pcx, cy - pcy)
        return center_distance - 0.01 * candidate.area + 100.0 * candidate.median_depth


class MediaPipeLandmarkDetector:
    POSE_LANDMARKS = {
        "left_shoulder": 11,
        "left_elbow": 13,
        "left_wrist": 15,
        "right_shoulder": 12,
        "right_elbow": 14,
        "right_wrist": 16,
    }

    def __init__(
        self,
        hands_model_complexity=0,
        pose_enabled=False,
        pose_backend="auto",
        pose_model_complexity=0,
        pose_model_path="",
        pose_task_model_path="",
        detection_confidence=0.65,
        tracking_confidence=0.65,
        logger=None,
    ):
        self.logger = logger
        self.mp_hands = mp.solutions.hands
        self.mp_pose = mp.solutions.pose
        self.pose_enabled = bool(pose_enabled)
        self.pose_backend = "disabled"
        self.pose_timestamp_ms = 0
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=int(hands_model_complexity),
            min_detection_confidence=float(detection_confidence),
            min_tracking_confidence=float(tracking_confidence),
        )
        self.pose = None
        if self.pose_enabled:
            pose_backend = str(pose_backend).strip().lower()
            pose_task_model_path = str(pose_task_model_path).strip()
            if pose_backend == "auto":
                pose_backend = "tasks" if pose_task_model_path else "solutions"

            if pose_backend == "tasks":
                self._init_pose_landmarker_task(
                    pose_task_model_path,
                    detection_confidence,
                    tracking_confidence,
                )
            elif pose_backend == "solutions":
                self._init_pose_solution(
                    pose_model_path,
                    pose_model_complexity,
                    detection_confidence,
                    tracking_confidence,
                )
            else:
                raise ValueError(
                    "mp_pose_backend must be 'auto', 'tasks', or 'solutions'"
                )

    def _init_pose_solution(
        self,
        pose_model_path,
        pose_model_complexity,
        detection_confidence,
        tracking_confidence,
    ):
        pose_model_path = str(pose_model_path).strip()
        if pose_model_path:
            install_local_mediapipe_pose_model(
                pose_model_path,
                pose_model_complexity,
                logger=self.logger,
            )
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=int(pose_model_complexity),
            min_detection_confidence=float(detection_confidence),
            min_tracking_confidence=float(tracking_confidence),
        )
        self.pose_backend = "solutions"

    def _init_pose_landmarker_task(
        self,
        pose_task_model_path,
        detection_confidence,
        tracking_confidence,
    ):
        if mp_tasks_python is None or mp_tasks_vision is None:
            raise RuntimeError(
                "MediaPipe Tasks API is unavailable in this Python environment. "
                "Install a newer mediapipe package or use mp_pose_backend:=solutions."
            )
        if not pose_task_model_path:
            raise ValueError(
                "mp_pose_task_model_path must be set when mp_pose_backend:=tasks"
            )
        model_path = Path(pose_task_model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"mp_pose_task_model_path does not exist: {model_path}"
            )

        base_options = mp_tasks_python.BaseOptions(
            model_asset_path=str(model_path),
        )
        options = mp_tasks_vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_tasks_vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=float(detection_confidence),
            min_pose_presence_confidence=float(detection_confidence),
            min_tracking_confidence=float(tracking_confidence),
            output_segmentation_masks=False,
        )
        self.pose = mp_tasks_vision.PoseLandmarker.create_from_options(options)
        self.pose_backend = "tasks"
        if self.logger is not None:
            self.logger.info(f"Using MediaPipe Tasks PoseLandmarker: {model_path}")

    def collect(self, image_rgb, offset=(0, 0)):
        height, width = image_rgb.shape[:2]
        points_2d = {}
        offset_x, offset_y = offset

        if self.pose is not None and self.pose_backend == "solutions":
            pose_results = self.pose.process(image_rgb)
            if pose_results.pose_landmarks:
                landmarks = pose_results.pose_landmarks.landmark
                for name, idx in self.POSE_LANDMARKS.items():
                    landmark = landmarks[idx]
                    if landmark.visibility >= 0.5:
                        points_2d[f"pose_{name}"] = self._landmark_to_pixel(
                            landmark, width, height, offset_x, offset_y
                        )
        elif self.pose is not None and self.pose_backend == "tasks":
            image_rgb = np.ascontiguousarray(image_rgb)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            self.pose_timestamp_ms += 33
            pose_results = self.pose.detect_for_video(
                mp_image,
                self.pose_timestamp_ms,
            )
            if pose_results.pose_landmarks:
                landmarks = pose_results.pose_landmarks[0]
                for name, idx in self.POSE_LANDMARKS.items():
                    landmark = landmarks[idx]
                    visibility = getattr(landmark, "visibility", 1.0)
                    if visibility >= 0.5:
                        points_2d[f"pose_{name}"] = self._landmark_to_pixel(
                            landmark, width, height, offset_x, offset_y
                        )

        hands_results = self.hands.process(image_rgb)
        if hands_results.multi_hand_landmarks:
            for hand_idx, hand_landmarks in enumerate(hands_results.multi_hand_landmarks):
                prefix = f"hand_{hand_idx}"
                if hands_results.multi_handedness:
                    prefix = hands_results.multi_handedness[hand_idx].classification[
                        0
                    ].label.lower()

                for landmark_idx, landmark in enumerate(hand_landmarks.landmark):
                    points_2d[f"{prefix}_{landmark_idx}"] = self._landmark_to_pixel(
                        landmark, width, height, offset_x, offset_y
                    )

        return points_2d

    def close(self):
        if self.hands is not None:
            self.hands.close()
            self.hands = None
        if self.pose is not None:
            self.pose.close()
            self.pose = None

    @staticmethod
    def _landmark_to_pixel(landmark, width, height, offset_x=0, offset_y=0):
        x = int(round(landmark.x * width))
        y = int(round(landmark.y * height))
        x = max(0, min(width - 1, x))
        y = max(0, min(height - 1, y))
        return x + offset_x, y + offset_y


class HandKinematics:
    FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
    PALM_DIRECTION_NAMES = ("up", "down", "left", "right", "forward", "backward")
    PALM_LANDMARKS = (0, 5, 9, 13, 17)
    FINGER_JOINTS = {
        "thumb": ((1, 2, 3), (2, 3, 4)),
        "index": ((0, 5, 6), (5, 6, 7), (6, 7, 8)),
        "middle": ((0, 9, 10), (9, 10, 11), (10, 11, 12)),
        "ring": ((0, 13, 14), (13, 14, 15), (14, 15, 16)),
        "pinky": ((0, 17, 18), (17, 18, 19), (18, 19, 20)),
    }
    FINGER_MAX_FLEXION_DEG = {
        "thumb": 150.0,
        "index": 260.0,
        "middle": 260.0,
        "ring": 260.0,
        "pinky": 260.0,
    }

    def __init__(self):
        self.palm_reference_point = None

    def reset(self):
        self.palm_reference_point = None

    def best_hand_prefix(self, points):
        counts = {}
        for key in points:
            prefix, separator, suffix = key.rpartition("_")
            if not separator or prefix == "pose":
                continue
            try:
                landmark_index = int(suffix)
            except ValueError:
                continue
            if 0 <= landmark_index <= 20:
                counts[prefix] = counts.get(prefix, 0) + 1

        if not counts:
            return None
        return max(counts, key=counts.get)

    def finger_bends(self, points_3d, points_2d):
        prefix = self.best_hand_prefix(points_2d)
        if prefix is None:
            return None, None, None

        source = "2D"
        points = self.points_2d_to_arrays(points_2d)
        if self.hand_points_complete(points_3d, prefix):
            source = "3D"
            points = points_3d

        bends, flexion_degrees = self._finger_bends_from_points(points, prefix)
        if bends is None:
            return None, None, None
        return bends, flexion_degrees, source

    def palm_direction_distances(self, points_3d, points_2d):
        prefix = self.best_hand_prefix(points_2d)
        if prefix is None:
            return None, None

        palm_points = [
            points_3d[f"{prefix}_{idx}"]
            for idx in self.PALM_LANDMARKS
            if f"{prefix}_{idx}" in points_3d
        ]
        if len(palm_points) < 3:
            return None, None

        palm_point = np.mean(palm_points, axis=0)
        if self.palm_reference_point is None:
            self.palm_reference_point = palm_point.copy()

        displacement = palm_point - self.palm_reference_point
        dx, dy, dz = [float(value) for value in displacement]
        distances = {
            "up": max(-dy, 0.0),
            "down": max(dy, 0.0),
            "left": max(-dx, 0.0),
            "right": max(dx, 0.0),
            "forward": max(-dz, 0.0),
            "backward": max(dz, 0.0),
        }
        return distances, displacement

    def arm_angles(self, points):
        angles = {}
        for side in ("left", "right"):
            shoulder = points.get(f"pose_{side}_shoulder")
            elbow = points.get(f"pose_{side}_elbow")
            wrist = points.get(f"pose_{side}_wrist")
            if shoulder is None or elbow is None or wrist is None:
                continue

            angle = self.joint_angle(shoulder, elbow, wrist)
            if angle is not None:
                angles[side] = angle
        return angles

    def hand_points_complete(self, points, prefix):
        return self.hand_required_keys(prefix).issubset(points)

    def hand_required_keys(self, prefix):
        required_indexes = set()
        for joints in self.FINGER_JOINTS.values():
            for joint in joints:
                required_indexes.update(joint)
        return {f"{prefix}_{idx}" for idx in required_indexes}

    @staticmethod
    def points_2d_to_arrays(points_2d):
        return {
            key: np.array([value[0], value[1]], dtype=np.float32)
            for key, value in points_2d.items()
        }

    @staticmethod
    def joint_angle(first, middle, last):
        vector_a = first - middle
        vector_b = last - middle
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        if norm_a < 1e-6 or norm_b < 1e-6:
            return None

        cos_angle = float(np.dot(vector_a, vector_b) / (norm_a * norm_b))
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.degrees(math.acos(cos_angle))

    def _finger_bends_from_points(self, points, prefix):
        bends = {}
        flexion_degrees = {}

        for finger_name in self.FINGER_NAMES:
            total_flexion = 0.0
            for first_idx, middle_idx, last_idx in self.FINGER_JOINTS[finger_name]:
                first = points.get(f"{prefix}_{first_idx}")
                middle = points.get(f"{prefix}_{middle_idx}")
                last = points.get(f"{prefix}_{last_idx}")
                if first is None or middle is None or last is None:
                    return None, None

                angle = self.joint_angle(first, middle, last)
                if angle is None:
                    return None, None

                total_flexion += max(0.0, 180.0 - angle)

            max_flexion = self.FINGER_MAX_FLEXION_DEG[finger_name]
            bends[finger_name] = max(0.0, min(1.0, total_flexion / max_flexion))
            flexion_degrees[finger_name] = total_flexion

        return bends, flexion_degrees


class NeuralHandKinematics(HandKinematics):
    """Fast landmark-to-control adapter with optional ONNX bend regression.

    The image-side neural network is MediaPipe's landmark stack. This adapter
    keeps the same public outputs as HandKinematics so the rest of the robot
    control pipeline, GUI tuning, ROS topics, and keyboard controls stay intact.
    """

    FINGER_TRIPLETS = (
        (1, 2, 3),
        (2, 3, 4),
        (0, 5, 6),
        (5, 6, 7),
        (6, 7, 8),
        (0, 9, 10),
        (9, 10, 11),
        (10, 11, 12),
        (0, 13, 14),
        (13, 14, 15),
        (14, 15, 16),
        (0, 17, 18),
        (17, 18, 19),
        (18, 19, 20),
    )
    FINGER_SLICES = {
        "thumb": slice(0, 2),
        "index": slice(2, 5),
        "middle": slice(5, 8),
        "ring": slice(8, 11),
        "pinky": slice(11, 14),
    }

    def __init__(self, node):
        super().__init__()
        self.logger = node.get_logger()
        self.enabled = bool(
            node.declare_parameter("nn_kinematics_enabled", True).value
        )
        self.hand_bend_model_path = str(
            node.declare_parameter("nn_hand_bend_model_path", "").value
        ).strip()
        self.hand_bend_blend = float(
            node.declare_parameter("nn_hand_bend_blend", 0.85).value
        )
        self.hand_bend_blend = max(0.0, min(1.0, self.hand_bend_blend))
        self.depth_bend_reject_enabled = bool(
            node.declare_parameter("nn_depth_bend_reject_enabled", True).value
        )
        self.depth_bend_reject_delta = float(
            node.declare_parameter("nn_depth_bend_reject_delta", 0.35).value
        )
        self.depth_bend_reject_2d_max = float(
            node.declare_parameter("nn_depth_bend_reject_2d_max", 0.35).value
        )
        self.depth_bend_reject_min_count = int(
            node.declare_parameter("nn_depth_bend_reject_min_count", 2).value
        )
        self.depth_bend_reject_delta = max(0.0, min(1.0, self.depth_bend_reject_delta))
        self.depth_bend_reject_2d_max = max(0.0, min(1.0, self.depth_bend_reject_2d_max))
        self.depth_bend_reject_min_count = max(1, self.depth_bend_reject_min_count)
        self.onnx_net = None
        self.onnx_input_name = ""

        if self.hand_bend_model_path:
            try:
                self.onnx_net = cv2.dnn.readNetFromONNX(self.hand_bend_model_path)
                self.logger.info(
                    f"Loaded neural hand bend ONNX model: {self.hand_bend_model_path}"
                )
            except Exception as exc:
                self.onnx_net = None
                self.logger.warn(
                    f"Failed to load nn_hand_bend_model_path={self.hand_bend_model_path}: "
                    f"{exc}; using vectorized fallback"
                )

    def finger_bends(self, points_3d, points_2d):
        if not self.enabled:
            return super().finger_bends(points_3d, points_2d)

        prefix = self.best_hand_prefix(points_2d)
        if prefix is None:
            return None, None, None

        points_2d_arrays = self.points_2d_to_arrays(points_2d)
        bends_2d, flexion_2d = self._finger_bends_vectorized(points_2d_arrays, prefix)
        if bends_2d is None:
            return None, None, None

        source = "NN-vectorized-2D"
        geometry_source = "2D"
        points = points_2d_arrays
        bends = bends_2d
        flexion_degrees = flexion_2d
        if self.hand_points_complete(points_3d, prefix):
            bends_3d, flexion_3d = self._finger_bends_vectorized(points_3d, prefix)
            if bends_3d is not None:
                if self._depth_bend_disagrees_with_2d(bends_3d, bends_2d):
                    source = "NN-vectorized-2D-depth-rejected"
                else:
                    source = "NN-vectorized-3D"
                    geometry_source = "3D"
                    points = points_3d
                    bends = bends_3d
                    flexion_degrees = flexion_3d

        if self.onnx_net is not None:
            predicted = self._predict_bends_onnx(points, prefix)
            if predicted is not None:
                depth_rejected = "depth-rejected" in source
                for name in self.FINGER_NAMES:
                    bends[name] = (
                        self.hand_bend_blend * predicted[name]
                        + (1.0 - self.hand_bend_blend) * bends[name]
                    )
                    flexion_degrees[name] = (
                        bends[name] * self.FINGER_MAX_FLEXION_DEG[name]
                    )
                source = f"NN-ONNX+geometry-{geometry_source}"
                if depth_rejected:
                    source += "-depth-rejected"

        return bends, flexion_degrees, source

    def _depth_bend_disagrees_with_2d(self, bends_3d, bends_2d):
        if not self.depth_bend_reject_enabled:
            return False

        disagreement_count = 0
        for name in ("index", "middle", "ring", "pinky"):
            if bends_2d.get(name, 0.0) > self.depth_bend_reject_2d_max:
                continue
            if bends_3d.get(name, 0.0) - bends_2d.get(name, 0.0) >= self.depth_bend_reject_delta:
                disagreement_count += 1
        return disagreement_count >= self.depth_bend_reject_min_count

    def arm_angles(self, points):
        if not self.enabled:
            return super().arm_angles(points)
        angles = {}
        for side in ("left", "right"):
            required = [
                points.get(f"pose_{side}_shoulder"),
                points.get(f"pose_{side}_elbow"),
                points.get(f"pose_{side}_wrist"),
            ]
            if any(value is None for value in required):
                continue
            angle = self._angles_from_triplet_arrays(
                np.asarray([required[0]], dtype=np.float32),
                np.asarray([required[1]], dtype=np.float32),
                np.asarray([required[2]], dtype=np.float32),
            )
            if angle.size:
                angles[side] = float(angle[0])
        return angles

    def _finger_bends_vectorized(self, points, prefix):
        hand_points = self._hand_array(points, prefix)
        if hand_points is None:
            return None, None

        triples = np.asarray(self.FINGER_TRIPLETS, dtype=np.int32)
        angles = self._angles_from_triplet_arrays(
            hand_points[triples[:, 0]],
            hand_points[triples[:, 1]],
            hand_points[triples[:, 2]],
        )
        if angles.size != len(self.FINGER_TRIPLETS):
            return None, None

        flexions = np.maximum(0.0, 180.0 - angles)
        bends = {}
        flexion_degrees = {}
        for name in self.FINGER_NAMES:
            total_flexion = float(np.sum(flexions[self.FINGER_SLICES[name]]))
            max_flexion = self.FINGER_MAX_FLEXION_DEG[name]
            bends[name] = max(0.0, min(1.0, total_flexion / max_flexion))
            flexion_degrees[name] = total_flexion
        return bends, flexion_degrees

    def _predict_bends_onnx(self, points, prefix):
        features = self._normalized_hand_features(points, prefix)
        if features is None:
            return None
        try:
            self.onnx_net.setInput(features)
            output = self.onnx_net.forward()
        except Exception as exc:
            self.logger.warn(f"Neural hand bend ONNX inference failed: {exc}")
            return None

        values = np.asarray(output, dtype=np.float32).reshape(-1)
        if values.size < len(self.FINGER_NAMES):
            self.logger.warn(
                f"Neural hand bend ONNX output too short: expected 5, got {values.size}"
            )
            return None
        return {
            name: max(0.0, min(1.0, float(values[index])))
            for index, name in enumerate(self.FINGER_NAMES)
        }

    def _normalized_hand_features(self, points, prefix):
        hand_points = self._hand_array(points, prefix)
        if hand_points is None:
            return None
        if hand_points.shape[1] == 2:
            hand_points = np.pad(hand_points, ((0, 0), (0, 1)), mode="constant")

        origin = hand_points[0].copy()
        palm_width = np.linalg.norm(hand_points[5] - hand_points[17])
        palm_length = np.linalg.norm(hand_points[0] - hand_points[9])
        scale = max(float(palm_width), float(palm_length), 1e-6)
        features = ((hand_points - origin) / scale).astype(np.float32)
        return features.reshape(1, -1)

    @staticmethod
    def _angles_from_triplet_arrays(first, middle, last):
        vector_a = first - middle
        vector_b = last - middle
        norm_a = np.linalg.norm(vector_a, axis=1)
        norm_b = np.linalg.norm(vector_b, axis=1)
        valid = (norm_a >= 1e-6) & (norm_b >= 1e-6)
        if not np.all(valid):
            return np.asarray([], dtype=np.float32)
        cos_angles = np.sum(vector_a * vector_b, axis=1) / (norm_a * norm_b)
        cos_angles = np.clip(cos_angles, -1.0, 1.0)
        return np.degrees(np.arccos(cos_angles)).astype(np.float32)

    @staticmethod
    def _hand_array(points, prefix):
        values = []
        for index in range(21):
            point = points.get(f"{prefix}_{index}")
            if point is None:
                return None
            values.append(point)
        return np.asarray(values, dtype=np.float32)


class CvDisplay:
    TUNING_TRACKBARS = (
        ("arm_mit_control_enabled", "MIT_on", 1, 1.0),
        ("arm_mit_kp", "MIT_kp", 200, 1.0),
        ("arm_mit_kd", "MIT_kd_x10", 500, 0.1),
        ("arm_mit_max_velocity_m_s", "MIT_vmax_x100", 100, 0.01),
        ("arm_mit_max_accel_m_s2", "MIT_amax_x100", 500, 0.01),
        ("max_arm_target_delta_m", "Max_d_mm", 50, 0.001),
        ("arm_command_interval_s", "Cmd_ms", 500, 0.001),
        ("hand_deadband_m", "Palm_db_mm", 50, 0.001),
        ("arm_axis_deadband_m", "Axis_db_mm", 30, 0.001),
        ("palm_filter_alpha", "Palm_lp_pct", 100, 0.01),
        ("palm_filter_hold_deadband_m", "Palm_hold_mm", 50, 0.001),
        ("palm_filter_release_deadband_m", "Palm_rel_mm", 80, 0.001),
        ("hand_bend_deadzone", "Bend_db_pct", 50, 0.01),
        ("hand_bend_smoothing", "Bend_alpha_pct", 100, 0.01),
        ("hand_bend_max_delta", "Bend_dmax_pct", 100, 0.01),
        ("hand_send_hz", "Hand_hz", 60, 1.0),
        ("hand_smoothing", "Hand_alpha_pct", 100, 0.01),
        ("hand_deadband", "Hand_db", 80, 1.0),
    )
    TUNING_WINDOW_BY_KEY = {
        "arm_mit_control_enabled": "Arm Tune",
        "arm_mit_kp": "Arm Tune",
        "arm_mit_kd": "Arm Tune",
        "arm_mit_max_velocity_m_s": "Arm Tune",
        "arm_mit_max_accel_m_s2": "Arm Tune",
        "max_arm_target_delta_m": "Arm Tune",
        "arm_command_interval_s": "Arm Tune",
        "hand_deadband_m": "Palm Tune",
        "arm_axis_deadband_m": "Palm Tune",
        "palm_filter_alpha": "Palm Tune",
        "palm_filter_hold_deadband_m": "Palm Tune",
        "palm_filter_release_deadband_m": "Palm Tune",
        "hand_bend_deadzone": "Hand Tune",
        "hand_bend_smoothing": "Hand Tune",
        "hand_bend_max_delta": "Hand Tune",
        "hand_send_hz": "Hand Tune",
        "hand_smoothing": "Hand Tune",
        "hand_deadband": "Hand Tune",
    }
    TUNING_WINDOW_SIZES = {
        "Arm Tune": (380, 220),
        "Palm Tune": (380, 170),
        "Hand Tune": (380, 190),
    }

    def __init__(self, window_name, max_width, max_height, show_window=True):
        self.window_name = window_name
        self.max_width = max(320, int(max_width))
        self.max_height = max(240, int(max_height))
        self.show_window = show_window
        self.window_initialized = False
        self.tuning_initialized = False
        self.tuning_enabled = False
        self.tuning_values = {}
        self.tuning_windows = {}

    def show(
        self,
        color_image,
        points_2d,
        finger_bends=None,
        finger_source=None,
        palm_distances=None,
        arm_offset=None,
        hand_positions=None,
        status_text=None,
        person_roi=None,
    ):
        if person_roi is not None:
            x0, y0, x1, y1 = person_roi
            cv2.rectangle(color_image, (x0, y0), (x1, y1), (0, 220, 255), 2)
        for x, y in points_2d.values():
            cv2.circle(color_image, (x, y), 5, (0, 255, 0), -1)
        if status_text:
            self._draw_text(color_image, status_text, (12, 28))
        self._draw_finger_bend_overlay(color_image, finger_bends, finger_source)
        self._draw_palm_motion_overlay(color_image, palm_distances)
        self._draw_arm_offset_overlay(color_image, arm_offset)
        self._draw_hand_position_overlay(color_image, hand_positions)

        if not self.show_window:
            return False

        display_image = self._resize_for_display(color_image)
        self._ensure_window(display_image)
        cv2.imshow(self.window_name, display_image)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return "quit"
        if key in (ord("h"), ord("H")):
            return "home"
        if key in (ord("s"), ord("S")):
            return "start"
        if key in (ord("p"), ord("P")):
            return "save_tuning"
        return None

    @staticmethod
    def close():
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    def _resize_for_display(self, image):
        height, width = image.shape[:2]
        scale = min(
            self.max_width / float(width),
            self.max_height / float(height),
            1.0,
        )
        if scale >= 1.0:
            return image

        display_width = max(1, int(round(width * scale)))
        display_height = max(1, int(round(height * scale)))
        return cv2.resize(
            image,
            (display_width, display_height),
            interpolation=cv2.INTER_AREA,
        )

    def _ensure_window(self, display_image):
        if self.window_initialized:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, display_image.shape[1], display_image.shape[0])
        try:
            cv2.moveWindow(self.window_name, 20, 20)
        except cv2.error:
            pass
        self.window_initialized = True
        self._ensure_tuning_trackbars()

    def configure_tuning(self, values, enabled=True):
        self.tuning_values = dict(values)
        self.tuning_enabled = bool(enabled)
        if self.window_initialized:
            self._ensure_tuning_trackbars()
            self._sync_tuning_trackbars()

    def set_tuning_enabled(self, enabled):
        self.tuning_enabled = bool(enabled)
        if self.window_initialized and not self.tuning_enabled:
            self._sync_tuning_trackbars()

    def get_tuning_values(self):
        if self.show_window and self.window_initialized and self.tuning_enabled:
            self._read_tuning_trackbars()
        return dict(self.tuning_values)

    def _ensure_tuning_trackbars(self):
        if self.tuning_initialized or not self.show_window:
            return
        if not self.tuning_values:
            return
        self._ensure_tuning_windows()
        for key, label, maximum, _scale in self.TUNING_TRACKBARS:
            value = self._trackbar_value_from_tuning(key)
            cv2.createTrackbar(
                label,
                self._tuning_window_for_key(key),
                value,
                maximum,
                lambda _value: None,
            )
        self.tuning_initialized = True

    def _sync_tuning_trackbars(self):
        if not self.tuning_initialized:
            return
        for key, label, _maximum, _scale in self.TUNING_TRACKBARS:
            cv2.setTrackbarPos(
                label,
                self._tuning_window_for_key(key),
                self._trackbar_value_from_tuning(key),
            )

    def _read_tuning_trackbars(self):
        if not self.tuning_initialized:
            return
        values = {}
        for key, label, _maximum, scale in self.TUNING_TRACKBARS:
            raw_value = cv2.getTrackbarPos(label, self._tuning_window_for_key(key))
            if key == "arm_mit_control_enabled":
                values[key] = bool(raw_value)
            else:
                values[key] = float(raw_value) * scale
        values["arm_mit_kp"] = max(1.0, values["arm_mit_kp"])
        values["arm_mit_kd"] = max(0.0, values["arm_mit_kd"])
        values["arm_mit_max_velocity_m_s"] = max(0.01, values["arm_mit_max_velocity_m_s"])
        values["arm_mit_max_accel_m_s2"] = max(0.01, values["arm_mit_max_accel_m_s2"])
        values["max_arm_target_delta_m"] = max(0.001, values["max_arm_target_delta_m"])
        values["arm_command_interval_s"] = max(0.03, values["arm_command_interval_s"])
        values["hand_deadband_m"] = max(0.0, values["hand_deadband_m"])
        axis_deadband = max(0.0, values["arm_axis_deadband_m"])
        values["arm_axis_deadband_m"] = [axis_deadband, axis_deadband, axis_deadband]
        values["palm_filter_alpha"] = max(0.0, min(1.0, values["palm_filter_alpha"]))
        values["palm_filter_hold_deadband_m"] = max(
            0.0,
            values["palm_filter_hold_deadband_m"],
        )
        values["palm_filter_release_deadband_m"] = max(
            values["palm_filter_hold_deadband_m"],
            values["palm_filter_release_deadband_m"],
        )
        values["hand_bend_deadzone"] = max(0.0, min(1.0, values["hand_bend_deadzone"]))
        values["hand_bend_smoothing"] = max(0.0, min(1.0, values["hand_bend_smoothing"]))
        values["hand_bend_max_delta"] = max(0.0, min(1.0, values["hand_bend_max_delta"]))
        values["hand_send_hz"] = max(1.0, values["hand_send_hz"])
        values["hand_smoothing"] = max(0.0, min(1.0, values["hand_smoothing"]))
        values["hand_deadband"] = max(0, int(round(values["hand_deadband"])))
        self.tuning_values.update(values)

    def _trackbar_value_from_tuning(self, key):
        value = self.tuning_values.get(key, 0)
        if key == "arm_mit_control_enabled":
            return 1 if bool(value) else 0
        maximum, scale = next(
            (trackbar_maximum, trackbar_scale)
            for trackbar_key, _label, trackbar_maximum, trackbar_scale in self.TUNING_TRACKBARS
            if trackbar_key == key
        )
        if key == "arm_axis_deadband_m" and isinstance(value, (list, tuple, np.ndarray)):
            value = max(float(item) for item in value[:3])
        return max(0, min(maximum, int(round(float(value) / scale))))

    def _ensure_tuning_windows(self):
        if self.tuning_windows:
            return
        for index, group_name in enumerate(self.TUNING_WINDOW_SIZES):
            window_name = f"{self.window_name} - {group_name}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            width, height = self.TUNING_WINDOW_SIZES[group_name]
            cv2.resizeWindow(window_name, width, height)
            try:
                cv2.moveWindow(window_name, self.max_width + 60, 20 + index * (height + 35))
            except cv2.error:
                pass
            self.tuning_windows[group_name] = window_name

    def _tuning_window_for_key(self, key):
        group_name = self.TUNING_WINDOW_BY_KEY.get(key, "Hand Tune")
        return self.tuning_windows.get(
            group_name,
            f"{self.window_name} - {group_name}",
        )

    @staticmethod
    def _draw_finger_bend_overlay(image, finger_bends, source):
        if not finger_bends:
            return

        label = " | ".join(
            f"{name[:1].upper()}:{finger_bends[name]:.2f}"
            for name in HandKinematics.FINGER_NAMES
        )
        if source:
            label = f"Finger bend {source}: {label}"
        CvDisplay._draw_text(image, label, (12, 58))

    @staticmethod
    def _draw_palm_motion_overlay(image, palm_distances):
        if not palm_distances:
            return

        label = (
            f"Palm m U:{palm_distances['up']:.2f} D:{palm_distances['down']:.2f} "
            f"L:{palm_distances['left']:.2f} R:{palm_distances['right']:.2f} "
            f"F:{palm_distances['forward']:.2f} B:{palm_distances['backward']:.2f}"
        )
        CvDisplay._draw_text(image, label, (12, 88))

    @staticmethod
    def _draw_arm_offset_overlay(image, arm_offset):
        if arm_offset is None:
            return

        label = (
            f"RM65 target offset m X:{arm_offset[0]:.3f} "
            f"Y:{arm_offset[1]:.3f} Z:{arm_offset[2]:.3f}"
        )
        CvDisplay._draw_text(image, label, (12, 118))

    @staticmethod
    def _draw_hand_position_overlay(image, hand_positions):
        if hand_positions is None:
            return

        labels = ("P", "R", "M", "I", "TB", "TR")
        value_text = " ".join(
            f"{label}:{value}" for label, value in zip(labels, hand_positions)
        )
        CvDisplay._draw_text(image, f"RM hand pos {value_text}", (12, 148))

    @staticmethod
    def _draw_text(image, label, origin):
        cv2.putText(
            image,
            label,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            label,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


class RM65PalmFollower:
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        self.enabled = bool(node.declare_parameter("robot_motion_enabled", False).value)
        self.robot_ip = node.declare_parameter("robot_ip", "192.168.1.18").value
        self.robot_port = int(node.declare_parameter("robot_port", 8080).value)
        self.robot_level = int(node.declare_parameter("robot_level", 3).value)
        self.thread_mode = int(node.declare_parameter("robot_thread_mode", 2).value)
        self.arm_speed = int(node.declare_parameter("arm_speed", 10).value)
        self.arm_blend_radius = int(node.declare_parameter("arm_blend_radius", 0).value)
        self.arm_block = int(node.declare_parameter("arm_block", 0).value)
        self.movel_redirect_to_canfd = bool(
            node.declare_parameter("arm_movel_redirect_to_canfd", False).value
        )
        self.slow_stop_on_pause = bool(
            node.declare_parameter("arm_slow_stop_on_pause", True).value
        )
        self.control_protocol = str(
            node.declare_parameter("arm_control_protocol", "rs485").value
        ).strip().lower()
        if self.control_protocol not in ("rs485", "canfd"):
            raise ValueError("arm_control_protocol must be 'rs485' or 'canfd'")
        self.rs485_motion_mode = str(
            node.declare_parameter("arm_rs485_motion_mode", "movej_p").value
        ).strip().lower()
        if self.rs485_motion_mode not in ("step", "movel", "movej_p"):
            raise ValueError("arm_rs485_motion_mode must be 'step', 'movel', or 'movej_p'")
        self.rs485_effective_motion_mode = self.rs485_motion_mode
        self.recover_motion_state_on_start = bool(
            node.declare_parameter("arm_recover_motion_state_on_start", True).value
        )
        self.recover_motion_state_on_tracking_start = bool(
            node.declare_parameter(
                "arm_recover_motion_state_on_tracking_start",
                False,
            ).value
        )
        self.canfd_follow = bool(node.declare_parameter("canfd_follow", False).value)
        self.canfd_trajectory_mode = int(
            node.declare_parameter("canfd_trajectory_mode", 0).value
        )
        self.canfd_radio = int(node.declare_parameter("canfd_radio", 0).value)
        self.rs485_configure_on_connect = bool(
            node.declare_parameter("rs485_configure_on_connect", False).value
        )
        self.rs485_mode = int(node.declare_parameter("rs485_mode", 0).value)
        self.rs485_baudrate = int(node.declare_parameter("rs485_baudrate", 115200).value)
        self.safe_home_joints = self._float_list_from_param(
            node.declare_parameter(
                "safe_home_joints",
                [-2.349, -91.303, 99.533, 2.447, 16.05, 0.032],
            ).value
        )
        self.auto_home_on_start = bool(
            node.declare_parameter("auto_home_on_start", True).value
        )
        self.home_key_enabled = bool(node.declare_parameter("home_key_enabled", True).value)
        self.home_speed = int(node.declare_parameter("home_speed", 20).value)
        self.home_blend_radius = int(node.declare_parameter("home_blend_radius", 0).value)
        self.home_block = int(node.declare_parameter("home_block", 1).value)
        self.home_verify_timeout_s = float(
            node.declare_parameter("home_verify_timeout_s", 8.0).value
        )
        self.home_verify_period_s = float(
            node.declare_parameter("home_verify_period_s", 0.15).value
        )
        self.home_verify_tolerance_deg = float(
            node.declare_parameter("home_verify_tolerance_deg", 1.5).value
        )
        self.joint_safety_enabled = bool(
            node.declare_parameter("joint_safety_enabled", True).value
        )
        self.target_ik_safety_enabled = bool(
            node.declare_parameter("target_ik_safety_enabled", True).value
        )
        self.singularity_safety_enabled = bool(
            node.declare_parameter("singularity_safety_enabled", True).value
        )
        self.wrist_singularity_joint5_min_abs_deg = float(
            node.declare_parameter("wrist_singularity_joint5_min_abs_deg", 15.0).value
        )
        self.singularity_safe_step_command_interval_s = float(
            node.declare_parameter("singularity_safe_step_command_interval_s", 0.8).value
        )
        self.joint_limit_margin_deg = float(
            node.declare_parameter("joint_limit_margin_deg", 2.0).value
        )
        self.arm_motion_unit_m = float(
            node.declare_parameter("arm_motion_unit_m", 0.01).value
        )
        self.palm_to_arm_scale = float(
            node.declare_parameter("palm_to_arm_scale", 1.0).value
        )
        self.mit_control_enabled = bool(
            node.declare_parameter(
                "arm_mit_control_enabled",
                persistent_default(node, "arm_mit_control_enabled", True),
            ).value
        )
        self.mit_kp = float(
            node.declare_parameter(
                "arm_mit_kp",
                persistent_default(node, "arm_mit_kp", 70.0),
            ).value
        )
        self.mit_kd = float(
            node.declare_parameter(
                "arm_mit_kd",
                persistent_default(node, "arm_mit_kd", 16.0),
            ).value
        )
        self.mit_max_velocity_m_s = float(
            node.declare_parameter(
                "arm_mit_max_velocity_m_s",
                persistent_default(node, "arm_mit_max_velocity_m_s", 0.22),
            ).value
        )
        self.mit_max_accel_m_s2 = float(
            node.declare_parameter(
                "arm_mit_max_accel_m_s2",
                persistent_default(node, "arm_mit_max_accel_m_s2", 1.5),
            ).value
        )
        mit_max_accel_typo = float(
            node.declare_parameter(
                "arm_mit_max_accl_m_s2",
                self.mit_max_accel_m_s2,
            ).value
        )
        if abs(mit_max_accel_typo - self.mit_max_accel_m_s2) > 1e-9:
            self.logger.warn(
                "Parameter arm_mit_max_accl_m_s2 is deprecated; use "
                "arm_mit_max_accel_m_s2 instead"
            )
            self.mit_max_accel_m_s2 = mit_max_accel_typo
        axis_scale_values = node.declare_parameter(
            "arm_axis_scale",
            [1.0, 1.4, 1.4],
        ).value
        self.arm_axis_scale = self._vector3_from_param(axis_scale_values, "arm_axis_scale")
        axis_deadband_values = node.declare_parameter(
            "arm_axis_deadband_m",
            persistent_default(node, "arm_axis_deadband_m", [0.003, 0.003, 0.003]),
        ).value
        self.arm_axis_deadband_m = self._vector3_from_param(
            axis_deadband_values,
            "arm_axis_deadband_m",
        )
        self.hand_deadband_m = float(
            node.declare_parameter(
                "hand_deadband_m",
                persistent_default(node, "hand_deadband_m", 0.005),
            ).value
        )
        self.palm_filter_enabled = bool(
            node.declare_parameter("palm_filter_enabled", True).value
        )
        self.palm_filter_alpha = float(
            node.declare_parameter(
                "palm_filter_alpha",
                persistent_default(node, "palm_filter_alpha", 0.55),
            ).value
        )
        self.palm_filter_still_alpha = float(
            node.declare_parameter("palm_filter_still_alpha", 0.10).value
        )
        self.palm_filter_hold_deadband_m = float(
            node.declare_parameter(
                "palm_filter_hold_deadband_m",
                persistent_default(node, "palm_filter_hold_deadband_m", 0.008),
            ).value
        )
        self.palm_filter_release_deadband_m = float(
            node.declare_parameter(
                "palm_filter_release_deadband_m",
                persistent_default(node, "palm_filter_release_deadband_m", 0.014),
            ).value
        )
        self.palm_filter_velocity_deadband_m_s = float(
            node.declare_parameter("palm_filter_velocity_deadband_m_s", 0.025).value
        )
        self.max_arm_offset_m = float(
            node.declare_parameter("max_arm_offset_m", 0.15).value
        )
        self.max_arm_target_delta_m = float(
            node.declare_parameter(
                "max_arm_target_delta_m",
                persistent_default(node, "max_arm_target_delta_m", 0.02),
            ).value
        )
        self.command_interval_s = float(
            node.declare_parameter(
                "arm_command_interval_s",
                persistent_default(node, "arm_command_interval_s", 0.15),
            ).value
        )
        self.rs485_step_min_command_interval_s = float(
            node.declare_parameter("arm_rs485_step_min_command_interval_s", 0.10).value
        )
        self.rs485_step_axis_interval_s = float(
            node.declare_parameter("arm_rs485_step_axis_interval_s", 0.02).value
        )
        self.rs485_step_max_axes_per_update = int(
            node.declare_parameter("arm_rs485_step_max_axes_per_update", 1).value
        )
        self.rs485_step_axis_switch_ratio = float(
            node.declare_parameter("arm_rs485_step_axis_switch_ratio", 1.4).value
        )
        self.rs485_step_axis_reverse_deadband_m = float(
            node.declare_parameter("arm_rs485_step_axis_reverse_deadband_m", 0.003).value
        )
        self.rs485_step_fallback_to_movel = bool(
            node.declare_parameter("arm_rs485_step_fallback_to_movel", True).value
        )
        self.rs485_step_fallback_after_stall_s = float(
            node.declare_parameter("arm_rs485_step_fallback_after_stall_s", 1.5).value
        )
        self.command_interval_s = max(
            self._min_command_interval_s(),
            self.command_interval_s,
        )
        self.sdk_stall_warn_s = float(
            node.declare_parameter("arm_sdk_stall_warn_s", 2.0).value
        )
        self.min_command_delta_m = float(
            node.declare_parameter("min_command_delta_m", 0.005).value
        )
        if (
            self.max_arm_target_delta_m > 0.0
            and self.min_command_delta_m >= self.max_arm_target_delta_m
        ):
            adjusted_min_delta = max(0.0005, self.max_arm_target_delta_m * 0.5)
            self.logger.warn(
                "min_command_delta_m is greater than or equal to "
                "max_arm_target_delta_m; this can suppress every bounded "
                "tracking target. Adjusting min_command_delta_m from "
                f"{self.min_command_delta_m:.4f}m to {adjusted_min_delta:.4f}m."
            )
            self.min_command_delta_m = adjusted_min_delta
        self.target_safety_check_interval_s = float(
            node.declare_parameter("target_safety_check_interval_s", 1.0).value
        )
        self.target_safety_allow_cached_on_read_error = bool(
            node.declare_parameter("target_safety_allow_cached_on_read_error", True).value
        )
        default_matrix = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        matrix_values = node.declare_parameter(
            "camera_to_arm_matrix",
            default_matrix,
        ).value
        self.camera_to_arm_matrix = self._matrix_from_param(matrix_values)

        self.robot = None
        self.sdk_lock = threading.RLock()
        self.arm_dof = 6
        self.joint_min_limits = None
        self.joint_max_limits = None
        self.initial_pose = None
        self.last_target_pose = None
        self.last_commanded_pose = None
        self.last_step_sent_pose = None
        self.last_step_axis_idx = None
        self.last_step_axis_sign = 0
        self.last_submit_time = 0.0
        self.last_arm_offset = None
        self.pending_motion = None
        self.home_motion_active = False
        self.motion_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker = None
        self.motion_inflight = False
        self.motion_stage = None
        self.motion_started_at = 0.0
        self.first_tracking_motion_ok = False
        self.tracking_cycle_started_at = 0.0
        self.last_motion_stall_log_time = 0.0
        self.arm_log_command_start = bool(
            node.declare_parameter("arm_log_command_start", True).value
        )
        self.home_requested = False
        self.last_target_safety_check_time = 0.0
        self.last_target_safety_check_ok = True
        self.last_target_safety_failure_reason = None
        self.mit_position_offset = np.zeros(3, dtype=np.float64)
        self.mit_velocity_offset = np.zeros(3, dtype=np.float64)
        self.last_mit_update_time = 0.0
        self.palm_filtered_offset = np.zeros(3, dtype=np.float64)
        self.last_palm_raw_offset = None
        self.last_palm_filter_time = 0.0
        self.palm_motion_locked = True
        self.last_sdk_busy_log_time = 0.0
        self.sdk_busy_log_interval_s = float(
            node.declare_parameter("sdk_busy_log_interval_s", 5.0).value
        )

        if not self.enabled:
            self.logger.warn(
                "Robot motion is disabled. Set robot_motion_enabled:=true to move RM65-B."
            )
            return

        if self.arm_block:
            self.logger.warn(
                "arm_block is enabled. Continuous palm following should normally use "
                "arm_block:=0, otherwise one slow/stalled rm_movel can block hand "
                "commands and keyboard recovery."
            )
        if (
            self.control_protocol == "rs485"
            and self.rs485_motion_mode == "movel"
            and not self.movel_redirect_to_canfd
            and self.command_interval_s < 0.2
        ):
            self.logger.warn(
                "arm_command_interval_s is very small for RS485/rm_movel planned "
                f"motion ({self.command_interval_s:.3f}s). Start with 0.25..0.35s "
                "for stability, then reduce only after hardware is stable."
            )
        if self.control_protocol == "rs485":
            self.logger.info(
                f"RS485 arm motion mode: {self.rs485_motion_mode} "
                f"(effective={self.rs485_effective_motion_mode})"
            )
            if self.rs485_motion_mode == "movel" and self.movel_redirect_to_canfd:
                self.logger.warn(
                    "RS485/movel continuous following is explicitly configured to "
                    "redirect targets to rm_movep_canfd. Disable "
                    "arm_movel_redirect_to_canfd when the arm must stay on RS485."
                )

        self._connect()
        self.worker = threading.Thread(target=self._motion_worker, daemon=True)
        self.worker.start()

    def request_home(self, reason="manual"):
        if not self.enabled:
            self.logger.warn("Ignoring home request because robot motion is disabled")
            return
        with self.motion_lock:
            self.home_requested = True
            self.home_motion_active = True
            self.pending_motion = ("home", reason)
        self.last_target_pose = None
        self.last_submit_time = 0.0
        self.last_target_safety_check_time = 0.0
        self.last_target_safety_check_ok = True
        self.logger.info(f"Queued RM65-B safe-home motion ({reason})")

    def begin_tracking_cycle(self, reason="person_start"):
        if not self.enabled:
            return True

        if not self.sdk_lock.acquire(blocking=False):
            self._warn_sdk_busy(
                "Cannot start a new tracking cycle because the RM SDK is busy. "
                "If this repeats, run with arm_block:=0 or power-cycle/reconnect the controller."
            )
            return False
        try:
            if self.recover_motion_state_on_tracking_start:
                self._recover_motion_state_locked("tracking_start")
            self._refresh_initial_pose_from_current_state()
        finally:
            self.sdk_lock.release()

        with self.motion_lock:
            self.pending_motion = None
            self.home_requested = False
            self.home_motion_active = False
        self.last_target_pose = list(self.initial_pose) if self.initial_pose else None
        self.last_commanded_pose = list(self.initial_pose) if self.initial_pose else None
        self.last_submit_time = 0.0
        self.last_arm_offset = np.zeros(3, dtype=np.float64)
        self.first_tracking_motion_ok = False
        self.tracking_cycle_started_at = time.monotonic()
        self._reset_mit_state()
        self._reset_palm_filter_state()
        self.logger.info(f"RM65-B control reference reset ({reason})")
        return True

    def _recover_motion_state_locked(self, reason):
        recover_calls = (
            ("slow_stop", "rm_set_arm_slow_stop", ()),
            ("stop", "rm_set_arm_stop", ()),
            ("delete_trajectory", "rm_set_arm_delete_trajectory", ()),
            ("clear_system_err", "rm_clear_system_err", ()),
            ("power_on", "rm_set_arm_power", (1,)),
            ("continue", "rm_set_arm_continue", ()),
        )
        results = []
        for label, method_name, args in recover_calls:
            method = getattr(self.robot, method_name, None)
            if method is None:
                continue
            self.motion_stage = f"recover:{label}:{reason}"
            self.logger.info(
                f"RM65-B motion state recovery step start "
                f"({reason}): {label}"
            )
            try:
                ret = method(*args)
            except Exception as exc:
                results.append(f"{label}=exc:{exc}")
                self.logger.warn(
                    f"RM65-B motion state recovery step failed "
                    f"({reason}): {label}, exc={exc}"
                )
                continue
            results.append(f"{label}={ret}")
            self.logger.info(
                f"RM65-B motion state recovery step done "
                f"({reason}): {label}={ret}"
            )
            time.sleep(0.05)
        self.logger.info(
            f"RM65-B motion state recovery ({reason}): {', '.join(results)}"
        )

    def mit_tuning_values(self):
        return {
            "arm_mit_control_enabled": bool(self.mit_control_enabled),
            "arm_mit_kp": float(self.mit_kp),
            "arm_mit_kd": float(self.mit_kd),
            "arm_mit_max_velocity_m_s": float(self.mit_max_velocity_m_s),
            "arm_mit_max_accel_m_s2": float(self.mit_max_accel_m_s2),
            "max_arm_target_delta_m": float(self.max_arm_target_delta_m),
            "arm_command_interval_s": float(self.command_interval_s),
            "hand_deadband_m": float(self.hand_deadband_m),
            "arm_axis_deadband_m": [float(value) for value in self.arm_axis_deadband_m],
            "palm_filter_alpha": float(self.palm_filter_alpha),
            "palm_filter_hold_deadband_m": float(self.palm_filter_hold_deadband_m),
            "palm_filter_release_deadband_m": float(self.palm_filter_release_deadband_m),
        }

    def apply_mit_tuning(self, values, reason="runtime_tuning"):
        self.mit_control_enabled = bool(
            values.get("arm_mit_control_enabled", self.mit_control_enabled)
        )
        self.mit_kp = max(1.0, float(values.get("arm_mit_kp", self.mit_kp)))
        self.mit_kd = max(0.0, float(values.get("arm_mit_kd", self.mit_kd)))
        self.mit_max_velocity_m_s = max(
            0.01,
            float(values.get("arm_mit_max_velocity_m_s", self.mit_max_velocity_m_s)),
        )
        self.mit_max_accel_m_s2 = max(
            0.01,
            float(values.get("arm_mit_max_accel_m_s2", self.mit_max_accel_m_s2)),
        )
        self.max_arm_target_delta_m = max(
            0.001,
            float(values.get("max_arm_target_delta_m", self.max_arm_target_delta_m)),
        )
        if (
            self.max_arm_target_delta_m > 0.0
            and self.min_command_delta_m >= self.max_arm_target_delta_m
        ):
            self.min_command_delta_m = max(0.0005, self.max_arm_target_delta_m * 0.5)
        self.command_interval_s = max(
            self._min_command_interval_s(),
            float(values.get("arm_command_interval_s", self.command_interval_s)),
        )
        self.hand_deadband_m = max(
            0.0,
            float(values.get("hand_deadband_m", self.hand_deadband_m)),
        )
        if "arm_axis_deadband_m" in values:
            self.arm_axis_deadband_m = self._vector3_from_param(
                values["arm_axis_deadband_m"],
                "arm_axis_deadband_m",
            )
        self.palm_filter_alpha = max(
            0.0,
            min(1.0, float(values.get("palm_filter_alpha", self.palm_filter_alpha))),
        )
        self.palm_filter_hold_deadband_m = max(
            0.0,
            float(
                values.get(
                    "palm_filter_hold_deadband_m",
                    self.palm_filter_hold_deadband_m,
                )
            ),
        )
        self.palm_filter_release_deadband_m = max(
            self.palm_filter_hold_deadband_m,
            float(
                values.get(
                    "palm_filter_release_deadband_m",
                    self.palm_filter_release_deadband_m,
                )
            ),
        )
        self._reset_mit_state()
        self._reset_palm_filter_state()
        self.logger.info(
            "Applied arm tuning "
            f"({reason}): enabled={self.mit_control_enabled}, "
            f"kp={self.mit_kp:.1f}, kd={self.mit_kd:.1f}, "
            f"vmax={self.mit_max_velocity_m_s:.3f}m/s, "
            f"amax={self.mit_max_accel_m_s2:.3f}m/s^2, "
            f"max_delta={self.max_arm_target_delta_m:.3f}m, "
            f"min_cmd_delta={self.min_command_delta_m:.4f}m, "
            f"cmd_interval={self.command_interval_s:.3f}s, "
            f"palm_deadband={self.hand_deadband_m:.3f}m, "
            f"axis_deadband={self.arm_axis_deadband_m.tolist()}, "
            f"palm_filter_alpha={self.palm_filter_alpha:.2f}, "
            f"palm_hold={self.palm_filter_hold_deadband_m:.3f}m, "
            f"palm_release={self.palm_filter_release_deadband_m:.3f}m"
        )

    def _min_command_interval_s(self):
        motion_mode = getattr(self, "rs485_effective_motion_mode", self.rs485_motion_mode)
        if (
            self.control_protocol == "rs485"
            and motion_mode == "movel"
            and self.movel_redirect_to_canfd
        ):
            return 0.03
        if self.control_protocol == "rs485" and motion_mode == "step":
            intervals = [0.03, self.rs485_step_min_command_interval_s]
            if self.singularity_safety_enabled:
                intervals.append(self.singularity_safe_step_command_interval_s)
            return max(intervals)
        if self.control_protocol == "rs485" and motion_mode in ("movel", "movej_p"):
            return 0.20
        return 0.03

    def _maybe_fallback_step_motion(self, ret, elapsed_s):
        if self.control_protocol != "rs485":
            return
        if self.rs485_effective_motion_mode != "step":
            return
        if not self.rs485_step_fallback_to_movel:
            return

        fallback_reason = None
        if elapsed_s >= max(0.1, self.rs485_step_fallback_after_stall_s):
            fallback_reason = f"step SDK call took {elapsed_s:.2f}s"
        elif ret != 0:
            fallback_reason = f"step command returned code={ret}"
        if fallback_reason is None:
            return

        self.rs485_effective_motion_mode = "movel"
        self.command_interval_s = max(self.command_interval_s, self._min_command_interval_s())
        target_backend = (
            "rm_movep_canfd"
            if self.movel_redirect_to_canfd
            else "rm_movel"
        )
        self.logger.warn(
            "Switching RS485 arm motion mode from step to movel because "
            f"{fallback_reason}. Future targets will use {target_backend}; set "
            "arm_rs485_motion_mode:=movel to start in this mode."
        )

    def pause_control(self, reason="tracking_paused"):
        if not self.enabled:
            return
        with self.motion_lock:
            self.pending_motion = None
            self.home_requested = False
        self.last_target_pose = None
        self.last_commanded_pose = None
        self.last_submit_time = 0.0
        self._reset_mit_state()
        self._reset_palm_filter_state()
        if self.slow_stop_on_pause:
            self._try_slow_stop(reason)
        self.logger.info(f"RM65-B control paused ({reason})")

    def has_pending_or_active_motion(self):
        if not self.enabled:
            return False
        if self.motion_inflight:
            return True
        with self.motion_lock:
            return (
                self.pending_motion is not None
                or self.home_requested
                or self.home_motion_active
            )

    def update_from_palm_displacement(self, displacement):
        if not self.enabled or self.initial_pose is None or displacement is None:
            return None
        with self.motion_lock:
            if self.home_requested:
                return self.last_arm_offset
            if self.home_motion_active:
                return self.last_arm_offset

        camera_offset = np.array(displacement, dtype=np.float64)
        arm_offset = self.camera_to_arm_matrix.dot(camera_offset)
        arm_offset *= self.palm_to_arm_scale
        arm_offset *= self.arm_axis_scale
        arm_offset = self._apply_axis_deadband(arm_offset, self.arm_axis_deadband_m)

        if np.linalg.norm(arm_offset) < self.hand_deadband_m:
            arm_offset = np.zeros(3, dtype=np.float64)
        else:
            arm_offset = self._limit_norm(arm_offset, self.max_arm_offset_m)
        arm_offset = self._anti_drift_filter_offset(arm_offset)
        arm_offset = self._mit_filter_offset(arm_offset)
        arm_offset = self._quantize(arm_offset, self.arm_motion_unit_m)

        target_pose = list(self.initial_pose)
        for idx in range(3):
            target_pose[idx] = self.initial_pose[idx] + float(arm_offset[idx])
        target_pose, arm_offset = self._limit_target_delta(target_pose, arm_offset)

        now = time.monotonic()
        if now - self.last_submit_time < self.command_interval_s:
            return self.last_arm_offset

        if self.last_target_pose is not None:
            delta = np.array(target_pose[:3]) - np.array(self.last_target_pose[:3])
            if np.linalg.norm(delta) < self.min_command_delta_m:
                return self.last_arm_offset

        with self.motion_lock:
            self.pending_motion = ("pose", target_pose)
        self.last_target_pose = target_pose
        self.last_submit_time = now
        self.last_arm_offset = arm_offset
        return arm_offset

    def close(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=2.0)
            self.worker = None

        if self.robot is not None:
            try:
                with self.sdk_lock:
                    ret = self.robot.rm_delete_robot_arm()
                if ret == 0:
                    self.logger.info("Disconnected from RM65-B")
                else:
                    self.logger.warn(f"Failed to disconnect RM65-B, code={ret}")
            except Exception as exc:
                self.logger.warn(f"Failed to close RM65-B connection: {exc}")
            finally:
                self.robot = None

    def _connect(self):
        self.robot = RoboticArm(rm_thread_mode_e(self.thread_mode))
        handle = self.robot.rm_create_robot_arm(
            self.robot_ip,
            self.robot_port,
            self.robot_level,
        )
        if handle.id == -1:
            raise RuntimeError(f"Failed to connect RM65-B at {self.robot_ip}:{self.robot_port}")

        self.logger.info(
            f"Connected to robot arm handle={handle.id}, API={rm_api_version()}, "
            f"protocol={self.control_protocol.upper()}"
        )

        info_ret, info = self.robot.rm_get_robot_info()
        if info_ret == 0:
            model = info.get("arm_model")
            self.arm_dof = int(info.get("arm_dof") or self.arm_dof)
            if model != "RM_65":
                self.logger.warn(f"Connected arm model is {model}; expected RM_65/RM65-B")
        else:
            self.logger.warn(f"Failed to read robot model, code={info_ret}")

        self._load_joint_limits()
        self._validate_home_joints()

        if self.control_protocol == "rs485":
            self._configure_rs485_if_requested()

        if self.recover_motion_state_on_start:
            with self.sdk_lock:
                self._recover_motion_state_locked("startup")

        if self.auto_home_on_start:
            self._move_home_blocking("startup")

        state_ret, state = self.robot.rm_get_current_arm_state()
        if state_ret != 0:
            raise RuntimeError(f"Failed to read current arm state, code={state_ret}")
        self.initial_pose = [float(value) for value in state["pose"]]
        self.last_target_pose = list(self.initial_pose)
        self.last_commanded_pose = list(self.initial_pose)
        self.logger.info(f"Initial TCP pose: {self.initial_pose}")

    def _motion_worker(self):
        while not self.stop_event.is_set():
            with self.motion_lock:
                motion = self.pending_motion
                self.pending_motion = None

            if motion is None:
                time.sleep(0.02)
                continue

            try:
                motion_type, payload = motion
                if motion_type == "home":
                    try:
                        self._move_home_blocking(payload)
                    finally:
                        with self.motion_lock:
                            self.home_requested = False
                            self.home_motion_active = False
                    continue

                pose = payload
                with self.motion_lock:
                    if self.home_requested:
                        continue
                self._mark_motion_started("target_pose")
                started_at = time.monotonic()
                ret = self._send_target_pose(pose)
                elapsed = time.monotonic() - started_at
                sent_stage = self.motion_stage
                self._mark_motion_finished()
                self._maybe_fallback_step_motion(ret, elapsed)
                with self.motion_lock:
                    if self.home_requested:
                        continue
                if ret == 0:
                    commanded_pose = self.last_step_sent_pose or pose
                    self.last_commanded_pose = list(commanded_pose)
                    if self.rs485_effective_motion_mode == "step":
                        self.last_target_pose = list(commanded_pose)
                    self.first_tracking_motion_ok = True
                    self.logger.info(
                        f"RM65-B {self.control_protocol.upper()} target pose sent "
                        f"via {sent_stage}: {commanded_pose}"
                    )
                else:
                    self.logger.error(
                        f"RM65-B {self.control_protocol.upper()} command failed, "
                        f"code={ret}, pose={pose}"
                    )
            except Exception as exc:
                self._mark_motion_finished()
                self.logger.error(f"RM65-B motion command failed: {exc}")
                self.logger.error(traceback.format_exc())

    def _send_target_pose(self, pose):
        with self.sdk_lock:
            self.motion_stage = "safety_check"
            if not self._target_pose_is_safe(pose):
                return -1001

            if self.control_protocol == "canfd":
                self.motion_stage = "rm_movep_canfd"
                self._log_arm_command_start(self.motion_stage, pose)
                return self.robot.rm_movep_canfd(
                    pose,
                    self.canfd_follow,
                    self.canfd_trajectory_mode,
                    self.canfd_radio,
                )

            if self.rs485_effective_motion_mode == "step":
                self.motion_stage = "rm_set_pos_step"
                self._log_arm_command_start(self.motion_stage, pose)
                self.last_step_sent_pose = None
                return self._send_position_steps(pose)

            if self.rs485_effective_motion_mode == "movej_p":
                self.motion_stage = "rm_movej_p"
                self._log_arm_command_start(self.motion_stage, pose)
                return self.robot.rm_movej_p(
                    pose,
                    self.arm_speed,
                    self.arm_blend_radius,
                    0,
                    self.arm_block,
                )

            if self.movel_redirect_to_canfd:
                self.motion_stage = "rm_movep_canfd:redirected_from_movel"
                self._log_arm_command_start(self.motion_stage, pose)
                return self.robot.rm_movep_canfd(
                    pose,
                    self.canfd_follow,
                    self.canfd_trajectory_mode,
                    self.canfd_radio,
                )

            self.motion_stage = "rm_movel"
            self._log_arm_command_start(self.motion_stage, pose)
            return self.robot.rm_movel(
                pose,
                self.arm_speed,
                self.arm_blend_radius,
                0,
                self.arm_block,
            )

    def _log_arm_command_start(self, stage, pose):
        if not self.arm_log_command_start:
            return
        rounded_pose = [round(float(value), 6) for value in pose]
        self.logger.info(f"RM65-B command start via {stage}: pose={rounded_pose}")

    def _send_position_steps(self, target_pose):
        reference_pose = self.last_commanded_pose or self.initial_pose
        if reference_pose is None:
            return -1003

        delta = np.array(target_pose[:3], dtype=np.float64) - np.array(
            reference_pose[:3],
            dtype=np.float64,
        )
        axes = [
            (0, rm_pos_teach_type_e.RM_X_DIR_E, float(delta[0]), "x"),
            (1, rm_pos_teach_type_e.RM_Y_DIR_E, float(delta[1]), "y"),
            (2, rm_pos_teach_type_e.RM_Z_DIR_E, float(delta[2]), "z"),
        ]
        axes = [axis for axis in axes if abs(axis[2]) >= 1e-4]
        max_axes = max(0, int(self.rs485_step_max_axes_per_update))
        if max_axes == 1 and len(axes) > 1:
            axes = self._select_single_step_axis_with_hysteresis(axes)
        elif max_axes > 0 and len(axes) > max_axes:
            axes = sorted(axes, key=lambda axis: abs(axis[2]), reverse=True)[:max_axes]
            axes = sorted(axes, key=lambda axis: axis[0])

        sent_any = False
        sent_pose = list(reference_pose)
        for axis_idx, teach_type, step, axis_name in axes:
            with self.motion_lock:
                if self.home_requested:
                    return -1004
            self.motion_stage = f"rm_set_pos_step:{axis_name}"
            self._log_arm_command_start(self.motion_stage, target_pose)
            ret = self.robot.rm_set_pos_step(
                teach_type,
                step,
                self.arm_speed,
                self.arm_block,
            )
            if ret != 0:
                return ret
            sent_any = True
            sent_pose[axis_idx] = float(sent_pose[axis_idx]) + step
            self.last_step_axis_idx = axis_idx
            self.last_step_axis_sign = 1 if step > 0.0 else -1
            if self.rs485_step_axis_interval_s > 0.0:
                time.sleep(min(0.10, self.rs485_step_axis_interval_s))

        if sent_any:
            self.last_step_sent_pose = sent_pose
        return 0 if sent_any else 0

    def _select_single_step_axis_with_hysteresis(self, axes):
        dominant = max(axes, key=lambda axis: abs(axis[2]))
        last_axis = None
        for axis in axes:
            if axis[0] == self.last_step_axis_idx:
                last_axis = axis
                break

        if last_axis is None:
            return [dominant]

        _, _, last_step, _ = last_axis
        dominant_abs = abs(dominant[2])
        last_abs = abs(last_step)
        switch_ratio = max(1.0, float(self.rs485_step_axis_switch_ratio))
        if last_abs > 0.0 and dominant[0] != last_axis[0]:
            if dominant_abs < last_abs * switch_ratio:
                dominant = last_axis

        step_sign = 1 if dominant[2] > 0.0 else -1
        if (
            dominant[0] == self.last_step_axis_idx
            and self.last_step_axis_sign
            and step_sign != self.last_step_axis_sign
            and abs(dominant[2]) < self.rs485_step_axis_reverse_deadband_m
        ):
            return []
        return [dominant]

    def _limit_target_delta(self, target_pose, arm_offset):
        reference_pose = self._target_delta_reference_pose()
        if self.max_arm_target_delta_m <= 0.0 or reference_pose is None:
            return target_pose, arm_offset

        previous_xyz = np.array(reference_pose[:3], dtype=np.float64)
        target_xyz = np.array(target_pose[:3], dtype=np.float64)
        delta = target_xyz - previous_xyz
        delta_norm = np.linalg.norm(delta)
        if delta_norm <= self.max_arm_target_delta_m or delta_norm < 1e-9:
            return target_pose, arm_offset

        limited_xyz = previous_xyz + delta * (self.max_arm_target_delta_m / delta_norm)
        limited_pose = list(target_pose)
        for idx in range(3):
            limited_pose[idx] = float(limited_xyz[idx])

        if self.initial_pose is None:
            return limited_pose, arm_offset
        limited_offset = limited_xyz - np.array(self.initial_pose[:3], dtype=np.float64)
        return limited_pose, limited_offset

    def _target_delta_reference_pose(self):
        motion_mode = getattr(self, "rs485_effective_motion_mode", self.rs485_motion_mode)
        if self.control_protocol == "rs485" and motion_mode == "step":
            return self.last_commanded_pose or self.last_target_pose
        return self.last_target_pose or self.last_commanded_pose

    def _mit_filter_offset(self, desired_offset):
        desired_offset = np.array(desired_offset, dtype=np.float64)
        if not self.mit_control_enabled:
            self.mit_position_offset = desired_offset.copy()
            self.mit_velocity_offset = np.zeros(3, dtype=np.float64)
            self.last_mit_update_time = time.monotonic()
            return desired_offset

        now = time.monotonic()
        if self.last_mit_update_time <= 0.0:
            self.last_mit_update_time = now
            self.mit_position_offset = desired_offset.copy()
            self.mit_velocity_offset = np.zeros(3, dtype=np.float64)
            return desired_offset

        dt = max(0.001, min(now - self.last_mit_update_time, 0.08))
        self.last_mit_update_time = now

        error = desired_offset - self.mit_position_offset
        acceleration = self.mit_kp * error - self.mit_kd * self.mit_velocity_offset
        acceleration = self._limit_norm(acceleration, self.mit_max_accel_m_s2)

        self.mit_velocity_offset += acceleration * dt
        self.mit_velocity_offset = self._limit_norm(
            self.mit_velocity_offset,
            self.mit_max_velocity_m_s,
        )
        self.mit_position_offset += self.mit_velocity_offset * dt
        self.mit_position_offset = self._limit_norm(
            self.mit_position_offset,
            self.max_arm_offset_m,
        )

        if (
            np.linalg.norm(error) < max(self.min_command_delta_m, 1e-4)
            and np.linalg.norm(self.mit_velocity_offset) < self.min_command_delta_m
        ):
            self.mit_position_offset = desired_offset.copy()
            self.mit_velocity_offset = np.zeros(3, dtype=np.float64)

        return self.mit_position_offset.copy()

    def _anti_drift_filter_offset(self, raw_offset):
        raw_offset = np.array(raw_offset, dtype=np.float64)
        if not self.palm_filter_enabled:
            self.palm_filtered_offset = raw_offset.copy()
            self.last_palm_raw_offset = raw_offset.copy()
            self.last_palm_filter_time = time.monotonic()
            self.palm_motion_locked = False
            return raw_offset

        now = time.monotonic()
        if self.last_palm_filter_time <= 0.0 or self.last_palm_raw_offset is None:
            self.last_palm_filter_time = now
            self.last_palm_raw_offset = raw_offset.copy()
            if np.linalg.norm(raw_offset) < self.palm_filter_release_deadband_m:
                self.palm_filtered_offset = np.zeros(3, dtype=np.float64)
                self.palm_motion_locked = True
                return self.palm_filtered_offset.copy()
            self.palm_filtered_offset = raw_offset.copy()
            self.palm_motion_locked = False
            return self.palm_filtered_offset.copy()

        dt = max(0.001, min(now - self.last_palm_filter_time, 0.2))
        raw_speed = np.linalg.norm(raw_offset - self.last_palm_raw_offset) / dt
        raw_norm = np.linalg.norm(raw_offset)
        self.last_palm_filter_time = now
        self.last_palm_raw_offset = raw_offset.copy()

        if (
            raw_norm < self.palm_filter_hold_deadband_m
            and raw_speed < self.palm_filter_velocity_deadband_m_s
        ):
            self.palm_motion_locked = True

        if self.palm_motion_locked:
            if (
                raw_norm >= self.palm_filter_release_deadband_m
                or raw_speed >= 2.0 * self.palm_filter_velocity_deadband_m_s
            ):
                self.palm_motion_locked = False
            else:
                self.palm_filtered_offset = np.zeros(3, dtype=np.float64)
                return self.palm_filtered_offset.copy()

        alpha = self.palm_filter_alpha
        if raw_speed < self.palm_filter_velocity_deadband_m_s:
            alpha = min(alpha, self.palm_filter_still_alpha)
        alpha = max(0.0, min(1.0, alpha))
        self.palm_filtered_offset = (
            alpha * raw_offset
            + (1.0 - alpha) * self.palm_filtered_offset
        )

        if (
            np.linalg.norm(self.palm_filtered_offset) < self.palm_filter_hold_deadband_m
            and raw_speed < self.palm_filter_velocity_deadband_m_s
        ):
            self.palm_filtered_offset = np.zeros(3, dtype=np.float64)
            self.palm_motion_locked = True

        return self.palm_filtered_offset.copy()

    def _reset_mit_state(self):
        self.mit_position_offset = np.zeros(3, dtype=np.float64)
        self.mit_velocity_offset = np.zeros(3, dtype=np.float64)
        self.last_mit_update_time = 0.0

    def _reset_palm_filter_state(self):
        self.palm_filtered_offset = np.zeros(3, dtype=np.float64)
        self.last_palm_raw_offset = None
        self.last_palm_filter_time = 0.0
        self.palm_motion_locked = True

    def _move_home_blocking(self, reason):
        with self.sdk_lock:
            self._mark_motion_started(f"home:{reason}")
            try:
                if not self._joints_within_limits(
                    self.safe_home_joints, label="safe_home_joints"
                ):
                    self.logger.error(
                        "Refusing safe-home motion because safe_home_joints exceed limits"
                    )
                    return -1002

                self.motion_stage = f"rm_movej:{reason}"
                ret = self.robot.rm_movej(
                    self.safe_home_joints,
                    self.home_speed,
                    self.home_blend_radius,
                    0,
                    self.home_block,
                )
                if ret == 0:
                    self._wait_until_home_reached(reason)
                    self.logger.info(
                        f"RM65-B returned to safe home ({reason}): {self.safe_home_joints}"
                    )
                    self.motion_stage = f"refresh_state:{reason}"
                    self._refresh_initial_pose_from_current_state()
                    self.last_arm_offset = np.zeros(3, dtype=np.float64)
                    self.last_target_pose = (
                        list(self.initial_pose) if self.initial_pose else None
                    )
                    self.last_commanded_pose = (
                        list(self.initial_pose) if self.initial_pose else None
                    )
                    self._reset_mit_state()
                    self._reset_palm_filter_state()
                else:
                    self.logger.error(f"RM65-B safe-home movej failed, code={ret}")
                return ret
            finally:
                self._mark_motion_finished()

    def _wait_until_home_reached(self, reason):
        if self.home_verify_timeout_s <= 0.0:
            return True

        deadline = time.monotonic() + self.home_verify_timeout_s
        last_error = None
        period = max(0.05, self.home_verify_period_s)
        while time.monotonic() < deadline and not self.stop_event.is_set():
            current_ret, current_joints = self.robot.rm_get_joint_degree()
            if current_ret == 0:
                current_joints = [float(value) for value in current_joints[: self.arm_dof]]
                errors = [
                    abs(current_joints[idx] - self.safe_home_joints[idx])
                    for idx in range(min(self.arm_dof, len(self.safe_home_joints)))
                ]
                last_error = max(errors) if errors else None
                if last_error is not None and last_error <= self.home_verify_tolerance_deg:
                    return True
            else:
                last_error = f"read_error={current_ret}"
            time.sleep(period)

        self.logger.warn(
            "Timed out waiting for RM65-B to settle at safe home "
            f"({reason}); last_error={last_error}"
        )
        return False

    def _refresh_initial_pose_from_current_state(self):
        state_ret, state = self.robot.rm_get_current_arm_state()
        if state_ret != 0:
            self.logger.warn(f"Failed to refresh initial TCP pose, code={state_ret}")
            return
        self.initial_pose = [float(value) for value in state["pose"]]
        self.logger.info(f"Updated hand-control TCP reference pose: {self.initial_pose}")

    def _load_joint_limits(self):
        if not self.joint_safety_enabled:
            self.logger.warn("Joint safety checks are disabled by parameter")
            return

        min_ret, min_limits = self.robot.rm_get_joint_min_pos()
        max_ret, max_limits = self.robot.rm_get_joint_max_pos()
        if min_ret != 0 or max_ret != 0:
            raise RuntimeError(
                f"Failed to read official joint limits, min_code={min_ret}, max_code={max_ret}"
            )

        self.joint_min_limits = [float(value) for value in min_limits[: self.arm_dof]]
        self.joint_max_limits = [float(value) for value in max_limits[: self.arm_dof]]
        self.robot.rm_algo_set_joint_min_limit(self.joint_min_limits)
        self.robot.rm_algo_set_joint_max_limit(self.joint_max_limits)
        self.logger.info(
            "Official joint limits loaded with "
            f"{self.joint_limit_margin_deg:.2f} deg margin: "
            f"min={self.joint_min_limits}, max={self.joint_max_limits}"
        )

    def _validate_home_joints(self):
        if len(self.safe_home_joints) != self.arm_dof:
            raise ValueError(
                f"safe_home_joints must contain {self.arm_dof} values, "
                f"got {len(self.safe_home_joints)}"
            )
        if not self._joints_within_limits(self.safe_home_joints, label="safe_home_joints"):
            raise ValueError("safe_home_joints exceed official joint limits")
        if not self._joints_away_from_singularity(
            self.safe_home_joints,
            label="safe_home_joints",
        ):
            raise ValueError(
                "safe_home_joints are too close to the RM65 wrist singularity; "
                "choose a home pose with |J5| above "
                f"{self.wrist_singularity_joint5_min_abs_deg:.1f} deg"
            )

    def _target_pose_is_safe(self, pose):
        if not self.joint_safety_enabled:
            return True

        now = time.monotonic()
        use_cached_follow_check = (
            self.target_safety_check_interval_s > 0.0
            and not self.singularity_safety_enabled
            and (
                self.control_protocol == "canfd"
                or (
                    self.control_protocol == "rs485"
                    and (
                        self.rs485_effective_motion_mode == "step"
                        or self.rs485_effective_motion_mode == "movej_p"
                        or (
                            self.rs485_effective_motion_mode == "movel"
                            and self.movel_redirect_to_canfd
                        )
                    )
                )
            )
        )
        if (
            use_cached_follow_check
            and now - self.last_target_safety_check_time
            < self.target_safety_check_interval_s
        ):
            return self.last_target_safety_check_ok

        safe = self._evaluate_target_pose_is_safe(pose)
        if (
            not safe
            and use_cached_follow_check
            and self.target_safety_allow_cached_on_read_error
            and self.last_target_safety_ok_recent(now)
            and self.last_target_safety_failure_reason == "current_joint_read"
        ):
            self.logger.warn(
                "Current-joint read timed out during bounded follow tracking; "
                "using the last successful safety state for this command"
            )
            return True
        if use_cached_follow_check:
            self.last_target_safety_check_time = now
            self.last_target_safety_check_ok = safe
        return safe

    def _evaluate_target_pose_is_safe(self, pose):
        self.last_target_safety_failure_reason = None
        current_ret, current_joints = self.robot.rm_get_joint_degree()
        if current_ret != 0:
            self.last_target_safety_failure_reason = "current_joint_read"
            self.logger.error(f"Failed to read current joints, code={current_ret}")
            return False

        current_joints = [float(value) for value in current_joints[: self.arm_dof]]
        if not self._joints_within_limits(current_joints, label="current_joints"):
            self.last_target_safety_failure_reason = "current_joint_limit"
            return False
        if not self._joints_away_from_singularity(
            current_joints,
            label="current_joints",
        ):
            self.last_target_safety_failure_reason = "current_singularity"
            return False

        if not self.target_ik_safety_enabled:
            return True

        q_in = self._pad_joints_for_sdk(current_joints)
        params = rm_inverse_kinematics_params_t(q_in=q_in, q_pose=pose, flag=1)
        ik_ret, target_joints = self.robot.rm_algo_inverse_kinematics(params)
        if ik_ret != 0:
            self.last_target_safety_failure_reason = "target_ik"
            self.logger.warn(f"Refusing target pose because IK failed, code={ik_ret}")
            return False

        target_joints = [float(value) for value in target_joints[: self.arm_dof]]
        if not self._joints_within_limits(target_joints, label="target_ik_joints"):
            self.last_target_safety_failure_reason = "target_joint_limit"
            return False
        if not self._joints_away_from_singularity(
            target_joints,
            label="target_ik_joints",
        ):
            self.last_target_safety_failure_reason = "target_singularity"
            return False

        try:
            limit_check = self.robot.rm_algo_ikine_check_joint_position_limit(
                self._pad_joints_for_sdk(target_joints)
            )
            if limit_check != 0:
                self.last_target_safety_failure_reason = "sdk_target_joint_limit"
                self.logger.warn(
                    "Refusing target pose because SDK IK joint-limit check "
                    f"failed: {limit_check}"
                )
                return False
        except Exception as exc:
            self.logger.debug(f"SDK IK joint-limit helper unavailable: {exc}")
        return True

    def _joints_away_from_singularity(self, joints, label):
        if not self.singularity_safety_enabled:
            return True
        if self.arm_dof < 5 or len(joints) < 5:
            return True
        j5 = float(joints[4])
        min_abs = max(0.0, float(self.wrist_singularity_joint5_min_abs_deg))
        if abs(j5) < min_abs:
            self.logger.warn(
                f"{label}[5]={j5:.3f} deg is too close to the RM65 wrist "
                f"singularity; require |J5| >= {min_abs:.3f} deg"
            )
            return False
        return True

    def last_target_safety_ok_recent(self, now):
        if not self.last_target_safety_check_ok:
            return False
        if self.last_target_safety_check_time <= 0.0:
            return False
        return now - self.last_target_safety_check_time <= max(
            self.target_safety_check_interval_s * 2.0,
            0.5,
        )

    def _joints_within_limits(self, joints, label):
        if not self.joint_safety_enabled:
            return True
        if self.joint_min_limits is None or self.joint_max_limits is None:
            self.logger.error("Joint limits are unavailable; refusing motion")
            return False

        values = [float(value) for value in joints[: self.arm_dof]]
        for index, value in enumerate(values):
            lower = self.joint_min_limits[index] + self.joint_limit_margin_deg
            upper = self.joint_max_limits[index] - self.joint_limit_margin_deg
            if value < lower or value > upper:
                self.logger.warn(
                    f"{label}[{index + 1}]={value:.3f} deg exceeds safe range "
                    f"[{lower:.3f}, {upper:.3f}] deg"
                )
                return False
        return True

    def _pad_joints_for_sdk(self, joints):
        values = [float(value) for value in joints[: self.arm_dof]]
        while len(values) < 7:
            values.append(0.0)
        return values[:7]

    def _configure_rs485_if_requested(self):
        if not self.rs485_configure_on_connect:
            return

        ret = self.robot.rm_set_controller_rs485_mode(
            self.rs485_mode,
            self.rs485_baudrate,
        )
        if ret == 0:
            self.logger.info(
                f"Controller RS485 configured: mode={self.rs485_mode}, "
                f"baudrate={self.rs485_baudrate}"
            )
        else:
            self.logger.warn(
                f"Failed to configure controller RS485, code={ret}. "
                "Motion commands will still be sent by the selected SDK interface."
            )

    def _warn_sdk_busy(self, message):
        now = time.monotonic()
        if now - self.last_sdk_busy_log_time < self.sdk_busy_log_interval_s:
            return
        self.logger.warn(message)
        self.last_sdk_busy_log_time = now

    def _try_slow_stop(self, reason):
        if self.robot is None:
            return
        if not self.sdk_lock.acquire(blocking=False):
            self._warn_sdk_busy(
                f"Could not slow-stop RM65-B while pausing ({reason}) because "
                "the shared RM SDK connection is busy"
            )
            return
        try:
            ret = self.robot.rm_set_arm_slow_stop()
            if ret == 0:
                self.logger.info(f"RM65-B slow stop sent ({reason})")
            else:
                self.logger.warn(f"RM65-B slow stop failed, code={ret} ({reason})")
        except Exception as exc:
            self.logger.warn(f"RM65-B slow stop failed ({reason}): {exc}")
        finally:
            self.sdk_lock.release()

    def warn_if_motion_stalled(self):
        if not self.enabled or not self.motion_inflight:
            return
        now = time.monotonic()
        elapsed = now - self.motion_started_at
        if elapsed < self.sdk_stall_warn_s:
            return
        if now - self.last_motion_stall_log_time < self.sdk_busy_log_interval_s:
            return
        self.logger.warn(
            f"RM65-B SDK call is still busy after {elapsed:.1f}s "
            f"(stage={self.motion_stage}). New arm targets and shared-connection "
            "commands cannot run until this SDK call returns."
        )
        self.last_motion_stall_log_time = now

    def _mark_motion_started(self, stage):
        self.motion_inflight = True
        self.motion_stage = stage
        self.motion_started_at = time.monotonic()

    def _mark_motion_finished(self):
        self.motion_inflight = False
        self.motion_stage = None

    @staticmethod
    def _limit_norm(vector, max_norm):
        norm = np.linalg.norm(vector)
        if max_norm <= 0.0 or norm <= max_norm or norm < 1e-9:
            return vector
        return vector * (max_norm / norm)

    @staticmethod
    def _quantize(vector, unit):
        if unit <= 0.0:
            return vector
        return np.round(vector / unit) * unit

    @staticmethod
    def _apply_axis_deadband(vector, deadband):
        filtered = np.array(vector, dtype=np.float64)
        for index in range(3):
            if abs(filtered[index]) < deadband[index]:
                filtered[index] = 0.0
        return filtered

    @staticmethod
    def _matrix_from_param(values):
        values = [float(value) for value in values]
        if len(values) != 9:
            raise ValueError("camera_to_arm_matrix must contain 9 float values")
        return np.array(values, dtype=np.float64).reshape((3, 3))

    @staticmethod
    def _vector3_from_param(values, name):
        values = np.array([float(value) for value in values], dtype=np.float64)
        if len(values) != 3:
            raise ValueError(f"{name} must contain 3 float values")
        return values

    @staticmethod
    def _float_list_from_param(values):
        return [float(value) for value in values]


REVO2_REG_HAND_TYPE = 901
REVO2_REG_FAST_POSITION = 1070
REVO2_REG_ACTUAL_POSITION = 2000
REVO2_FACTORY_SLAVE_IDS = (127, 126)


def raise_for_rm_hand_error(error_code, operation):
    if error_code == 0:
        return
    hints = {
        1: (
            "controller reported failure; check Revo2 power, slave ID, baudrate, "
            "RS485 A/B polarity, and signal ground"
        ),
        -1: "command send failed",
        -2: (
            "controller/RS485 response timed out; check Revo2 power, slave ID, "
            "baudrate, RS485 A/B polarity, and signal ground"
        ),
        -3: "response could not be parsed; check RS485 signal integrity and baudrate",
        -4: "API unsupported by this controller generation, or end device type mismatch",
        -5: "blocking hand motion timed out",
    }
    hint = hints.get(error_code, "unknown RealMan SDK error")
    raise RuntimeError(f"{operation} failed ({error_code}): {hint}")


class Revo2ToolModbus:
    """BrainCo Revo2 register transport through the RM tool RS485 connector."""

    def __init__(self, robot, generation, slave_id, baudrate, logger=None):
        self.robot = robot
        self.generation = str(generation).strip().lower()
        self.slave_id = int(slave_id)
        self.baudrate = int(baudrate)
        self.logger = logger
        self.current_tool_rs485_mode = None
        self.current_tool_rs485_baudrate = None

    @classmethod
    def create(cls, robot, generation, slave_id, baudrate, logger=None):
        generation = str(generation).strip().lower()
        if generation == "auto":
            error_code, _ = robot.rm_get_tool_rs485_mode_v4()
            if error_code == 0:
                generation = "gen4"
            elif error_code == -4:
                state = robot.rm_get_tool_rs485_mode()
                raise_for_rm_hand_error(
                    state["return_code"],
                    "rm_get_tool_rs485_mode",
                )
                generation = "gen3"
            else:
                raise_for_rm_hand_error(error_code, "rm_get_tool_rs485_mode_v4")
        if generation not in ("gen3", "gen4"):
            raise ValueError("hand_raw_modbus_generation must be auto, gen3, or gen4")
        return cls(robot, generation, slave_id, baudrate, logger=logger)

    def configure(self):
        if self._tool_port_already_configured():
            self._info(
                f"Revo2 {self.generation} tool port is already Modbus RTU master "
                f"at {self.baudrate} baud; skipping repeated port configuration"
            )
            return

        if self.generation == "gen4":
            ret = self.robot.rm_set_tool_rs485_mode(0, self.baudrate)
            operation = "rm_set_tool_rs485_mode(RTU master)"
        else:
            ret = self.robot.rm_set_modbus_mode(1, self.baudrate, 10)
            operation = "rm_set_modbus_mode(tool RTU master)"
        raise_for_rm_hand_error(ret, operation)
        self._info(
            f"Configured Revo2 {self.generation} tool port as Modbus RTU master "
            f"at {self.baudrate} baud"
        )
        if self.generation == "gen3":
            time.sleep(0.2)

    def describe(self):
        try:
            voltage_ret, voltage_type = self.robot.rm_get_tool_voltage()
            if voltage_ret == 0:
                voltages = {0: "off", 1: "5 V", 2: "12 V", 3: "24 V"}
                self._info(
                    "Tool power output for Revo2: "
                    f"{voltages.get(voltage_type, f'unknown({voltage_type})')}"
                )
            else:
                self._warn(f"Could not read tool power output, code={voltage_ret}")
        except Exception as exc:
            self._warn(f"Could not read tool power output: {exc}")

        if self.generation == "gen4":
            mode_ret, mode = self.robot.rm_get_tool_rs485_mode_v4()
            raise_for_rm_hand_error(mode_ret, "rm_get_tool_rs485_mode_v4")
        else:
            mode = self.robot.rm_get_tool_rs485_mode()
            raise_for_rm_hand_error(mode["return_code"], "rm_get_tool_rs485_mode")
        self._info(
            "Tool RS485 before Revo2 raw-Modbus configuration: "
            f"mode={mode['mode']}, baudrate={mode['baudrate']}"
        )
        self.current_tool_rs485_mode = int(mode.get("mode", -1))
        self.current_tool_rs485_baudrate = int(mode.get("baudrate", 0))

    def _tool_port_already_configured(self):
        if self.current_tool_rs485_baudrate != self.baudrate:
            return False
        if self.generation == "gen4":
            return self.current_tool_rs485_mode == 0
        return self.current_tool_rs485_mode == 1

    def detect_slave_id(self, candidates=REVO2_FACTORY_SLAVE_IDS):
        failures = []
        for candidate in candidates:
            self.slave_id = int(candidate)
            try:
                self.probe()
                return self.slave_id
            except RuntimeError as exc:
                failures.append(f"{candidate}: {exc}")
        raise RuntimeError(
            "No Revo2 responded at factory slave IDs "
            f"{REVO2_FACTORY_SLAVE_IDS}: {'; '.join(failures)}"
        )

    def probe(self):
        hand_type = self.read_holding_register(REVO2_REG_HAND_TYPE)
        hand_names = {1: "right", 2: "left"}
        if hand_type not in hand_names:
            raise RuntimeError(
                f"Unexpected hand type {hand_type} in Revo2 register "
                f"{REVO2_REG_HAND_TYPE}"
            )
        self._info(f"Revo2 FC03 communication OK; detected {hand_names[hand_type]} hand")

    def read_positions(self):
        if self.generation == "gen4":
            params = rm_api.rm_modbus_rtu_read_params_t(
                address=REVO2_REG_ACTUAL_POSITION,
                device=self.slave_id,
                type=1,
                num=6,
            )
            error_code, positions = self.robot.rm_read_modbus_rtu_input_registers(params)
            raise_for_rm_hand_error(
                error_code,
                f"Modbus FC04 read slave={self.slave_id}, address={REVO2_REG_ACTUAL_POSITION}",
            )
            if len(positions) < 6:
                raise RuntimeError(
                    f"Short Modbus response: expected 6 registers, received {len(positions)}"
                )
            return [int(value) & 0xFFFF for value in positions[:6]]

        positions = []
        for offset in range(6):
            address = REVO2_REG_ACTUAL_POSITION + offset
            params = rm_api.rm_peripheral_read_write_params_t(
                port=1,
                address=address,
                device=self.slave_id,
            )
            error_code, value = self.robot.rm_read_input_registers(params)
            raise_for_rm_hand_error(
                error_code,
                f"Modbus FC04 read slave={self.slave_id}, address={address}",
            )
            positions.append(int(value) & 0xFFFF)
        return positions

    def read_holding_register(self, address):
        if self.generation == "gen4":
            params = rm_api.rm_modbus_rtu_read_params_t(
                address=address,
                device=self.slave_id,
                type=1,
                num=1,
            )
            error_code, values = self.robot.rm_read_modbus_rtu_holding_registers(params)
            raise_for_rm_hand_error(
                error_code,
                f"Modbus FC03 read slave={self.slave_id}, address={address}",
            )
            if not values:
                raise RuntimeError(f"Empty Modbus response for holding register {address}")
            return int(values[0]) & 0xFFFF

        params = rm_api.rm_peripheral_read_write_params_t(
            port=1,
            address=address,
            device=self.slave_id,
        )
        error_code, value = self.robot.rm_read_holding_registers(params)
        raise_for_rm_hand_error(
            error_code,
            f"Modbus FC03 read slave={self.slave_id}, address={address}",
        )
        return int(value) & 0xFFFF

    def set_positions(self, positions):
        if len(positions) != 6:
            raise ValueError(f"Expected 6 Revo2 positions, received {len(positions)}")
        values = [max(0, min(1000, int(value))) for value in positions]
        percentages = [(value + 5) // 10 for value in values]
        thumb_flex, thumb_aux, index, middle, ring, pinky = percentages
        registers = [
            (thumb_aux << 8) | thumb_flex,
            (index << 8) | middle,
            (ring << 8) | pinky,
        ]
        self.write_holding_registers(REVO2_REG_FAST_POSITION, registers)

    def write_holding_registers(self, address, registers):
        values = [int(value) & 0xFFFF for value in registers]
        if not values:
            raise ValueError("At least one register must be written")

        if self.generation == "gen4":
            params = rm_api.rm_modbus_rtu_write_params_t(
                address=address,
                device=self.slave_id,
                type=1,
                num=len(values),
                data=values,
            )
            error_code = self.robot.rm_write_modbus_rtu_registers(params)
        else:
            if len(values) == 1:
                params = rm_api.rm_peripheral_read_write_params_t(
                    port=1,
                    address=address,
                    device=self.slave_id,
                )
                error_code = self.robot.rm_write_single_register(params, values[0])
            else:
                params = rm_api.rm_peripheral_read_write_params_t(
                    port=1,
                    address=address,
                    device=self.slave_id,
                    num=len(values),
                )
                error_code = self.robot.rm_write_registers(
                    params,
                    self._registers_to_bytes(values),
                )
        raise_for_rm_hand_error(
            error_code,
            f"Modbus write slave={self.slave_id}, address={address}",
        )

    @staticmethod
    def rm_order_to_revo2_order(positions: Sequence[int]):
        values = [int(value) for value in positions]
        if len(values) != 6:
            raise ValueError(f"Expected 6 RM hand positions, received {len(values)}")
        pinky, ring, middle, index, thumb_flex, thumb_aux = values
        return [thumb_flex, thumb_aux, index, middle, ring, pinky]

    @staticmethod
    def _registers_to_bytes(registers):
        result = []
        for value in registers:
            unsigned = int(value) & 0xFFFF
            result.extend((unsigned >> 8, unsigned & 0xFF))
        return result

    def _info(self, message):
        if self.logger is not None:
            self.logger.info(message)

    def _warn(self, message):
        if self.logger is not None:
            self.logger.warn(message)


class BrainCoCanfdHandClient:
    """BrainCo Revo2 SocketCAN CANFD transport using the official Python SDK."""

    def __init__(
        self,
        sdk_path="",
        iface="can0",
        slave_id=0x7F,
        master_id=1,
        use_builtin_socketcan=True,
        use_durations=True,
        duration_ms=70,
        speed=1000,
        command_timeout_s=0.20,
        verify_on_start=True,
        logger=None,
    ):
        self.sdk_path = str(sdk_path or "").strip()
        self.iface = str(iface or "can0").strip()
        self.slave_id = int(slave_id)
        self.master_id = int(master_id)
        self.use_builtin_socketcan = bool(use_builtin_socketcan)
        self.use_durations = bool(use_durations)
        self.duration_ms = max(1, int(duration_ms))
        self.speed = max(1, int(speed))
        self.command_timeout_s = max(0.02, float(command_timeout_s))
        self.verify_on_start = bool(verify_on_start)
        self.logger = logger

        self.sdk = None
        self.client = None
        self.hw_type = None
        self.cleanup_mode = None
        self.socketcan_close = None
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._loop_worker, daemon=True)
        self.thread.start()

        try:
            self._run(self._connect_async(), timeout=5.0)
        except Exception:
            self.close()
            raise

    @staticmethod
    def parse_int(value, default=0):
        if value is None:
            return int(default)
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return int(default)
            return int(text, 0)
        return int(value)

    @staticmethod
    def rm_order_to_revo2_order(positions: Sequence[int]):
        return Revo2ToolModbus.rm_order_to_revo2_order(positions)

    def set_positions(self, positions):
        values = [max(0, min(1000, int(value))) for value in positions[:6]]
        while len(values) < 6:
            values.append(0)
        self._run(
            self._set_positions_async(values),
            timeout=self.command_timeout_s,
        )

    def close(self):
        if self.loop is None:
            return

        try:
            self._run(self._cleanup_async(), timeout=1.5)
        except Exception as exc:
            self._warn(f"BrainCo CANFD cleanup failed: {exc}")

        loop = self.loop
        self.loop = None
        try:
            loop.call_soon_threadsafe(loop.stop)
        except Exception:
            pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        try:
            loop.close()
        except Exception:
            pass

    def _loop_worker(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _run(self, coro, timeout=None):
        if self.loop is None:
            raise RuntimeError("BrainCo CANFD event loop is closed")
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=timeout)

    async def _connect_async(self):
        self._add_brainco_sdk_path()
        self._validate_socketcan_iface()
        try:
            from common_imports import check_sdk, get_hw_type_name, supports_durations_api

            self.sdk = check_sdk()
        except ModuleNotFoundError:
            try:
                from bc_stark_sdk import main_mod as sdk
            except ImportError as exc:
                raise ModuleNotFoundError(
                    "Could not import BrainCo SDK helpers or bc_stark_sdk. "
                    "Install bc_stark_sdk in this Python environment, or pass "
                    "hand_canfd_sdk_path:=/path/to/brainco-hand-sdk/python"
                ) from exc
            self.sdk = sdk
            get_hw_type_name = self._fallback_hw_type_name
            supports_durations_api = self._fallback_supports_durations_api

        if self.use_builtin_socketcan and hasattr(self.sdk, "init_socketcan_canfd"):
            self.sdk.init_socketcan_canfd(self.iface)
            self.client = self.sdk.init_device_handler(
                self.sdk.StarkProtocolType.CanFd,
                self.master_id,
            )
            self.cleanup_mode = "builtin"
        else:
            if self.use_builtin_socketcan:
                self._warn(
                    "BrainCo SDK has no init_socketcan_canfd(); falling back to "
                    "the Python SocketCAN CANFD callback adapter"
                )
            self._connect_external_socketcan()
            self.client = self.sdk.init_device_handler(
                self.sdk.StarkProtocolType.CanFd,
                self.master_id,
            )
            self.cleanup_mode = "external"

        info = None
        if self.verify_on_start:
            try:
                info = await self.client.get_device_info(self.slave_id)
            except RuntimeError as exc:
                raise RuntimeError(
                    f"BrainCo CANFD get_device_info failed on {self.iface}, "
                    f"slave=0x{self.slave_id:02X}: {exc}. If the error contains "
                    "'Invalid argument' or 'os error 22', configure the SocketCAN "
                    "interface for CANFD first: sudo ip link set "
                    f"{self.iface} down; sudo ip link set {self.iface} type can "
                    "bitrate 1000000 dbitrate 5000000 fd on; sudo ip link set "
                    f"{self.iface} up; then verify `ip -details link show {self.iface}` "
                    "shows mtu 72 and fd on."
                ) from exc
            self.hw_type = getattr(info, "hardware_type", None)
            description = getattr(info, "description", None) or str(info)
            self._info(
                f"BrainCo Revo2 CANFD device OK on {self.iface}: "
                f"slave=0x{self.slave_id:02X}, {description}"
            )
            try:
                baudrate = await self.client.get_canfd_baudrate(self.slave_id)
                self._info(f"BrainCo Revo2 CANFD baudrate: {baudrate}")
            except Exception as exc:
                self._warn(f"Could not read BrainCo CANFD baudrate: {exc}")
        else:
            self.hw_type = None

        try:
            await self.client.set_finger_unit_mode(
                self.slave_id,
                self.sdk.FingerUnitMode.Normalized,
            )
            self._info("BrainCo Revo2 finger unit mode set to Normalized")
        except Exception as exc:
            self._warn(f"Could not set BrainCo Revo2 normalized unit mode: {exc}")

        if self.use_durations and self.hw_type is not None:
            if not supports_durations_api(self.hw_type):
                self._warn(
                    "Connected BrainCo hand may not support duration commands; "
                    "falling back to speed/position commands"
                )
                self.use_durations = False
            elif info is not None:
                self._info(
                    "BrainCo Revo2 hardware type: "
                    f"{get_hw_type_name(self.hw_type)}"
                )

    @staticmethod
    def _fallback_hw_type_name(hw_type):
        if hw_type is None:
            return "unknown"
        return str(hw_type)

    @staticmethod
    def _fallback_supports_durations_api(hw_type):
        if hw_type is None:
            return True
        name = str(hw_type).lower()
        return "revo2" in name or "advanced" in name

    def _validate_socketcan_iface(self):
        iface_dir = Path("/sys/class/net") / self.iface
        if not iface_dir.exists():
            return

        mtu_path = iface_dir / "mtu"
        try:
            mtu = int(mtu_path.read_text(encoding="ascii").strip())
        except Exception:
            mtu = 0
        if mtu and mtu < 72:
            raise RuntimeError(
                f"SocketCAN interface {self.iface} is not in CANFD mode "
                f"(mtu={mtu}, expected 72). Run: sudo ip link set {self.iface} down; "
                f"sudo ip link set {self.iface} type can bitrate 1000000 "
                f"dbitrate 5000000 fd on; sudo ip link set {self.iface} up"
            )

    def _connect_external_socketcan(self):
        from common_socketcan import (
            socketcan_open,
            socketcan_close,
            socketcan_send_message,
            socketcan_receive_canfd_filtered,
        )

        socketcan_open(self.iface, canfd=True)
        self.socketcan_close = socketcan_close

        def canfd_send(_slave_id, can_id, data):
            if not socketcan_send_message(can_id, bytes(data)):
                raise RuntimeError(f"SocketCAN CANFD send failed: can_id=0x{can_id:X}")

        def canfd_read(_slave_id, expected_can_id, expected_frames):
            result = socketcan_receive_canfd_filtered(expected_can_id, expected_frames)
            if result is None:
                return 0, bytes([])
            can_id, data, _frame_count = result
            return can_id, bytes(data)

        self.sdk.set_can_tx_callback(canfd_send)
        self.sdk.set_can_rx_callback(canfd_read)

    async def _set_positions_async(self, positions):
        if self.use_durations and hasattr(
            self.client,
            "set_finger_positions_and_durations",
        ):
            durations = [self.duration_ms] * 6
            await self.client.set_finger_positions_and_durations(
                self.slave_id,
                positions,
                durations,
            )
            return

        if hasattr(self.client, "set_finger_positions_and_speeds"):
            speeds = [self.speed] * 6
            await self.client.set_finger_positions_and_speeds(
                self.slave_id,
                positions,
                speeds,
            )
            return

        await self.client.set_finger_positions(self.slave_id, positions)

    async def _cleanup_async(self):
        if self.client is None and self.cleanup_mode is None:
            return

        if self.cleanup_mode == "builtin" and self.sdk is not None:
            if hasattr(self.sdk, "close_device_handler") and self.client is not None:
                await self.sdk.close_device_handler(self.client)
            elif hasattr(self.sdk, "close_socketcan"):
                self.sdk.close_socketcan()
        elif self.cleanup_mode == "external" and self.socketcan_close is not None:
            self.socketcan_close()

        self.client = None
        self.cleanup_mode = None

    def _add_brainco_sdk_path(self):
        paths = []
        if self.sdk_path:
            paths.append(Path(self.sdk_path).expanduser())
        for env_name in ("BRAINCO_HAND_SDK_PATH", "STARK_SDK_PATH"):
            env_path = os.getenv(env_name, "").strip()
            if env_path:
                paths.append(Path(env_path).expanduser())

        script_dir = Path(__file__).resolve().parent
        paths.extend(
            [
                script_dir / "brainco-hand-sdk" / "python",
                script_dir.parent / "brainco-hand-sdk" / "python",
                Path.cwd() / "brainco-hand-sdk" / "python",
                Path.home() / "ws" / "brainco-hand-sdk" / "python",
                Path.home() / "ws" / "rm_brainco_orbbec_demo" / "brainco-hand-sdk" / "python",
                Path("/home/jetson/ws/brainco-hand-sdk/python"),
                Path("/home/jetson/ws/rm_brainco_orbbec_demo/brainco-hand-sdk/python"),
            ]
        )

        for path in paths:
            try:
                resolved = path.resolve()
            except Exception:
                resolved = path
            if resolved.exists():
                text = str(resolved)
                if text not in sys.path:
                    sys.path.insert(0, text)
                return

        if self.sdk_path:
            text = str(Path(self.sdk_path).expanduser())
            if text not in sys.path:
                sys.path.insert(0, text)

    def _info(self, message):
        if self.logger is not None:
            self.logger.info(message)

    def _warn(self, message):
        if self.logger is not None:
            self.logger.warn(message)


class BrainCoRs485HandClient:
    """BrainCo Revo2 direct RS485/Modbus transport using the official Python SDK."""

    def __init__(
        self,
        sdk_path="",
        port="auto",
        slave_id=127,
        baudrate=460800,
        quick_detect=True,
        use_durations=True,
        duration_ms=60,
        speed=1000,
        command_timeout_s=0.20,
        verify_on_start=True,
        logger=None,
    ):
        self.sdk_path = str(sdk_path or "").strip()
        self.port = str(port or "auto").strip()
        self.slave_id = int(slave_id)
        self.baudrate = int(baudrate)
        self.quick_detect = bool(quick_detect)
        self.use_durations = bool(use_durations)
        self.duration_ms = max(1, int(duration_ms))
        self.speed = max(1, int(speed))
        self.command_timeout_s = max(0.02, float(command_timeout_s))
        self.verify_on_start = bool(verify_on_start)
        self.logger = logger

        self.sdk = None
        self.client = None
        self.hw_type = None
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._loop_worker, daemon=True)
        self.thread.start()

        try:
            self._run(self._connect_async(), timeout=8.0)
        except Exception:
            self.close()
            raise

    @staticmethod
    def rm_order_to_revo2_order(positions: Sequence[int]):
        return Revo2ToolModbus.rm_order_to_revo2_order(positions)

    def set_positions(self, positions):
        values = [max(0, min(1000, int(value))) for value in positions[:6]]
        while len(values) < 6:
            values.append(0)
        self._run(
            self._set_positions_async(values),
            timeout=self.command_timeout_s,
        )

    def close(self):
        if self.loop is None:
            return

        loop = self.loop
        self.loop = None
        if not loop.is_closed():
            try:
                self._run_on_loop(loop, self._cleanup_async(), timeout=1.5)
            except Exception as exc:
                self._warn(f"BrainCo RS485 cleanup failed: {exc}")
        try:
            loop.call_soon_threadsafe(loop.stop)
        except Exception:
            pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        try:
            loop.close()
        except Exception:
            pass

    def _loop_worker(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _run(self, coro, timeout=None):
        if self.loop is None:
            raise RuntimeError("BrainCo RS485 event loop is closed")
        return self._run_on_loop(self.loop, coro, timeout=timeout)

    @staticmethod
    def _run_on_loop(loop, coro, timeout=None):
        if loop.is_closed():
            raise RuntimeError("BrainCo RS485 event loop is closed")
        future = asyncio.run_coroutine_threadsafe(coro, loop)
        return future.result(timeout=timeout)

    async def _connect_async(self):
        self._add_brainco_sdk_path()
        helpers = self._import_brainco_helpers()
        self.sdk = helpers["sdk"]
        get_hw_type_name = helpers["get_hw_type_name"]
        supports_durations_api = helpers["supports_durations_api"]

        port = self.port
        baudrate = self.baudrate
        slave_id = self.slave_id
        if port.lower() in ("", "auto") or slave_id <= 0:
            if not hasattr(self.sdk, "auto_detect_modbus_revo2"):
                raise RuntimeError(
                    "BrainCo SDK does not expose auto_detect_modbus_revo2; "
                    "set hand_rs485_port, hand_rs485_slave_id, and hand_rs485_baudrate explicitly"
                )
            protocol, detected_port, detected_baudrate, detected_slave_id = (
                await self.sdk.auto_detect_modbus_revo2(
                    None if port.lower() in ("", "auto") else port,
                    self.quick_detect,
                )
            )
            if protocol != self.sdk.StarkProtocolType.Modbus:
                raise RuntimeError(f"Detected BrainCo device is not Modbus/RS485: {protocol}")
            port = detected_port
            baudrate = self._baudrate_to_int(detected_baudrate)
            if slave_id <= 0:
                slave_id = int(detected_slave_id)

        self.port = str(port)
        self.baudrate = int(baudrate)
        self.slave_id = int(slave_id)
        self.client = await self._modbus_open(self.port, self.baudrate, helpers)

        if hasattr(self.client, "set_hardware_type") and hasattr(self.sdk, "StarkHardwareType"):
            try:
                await self.client.set_hardware_type(
                    self.slave_id,
                    self.sdk.StarkHardwareType.Revo2Basic,
                )
            except Exception as exc:
                self._warn(f"Could not preset BrainCo Revo2 hardware type: {exc}")

        info = None
        if self.verify_on_start:
            try:
                info = await self.client.get_device_info(self.slave_id)
            except RuntimeError as exc:
                raise RuntimeError(
                    f"BrainCo RS485 get_device_info failed on {self.port}, "
                    f"slave={self.slave_id}, baudrate={self.baudrate}: {exc}. "
                    "Check that the Revo2 RS485 A/B/GND lines are connected to "
                    "this independent Jetson USB-RS485 adapter, not only to the "
                    "RealMan tool RS485 port; also check slave ID, baudrate, "
                    "A/B polarity, signal ground, and 24V power."
                ) from exc
            self.hw_type = getattr(info, "hardware_type", None)
            description = getattr(info, "description", None) or str(info)
            self._info(
                f"BrainCo Revo2 RS485 device OK on {self.port}: "
                f"slave={self.slave_id}, baudrate={self.baudrate}, {description}"
            )
        else:
            self.hw_type = None

        try:
            await self.client.set_finger_unit_mode(
                self.slave_id,
                self.sdk.FingerUnitMode.Normalized,
            )
            self._info("BrainCo Revo2 RS485 finger unit mode set to Normalized")
        except Exception as exc:
            self._warn(f"Could not set BrainCo Revo2 RS485 normalized unit mode: {exc}")

        if self.use_durations and self.hw_type is not None:
            if not supports_durations_api(self.hw_type):
                self._warn(
                    "Connected BrainCo RS485 hand may not support duration commands; "
                    "falling back to speed/position commands"
                )
                self.use_durations = False
            elif info is not None:
                self._info(
                    "BrainCo RS485 hardware type: "
                    f"{get_hw_type_name(self.hw_type)}"
                )

    def _import_brainco_helpers(self):
        try:
            from common_imports import (
                check_sdk,
                get_hw_type_name,
                int_to_baudrate,
                modbus_open,
                supports_durations_api,
            )

            return {
                "sdk": check_sdk(),
                "get_hw_type_name": get_hw_type_name,
                "supports_durations_api": supports_durations_api,
                "int_to_baudrate": int_to_baudrate,
                "modbus_open": modbus_open,
            }
        except ModuleNotFoundError:
            try:
                from bc_stark_sdk import main_mod as sdk
            except ImportError as exc:
                raise ModuleNotFoundError(
                    "Could not import BrainCo SDK helpers or bc_stark_sdk. "
                    "Install bc_stark_sdk in this Python environment, or pass "
                    "hand_rs485_sdk_path:=/path/to/brainco-hand-sdk/python"
                ) from exc
            return {
                "sdk": sdk,
                "get_hw_type_name": BrainCoCanfdHandClient._fallback_hw_type_name,
                "supports_durations_api": BrainCoCanfdHandClient._fallback_supports_durations_api,
                "int_to_baudrate": self._fallback_int_to_baudrate,
                "modbus_open": None,
            }

    async def _modbus_open(self, port, baudrate, helpers):
        modbus_open = helpers.get("modbus_open")
        if modbus_open is not None:
            return await modbus_open(port, baudrate)
        baudrate_enum = self._fallback_int_to_baudrate(baudrate)
        return await self.sdk.modbus_open(port, baudrate_enum)

    def _fallback_int_to_baudrate(self, value):
        if hasattr(self.sdk.Baudrate, "from_int"):
            return self.sdk.Baudrate.from_int(int(value))
        baudrate_map = {
            115200: self.sdk.Baudrate.Baud115200,
            57600: self.sdk.Baudrate.Baud57600,
            19200: self.sdk.Baudrate.Baud19200,
            460800: self.sdk.Baudrate.Baud460800,
            1000000: self.sdk.Baudrate.Baud1Mbps,
            2000000: self.sdk.Baudrate.Baud2Mbps,
            3000000: self.sdk.Baudrate.Baud3Mbps,
            5000000: self.sdk.Baudrate.Baud5Mbps,
        }
        return baudrate_map.get(int(value), self.sdk.Baudrate.Baud460800)

    def _baudrate_to_int(self, baudrate):
        if isinstance(baudrate, int):
            return int(baudrate)
        baudrate_pairs = (
            ("Baud115200", 115200),
            ("Baud57600", 57600),
            ("Baud19200", 19200),
            ("Baud460800", 460800),
            ("Baud1Mbps", 1000000),
            ("Baud2Mbps", 2000000),
            ("Baud3Mbps", 3000000),
            ("Baud5Mbps", 5000000),
        )
        for attr_name, value in baudrate_pairs:
            enum_value = getattr(self.sdk.Baudrate, attr_name, None)
            if enum_value is None:
                continue
            try:
                if baudrate == enum_value:
                    return value
            except Exception:
                pass
        text = str(baudrate)
        for candidate in (115200, 57600, 19200, 460800, 1000000, 2000000, 3000000, 5000000):
            if str(candidate) in text:
                return candidate
        for attr_name, value in baudrate_pairs:
            if attr_name.lower() in text.lower():
                return value
        return self.baudrate

    async def _set_positions_async(self, positions):
        if self.use_durations and hasattr(
            self.client,
            "set_finger_positions_and_durations",
        ):
            durations = [self.duration_ms] * 6
            await self.client.set_finger_positions_and_durations(
                self.slave_id,
                positions,
                durations,
            )
            return

        if hasattr(self.client, "set_finger_positions_and_speeds"):
            speeds = [self.speed] * 6
            await self.client.set_finger_positions_and_speeds(
                self.slave_id,
                positions,
                speeds,
            )
            return

        await self.client.set_finger_positions(self.slave_id, positions)

    async def _cleanup_async(self):
        if self.client is None or self.sdk is None:
            return
        close_result = self.sdk.modbus_close(self.client)
        if asyncio.iscoroutine(close_result):
            await close_result
        self.client = None

    def _add_brainco_sdk_path(self):
        paths = []
        if self.sdk_path:
            paths.append(Path(self.sdk_path).expanduser())
        for env_name in ("BRAINCO_HAND_SDK_PATH", "STARK_SDK_PATH"):
            env_path = os.getenv(env_name, "").strip()
            if env_path:
                paths.append(Path(env_path).expanduser())

        script_dir = Path(__file__).resolve().parent
        paths.extend(
            [
                script_dir / "brainco-hand-sdk" / "python",
                script_dir.parent / "brainco-hand-sdk" / "python",
                Path.cwd() / "brainco-hand-sdk" / "python",
                Path.home() / "ws" / "brainco-hand-sdk" / "python",
                Path.home() / "ws" / "brainco-hand-sdk-main" / "python",
                Path.home() / "ws" / "rm_brainco_orbbec_demo" / "brainco-hand-sdk" / "python",
                Path("/home/jetson/ws/brainco-hand-sdk/python"),
                Path("/home/jetson/ws/brainco-hand-sdk-main/python"),
                Path("/home/jetson/ws/rm_brainco_orbbec_demo/brainco-hand-sdk/python"),
            ]
        )

        for path in paths:
            try:
                resolved = path.resolve()
            except Exception:
                resolved = path
            if resolved.exists():
                text = str(resolved)
                if text not in sys.path:
                    sys.path.insert(0, text)
                return

        if self.sdk_path:
            text = str(Path(self.sdk_path).expanduser())
            if text not in sys.path:
                sys.path.insert(0, text)

    def _info(self, message):
        if self.logger is not None:
            self.logger.info(message)

    def _warn(self, message):
        if self.logger is not None:
            self.logger.warn(message)


class RMHandFingerFollower:
    RM_HAND_LABELS = (
        "pinky",
        "ring",
        "middle",
        "index",
        "thumb_bend",
        "thumb_rotate",
    )

    def __init__(self, node, palm_follower):
        self.node = node
        self.logger = node.get_logger()
        self.palm_follower = palm_follower
        self.enabled = bool(node.declare_parameter("hand_control_enabled", False).value)
        self.use_dedicated_connection = bool(
            node.declare_parameter("hand_use_dedicated_connection", False).value
        )
        self.robot_ip = str(
            node.declare_parameter("hand_robot_ip", palm_follower.robot_ip).value
        )
        self.robot_port = int(
            node.declare_parameter("hand_robot_port", palm_follower.robot_port).value
        )
        self.robot_level = int(
            node.declare_parameter("hand_robot_level", palm_follower.robot_level).value
        )
        self.thread_mode = int(
            node.declare_parameter("hand_robot_thread_mode", palm_follower.thread_mode).value
        )
        self.robot = None
        self.sdk_lock = threading.RLock()
        self.owns_robot_connection = False
        if not self.use_dedicated_connection:
            self.robot = palm_follower.robot
            self.sdk_lock = palm_follower.sdk_lock
        self.position_min = int(node.declare_parameter("hand_position_min", 0).value)
        self.position_max = int(node.declare_parameter("hand_position_max", 1000).value)
        self.thumb_rotate_scale = float(
            node.declare_parameter("hand_thumb_rotate_scale", 1.0).value
        )
        self.smoothing = float(
            node.declare_parameter(
                "hand_smoothing",
                persistent_default(node, "hand_smoothing", 0.90),
            ).value
        )
        self.deadband = int(
            node.declare_parameter(
                "hand_deadband",
                persistent_default(node, "hand_deadband", 1),
            ).value
        )
        self.bend_deadzone = float(
            node.declare_parameter(
                "hand_bend_deadzone",
                persistent_default(node, "hand_bend_deadzone", 0.10),
            ).value
        )
        self.bend_smoothing = float(
            node.declare_parameter(
                "hand_bend_smoothing",
                persistent_default(node, "hand_bend_smoothing", 0.65),
            ).value
        )
        self.bend_max_delta = float(
            node.declare_parameter(
                "hand_bend_max_delta",
                persistent_default(node, "hand_bend_max_delta", 0.60),
            ).value
        )
        self.reject_single_finger_spikes = bool(
            node.declare_parameter("hand_reject_single_finger_spikes", True).value
        )
        self.single_finger_spike_delta = float(
            node.declare_parameter("hand_single_finger_spike_delta", 0.22).value
        )
        self.single_finger_spike_peer_max = float(
            node.declare_parameter("hand_single_finger_spike_peer_max", 0.22).value
        )
        self.spike_confirm_frames = int(
            node.declare_parameter("hand_spike_confirm_frames", 4).value
        )
        self.require_3d_bends_for_control = bool(
            node.declare_parameter("hand_require_3d_bends_for_control", True).value
        )
        self.reject_multi_finger_spikes = bool(
            node.declare_parameter("hand_reject_multi_finger_spikes", True).value
        )
        self.multi_finger_spike_delta = float(
            node.declare_parameter("hand_multi_finger_spike_delta", 0.30).value
        )
        self.multi_finger_spike_min_count = int(
            node.declare_parameter("hand_multi_finger_spike_min_count", 2).value
        )
        self.multi_finger_spike_confirm_frames = int(
            node.declare_parameter("hand_multi_finger_spike_confirm_frames", 6).value
        )
        self.closure_confirm_frames = int(
            node.declare_parameter("hand_closure_confirm_frames", 5).value
        )
        self.closure_confirm_delta = float(
            node.declare_parameter("hand_closure_confirm_delta", 0.25).value
        )
        self.closure_confirm_value = float(
            node.declare_parameter("hand_closure_confirm_value", 0.45).value
        )
        self.closure_confirm_min_fingers = int(
            node.declare_parameter("hand_closure_confirm_min_fingers", 2).value
        )
        self.send_hz = float(
            node.declare_parameter(
                "hand_send_hz",
                persistent_default(node, "hand_send_hz", 35.0),
            ).value
        )
        self.wait_for_first_arm_command = bool(
            node.declare_parameter("hand_wait_for_first_arm_command", True).value
        )
        self.wait_for_first_arm_command_timeout_s = float(
            node.declare_parameter("hand_wait_for_first_arm_command_timeout_s", 1.0).value
        )
        self.pause_while_arm_sdk_busy = bool(
            node.declare_parameter("hand_pause_while_arm_sdk_busy", True).value
        )
        self.pause_while_arm_motion_pending = bool(
            node.declare_parameter("hand_pause_while_arm_motion_pending", False).value
        )
        self.sdk_lock_timeout_s = float(
            node.declare_parameter("hand_sdk_lock_timeout_s", 0.25).value
        )
        self.block = bool(node.declare_parameter("hand_block", False).value)
        self.use_follow_pos = bool(
            node.declare_parameter("hand_use_follow_pos", True).value
        )
        self.transport = str(
            node.declare_parameter("hand_transport", "realman-hand").value
        ).strip().lower()
        if self.transport in ("brainco-socketcanfd", "socketcanfd", "canfd"):
            self.transport = "brainco-canfd"
        if self.transport in ("brainco-modbus", "brainco-rs-485", "stark-rs485"):
            self.transport = "brainco-rs485"
        if self.transport not in (
            "realman-hand",
            "raw-modbus",
            "brainco-canfd",
            "brainco-rs485",
        ):
            raise ValueError(
                "hand_transport must be 'realman-hand', 'raw-modbus', "
                "'brainco-canfd', or 'brainco-rs485'"
            )
        if (
            self.enabled
            and self.transport in ("brainco-canfd", "brainco-rs485")
            and self.pause_while_arm_sdk_busy
        ):
            self.pause_while_arm_sdk_busy = False
            self.logger.info(
                "Disabling hand_pause_while_arm_sdk_busy for BrainCo direct hand "
                "transport because it does not share the RM_API tool-RS485 channel"
            )
        self.force_shared_connection_for_raw_modbus = bool(
            node.declare_parameter(
                "hand_force_shared_connection_for_raw_modbus",
                True,
            ).value
        )
        if (
            self.enabled
            and self.transport == "raw-modbus"
            and getattr(palm_follower, "enabled", False)
            and self.use_dedicated_connection
            and self.force_shared_connection_for_raw_modbus
        ):
            self.use_dedicated_connection = False
            self.robot = palm_follower.robot
            self.sdk_lock = palm_follower.sdk_lock
            self.owns_robot_connection = False
            self.logger.warn(
                "Overriding hand_use_dedicated_connection:=true because Revo2 "
                "raw-Modbus and RM65-B arm motion share the same controller/tool "
                "RS485 path. Hand and arm commands will use one serialized RM_API "
                "connection."
            )
        self.raw_modbus_generation = str(
            node.declare_parameter("hand_raw_modbus_generation", "auto").value
        ).strip().lower()
        self.raw_modbus_slave_id = int(
            node.declare_parameter("hand_raw_modbus_slave_id", 0).value
        )
        self.raw_modbus_baudrate = int(
            node.declare_parameter("hand_raw_modbus_baudrate", 460800).value
        )
        self.raw_modbus_force_arm_step_mode = bool(
            node.declare_parameter("hand_raw_modbus_force_arm_step_mode", True).value
        )
        self.raw_modbus_configure_tool_port = bool(
            node.declare_parameter("hand_raw_modbus_configure_tool_port", True).value
        )
        self.raw_modbus_probe_on_start = bool(
            node.declare_parameter("hand_raw_modbus_probe_on_start", True).value
        )
        self.raw_modbus_read_position_on_start = bool(
            node.declare_parameter("hand_raw_modbus_read_position_on_start", True).value
        )
        self.canfd_sdk_path = str(
            node.declare_parameter("hand_canfd_sdk_path", "").value
        ).strip()
        self.canfd_iface = str(
            node.declare_parameter("hand_canfd_iface", "can0").value
        ).strip()
        self.canfd_slave_id = BrainCoCanfdHandClient.parse_int(
            node.declare_parameter("hand_canfd_slave_id", 127).value,
            0x7F,
        )
        self.canfd_master_id = BrainCoCanfdHandClient.parse_int(
            node.declare_parameter("hand_canfd_master_id", 1).value,
            1,
        )
        self.canfd_use_builtin_socketcan = bool(
            node.declare_parameter("hand_canfd_use_builtin_socketcan", True).value
        )
        self.canfd_use_durations = bool(
            node.declare_parameter("hand_canfd_use_durations", True).value
        )
        self.canfd_duration_ms = int(
            node.declare_parameter("hand_canfd_duration_ms", 70).value
        )
        self.canfd_speed = int(node.declare_parameter("hand_canfd_speed", 1000).value)
        self.canfd_command_timeout_s = float(
            node.declare_parameter("hand_canfd_command_timeout_s", 0.20).value
        )
        self.canfd_verify_on_start = bool(
            node.declare_parameter("hand_canfd_verify_on_start", True).value
        )
        self.rs485_sdk_path = str(
            node.declare_parameter("hand_rs485_sdk_path", "").value
        ).strip()
        self.rs485_port = str(
            node.declare_parameter("hand_rs485_port", "auto").value
        ).strip()
        self.rs485_slave_id = BrainCoCanfdHandClient.parse_int(
            node.declare_parameter("hand_rs485_slave_id", 127).value,
            127,
        )
        self.rs485_baudrate = int(
            node.declare_parameter("hand_rs485_baudrate", 460800).value
        )
        self.rs485_quick_detect = bool(
            node.declare_parameter("hand_rs485_quick_detect", True).value
        )
        self.rs485_use_durations = bool(
            node.declare_parameter("hand_rs485_use_durations", True).value
        )
        self.rs485_duration_ms = int(
            node.declare_parameter("hand_rs485_duration_ms", 60).value
        )
        self.rs485_speed = int(node.declare_parameter("hand_rs485_speed", 1000).value)
        self.rs485_command_timeout_s = float(
            node.declare_parameter("hand_rs485_command_timeout_s", 0.20).value
        )
        self.rs485_verify_on_start = bool(
            node.declare_parameter("hand_rs485_verify_on_start", True).value
        )
        self._force_arm_step_mode_for_raw_modbus_if_needed()
        self.open_on_start = bool(
            node.declare_parameter("hand_open_on_start", False).value
        )
        self.open_on_home = bool(node.declare_parameter("hand_open_on_home", False).value)
        self.open_on_exit = bool(node.declare_parameter("hand_open_on_exit", True).value)
        self.open_on_lost = bool(node.declare_parameter("hand_open_on_lost", False).value)
        self.lost_open_delay = float(
            node.declare_parameter("hand_lost_open_delay_s", 1.0).value
        )
        self.configure_speed_force = bool(
            node.declare_parameter("hand_configure_speed_force", False).value
        )
        self.speed = int(node.declare_parameter("hand_speed", 500).value)
        self.force = int(node.declare_parameter("hand_force", 500).value)
        self.configure_tool_voltage = bool(
            node.declare_parameter("hand_configure_tool_voltage", True).value
        )
        self.tool_voltage_type = int(
            node.declare_parameter("hand_tool_voltage_type", 3).value
        )
        self.configure_tool_rs485 = bool(
            node.declare_parameter("hand_configure_tool_rs485", False).value
        )
        self.tool_rs485_mode = int(node.declare_parameter("hand_tool_rs485_mode", 1).value)
        self.tool_rs485_baudrate = int(
            node.declare_parameter("hand_tool_rs485_baudrate", 115200).value
        )
        self.configure_rm_plus_mode = bool(
            node.declare_parameter("hand_configure_rm_plus_mode", False).value
        )
        self.rm_plus_mode = int(node.declare_parameter("hand_rm_plus_mode", 115200).value)

        if self.position_max < self.position_min:
            raise ValueError("hand_position_max must be greater than hand_position_min")
        self.smoothing = max(0.0, min(1.0, self.smoothing))
        self.bend_deadzone = max(0.0, min(1.0, self.bend_deadzone))
        self.bend_smoothing = max(0.0, min(1.0, self.bend_smoothing))
        self.bend_max_delta = max(0.0, min(1.0, self.bend_max_delta))
        self.spike_confirm_frames = max(1, self.spike_confirm_frames)
        self.multi_finger_spike_delta = max(0.0, min(1.0, self.multi_finger_spike_delta))
        self.multi_finger_spike_min_count = max(2, self.multi_finger_spike_min_count)
        self.multi_finger_spike_confirm_frames = max(
            1,
            self.multi_finger_spike_confirm_frames,
        )
        self.closure_confirm_frames = max(1, self.closure_confirm_frames)
        self.closure_confirm_delta = max(0.0, min(1.0, self.closure_confirm_delta))
        self.closure_confirm_value = max(0.0, min(1.0, self.closure_confirm_value))
        self.closure_confirm_min_fingers = max(1, self.closure_confirm_min_fingers)
        self.send_interval_s = 1.0 / max(self.send_hz, 1.0)

        self.target_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pending_positions = None
        self.smoothed_positions = None
        self.filtered_bends = None
        self.spike_candidate_counts = {name: 0 for name in self.RM_HAND_LABELS}
        self.multi_finger_spike_frames = 0
        self.closure_candidate_frames = 0
        self.last_sent_positions = None
        self.last_send_time = 0.0
        self.last_seen_time = 0.0
        self.last_unreliable_bend_log_time = 0.0
        self.lost_open_sent = False
        self.worker = None
        self.revo2_modbus = None
        self.brainco_canfd = None
        self.brainco_rs485 = None
        self.last_sdk_busy_log_time = 0.0
        self.last_arm_gate_log_time = 0.0
        self.sdk_busy_count = 0
        self.sdk_busy_log_interval_s = getattr(
            palm_follower,
            "sdk_busy_log_interval_s",
            1.0,
        )

        if not self.enabled:
            self.logger.warn(
                "Dexterous hand control is disabled. Set hand_control_enabled:=true "
                "to send dexterous hand commands."
            )
            return

        if self.use_dedicated_connection:
            if self.transport in ("brainco-canfd", "brainco-rs485"):
                self.use_dedicated_connection = False
                self.robot = palm_follower.robot
                self.sdk_lock = palm_follower.sdk_lock
                self.owns_robot_connection = False
                self.logger.info(
                    "Ignoring hand_use_dedicated_connection for BrainCo direct hand "
                    "transport; the hand uses its own adapter directly"
                )
            else:
                try:
                    self._connect_hand_robot()
                except Exception as exc:
                    self.logger.warn(
                        f"Dedicated RM hand connection failed, falling back to shared "
                        f"connection: {exc}"
                    )
                    self.robot = palm_follower.robot
                    self.sdk_lock = palm_follower.sdk_lock
                    self.owns_robot_connection = False

        if self.robot is None and self.transport not in ("brainco-canfd", "brainco-rs485"):
            self.enabled = False
            self.logger.warn(
                "Dexterous hand control needs the RM65-B SDK connection. "
                "Set robot_motion_enabled:=true or hand_use_dedicated_connection:=true "
                "before enabling hand_control_enabled."
            )
            return

        self._configure_if_requested()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        if self.open_on_start:
            self.open_hand("startup")

        command_name = self._command_name()
        self.logger.info(
            f"Dexterous hand follower started with {command_name}; "
            f"order={self.RM_HAND_LABELS}, range=[{self.position_min}, {self.position_max}]"
        )

    def _force_arm_step_mode_for_raw_modbus_if_needed(self):
        palm_follower = self.palm_follower
        if not self.enabled:
            return
        if self.transport != "raw-modbus":
            return
        if not self.raw_modbus_force_arm_step_mode:
            return
        if palm_follower is None or not getattr(palm_follower, "enabled", False):
            return
        if getattr(palm_follower, "control_protocol", None) != "rs485":
            return
        current_mode = getattr(palm_follower, "rs485_effective_motion_mode", None)
        if getattr(palm_follower, "rs485_step_fallback_to_movel", False):
            palm_follower.rs485_step_fallback_to_movel = False
            self.logger.warn(
                "Disabling arm_rs485_step_fallback_to_movel because Revo2 "
                "raw-Modbus hand control is enabled. rm_movel/rm_movej_p planner "
                "calls can stall after tool Modbus configuration."
            )
        if current_mode == "step":
            palm_follower.command_interval_s = max(
                palm_follower.command_interval_s,
                palm_follower._min_command_interval_s(),
            )
            return

        palm_follower.rs485_effective_motion_mode = "step"
        palm_follower.command_interval_s = max(
            palm_follower.command_interval_s,
            palm_follower._min_command_interval_s(),
        )
        self.logger.warn(
            "Switching RM65-B RS485 effective motion mode from "
            f"{current_mode} to step because Revo2 raw-Modbus hand control is "
            "enabled. This avoids rm_movej_p/rm_movel planner calls stalling "
            "after the tool Modbus port is configured."
        )

    def update_from_bends(self, bends, source=None):
        if not self.enabled or not bends:
            return None

        now = time.monotonic()
        if self.require_3d_bends_for_control and not self._reliable_bend_source(source):
            if now - self.last_unreliable_bend_log_time > 1.0:
                self.logger.warn(
                    "Skipping hand command because finger bends are not from reliable "
                    f"3D landmarks (source={source})"
                )
                self.last_unreliable_bend_log_time = now
            return self.last_sent_positions

        if not self._arm_gate_allows_hand_command(now):
            return self.last_sent_positions

        positions = self._positions_from_bends(bends)
        with self.target_lock:
            self.pending_positions = positions
            self.last_seen_time = now
            self.lost_open_sent = False
        return positions

    def _arm_gate_allows_hand_command(self, now):
        palm_follower = self.palm_follower
        if palm_follower is None or not getattr(palm_follower, "enabled", False):
            return True

        if (
            self.wait_for_first_arm_command
            and not getattr(palm_follower, "first_tracking_motion_ok", False)
        ):
            tracking_started_at = getattr(palm_follower, "tracking_cycle_started_at", 0.0)
            wait_elapsed_s = now - tracking_started_at if tracking_started_at > 0.0 else 0.0
            wait_timed_out = (
                self.wait_for_first_arm_command_timeout_s >= 0.0
                and wait_elapsed_s >= self.wait_for_first_arm_command_timeout_s
            )
            if not wait_timed_out or palm_follower.has_pending_or_active_motion():
                self._log_arm_gate(
                    now,
                    "Holding hand command until the first RM65-B tracking command "
                    "succeeds or the arm stays idle past the startup hand gate timeout",
                )
                return False

        if (
            self.pause_while_arm_motion_pending
            and callable(getattr(palm_follower, "has_pending_or_active_motion", None))
            and palm_follower.has_pending_or_active_motion()
        ):
            stage = getattr(palm_follower, "motion_stage", None)
            self._log_arm_gate(
                now,
                f"Holding hand command while RM65-B motion is pending/active "
                f"(stage={stage})",
            )
            return False

        if self.pause_while_arm_sdk_busy and getattr(palm_follower, "motion_inflight", False):
            stage = getattr(palm_follower, "motion_stage", None)
            self._log_arm_gate(
                now,
                f"Holding hand command while RM65-B SDK is busy (stage={stage})",
            )
            return False

        return True

    def _log_arm_gate(self, now, message):
        if now - self.last_arm_gate_log_time < 2.0:
            return
        self.logger.warn(message)
        self.last_arm_gate_log_time = now

    def notify_hand_lost(self):
        if not self.enabled or not self.open_on_lost:
            return

        now = time.monotonic()
        with self.target_lock:
            last_seen_time = self.last_seen_time
            lost_open_sent = self.lost_open_sent

        if last_seen_time <= 0.0 or lost_open_sent:
            return
        if now - last_seen_time < self.lost_open_delay:
            return

        self.open_hand("hand_lost")
        with self.target_lock:
            self.lost_open_sent = True

    def open_hand(self, reason="manual"):
        if not self.enabled:
            return
        positions = [self.position_min] * 6
        with self.target_lock:
            self.pending_positions = positions
        self.logger.info(f"Queued dexterous hand open command ({reason})")

    def begin_tracking_cycle(self, reason="person_start"):
        if not self.enabled:
            return
        with self.target_lock:
            self.pending_positions = None
            self.last_seen_time = 0.0
            self.lost_open_sent = False
        self.smoothed_positions = None
        self.filtered_bends = None
        self.spike_candidate_counts = {name: 0 for name in self.RM_HAND_LABELS}
        self.multi_finger_spike_frames = 0
        self.closure_candidate_frames = 0
        self.logger.info(f"Dexterous hand control reference reset ({reason})")

    def tuning_values(self):
        return {
            "hand_bend_deadzone": float(self.bend_deadzone),
            "hand_bend_smoothing": float(self.bend_smoothing),
            "hand_bend_max_delta": float(self.bend_max_delta),
            "hand_send_hz": float(self.send_hz),
            "hand_smoothing": float(self.smoothing),
            "hand_deadband": int(self.deadband),
        }

    def apply_tuning(self, values, reason="runtime_tuning"):
        self.bend_deadzone = max(
            0.0,
            min(1.0, float(values.get("hand_bend_deadzone", self.bend_deadzone))),
        )
        self.bend_smoothing = max(
            0.0,
            min(1.0, float(values.get("hand_bend_smoothing", self.bend_smoothing))),
        )
        self.bend_max_delta = max(
            0.0,
            min(1.0, float(values.get("hand_bend_max_delta", self.bend_max_delta))),
        )
        self.send_hz = max(1.0, float(values.get("hand_send_hz", self.send_hz)))
        self.send_interval_s = 1.0 / self.send_hz
        self.smoothing = max(
            0.0,
            min(1.0, float(values.get("hand_smoothing", self.smoothing))),
        )
        self.deadband = max(0, int(round(float(values.get("hand_deadband", self.deadband)))))
        self.filtered_bends = None
        self.spike_candidate_counts = {name: 0 for name in self.RM_HAND_LABELS}
        self.multi_finger_spike_frames = 0
        self.closure_candidate_frames = 0
        self.logger.info(
            "Applied hand bend tuning "
            f"({reason}): bend_deadzone={self.bend_deadzone:.2f}, "
            f"bend_smoothing={self.bend_smoothing:.2f}, "
            f"bend_max_delta={self.bend_max_delta:.2f}, "
            f"send_hz={self.send_hz:.1f}, "
            f"hand_smoothing={self.smoothing:.2f}, "
            f"hand_deadband={self.deadband}, "
            f"closure_confirm_frames={self.closure_confirm_frames}"
        )

    def pause_control(self, reason="tracking_paused"):
        if not self.enabled:
            return
        with self.target_lock:
            self.pending_positions = None
        self.smoothed_positions = None
        self.filtered_bends = None
        self.spike_candidate_counts = {name: 0 for name in self.RM_HAND_LABELS}
        self.multi_finger_spike_frames = 0
        self.closure_candidate_frames = 0
        self.logger.info(f"Dexterous hand control paused ({reason})")

    def close(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=2.0)
            self.worker = None

        if self.enabled and self.open_on_exit:
            try:
                ret = self._send_positions(self.open_positions())
                if ret == 0:
                    self.logger.info("Dexterous hand opened before exit")
                else:
                    self.logger.warn(f"Failed to open dexterous hand before exit, code={ret}")
            except Exception as exc:
                self.logger.warn(f"Failed to open dexterous hand before exit: {exc}")

        if self.brainco_canfd is not None:
            try:
                self.brainco_canfd.close()
                self.logger.info("BrainCo Revo2 CANFD hand transport closed")
            except Exception as exc:
                self.logger.warn(f"Failed to close BrainCo CANFD hand transport: {exc}")
            finally:
                self.brainco_canfd = None

        if self.brainco_rs485 is not None:
            try:
                self.brainco_rs485.close()
                self.logger.info("BrainCo Revo2 RS485 hand transport closed")
            except Exception as exc:
                self.logger.warn(f"Failed to close BrainCo RS485 hand transport: {exc}")
            finally:
                self.brainco_rs485 = None

        if self.owns_robot_connection and self.robot is not None:
            try:
                with self.sdk_lock:
                    ret = self.robot.rm_delete_robot_arm()
                if ret == 0:
                    self.logger.info("Disconnected dedicated RM hand connection")
                else:
                    self.logger.warn(
                        f"Failed to disconnect dedicated RM hand connection, code={ret}"
                    )
            except Exception as exc:
                self.logger.warn(f"Failed to close dedicated RM hand connection: {exc}")
            finally:
                self.robot = None

    def open_positions(self):
        return [self.position_min] * 6

    def _connect_hand_robot(self):
        self.robot = RoboticArm(rm_thread_mode_e(self.thread_mode))
        handle = self.robot.rm_create_robot_arm(
            self.robot_ip,
            self.robot_port,
            self.robot_level,
        )
        if handle.id == -1:
            self.robot = None
            raise RuntimeError(
                f"Failed to create dedicated RM hand connection at "
                f"{self.robot_ip}:{self.robot_port}"
            )
        self.owns_robot_connection = True
        self.logger.info(
            f"Dedicated RM hand connection opened handle={handle.id}, "
            f"API={rm_api_version()}"
        )

    def _worker_loop(self):
        while not self.stop_event.is_set():
            with self.target_lock:
                positions = self.pending_positions
                self.pending_positions = None

            if positions is None:
                time.sleep(0.01)
                continue

            now = time.monotonic()
            wait_s = self.send_interval_s - (now - self.last_send_time)
            if wait_s > 0.0:
                time.sleep(min(wait_s, self.send_interval_s))

            if self._within_deadband(positions, self.last_sent_positions):
                continue

            now = time.monotonic()
            if not self._arm_gate_allows_hand_command(now):
                with self.target_lock:
                    if self.pending_positions is None:
                        self.pending_positions = positions
                time.sleep(min(0.05, self.send_interval_s))
                continue

            try:
                ret = self._send_positions(positions)
            except Exception as exc:
                self.last_send_time = time.monotonic()
                self.logger.warn(f"Dexterous hand command failed: {exc}, pos={positions}")
                continue
            self.last_send_time = time.monotonic()
            if ret == 0:
                self.last_sent_positions = list(positions)
                self.logger.debug(f"Dexterous hand positions sent: {positions}")
            elif ret == -1002:
                with self.target_lock:
                    if self.pending_positions is None:
                        self.pending_positions = positions
                time.sleep(min(0.05, self.send_interval_s))
                continue
            else:
                self.logger.warn(f"Dexterous hand command failed, code={ret}, pos={positions}")

    def _reliable_bend_source(self, source):
        if source is None:
            return False
        source_text = str(source).lower()
        return "3d" in source_text or "depth-rejected" in source_text

    def _positions_from_bends(self, bends):
        bends = self._filtered_bend_values(bends)
        thumb_bend = self._bend_value(bends, "thumb")
        thumb_rotate = max(0.0, min(1.0, thumb_bend * self.thumb_rotate_scale))
        normalized = [
            self._bend_value(bends, "pinky"),
            self._bend_value(bends, "ring"),
            self._bend_value(bends, "middle"),
            self._bend_value(bends, "index"),
            thumb_bend,
            thumb_rotate,
        ]
        raw_positions = [self._scale_position(value) for value in normalized]
        if self.smoothed_positions is None:
            self.smoothed_positions = [float(value) for value in raw_positions]
        else:
            alpha = self.smoothing
            self.smoothed_positions = [
                alpha * float(new_value) + (1.0 - alpha) * old_value
                for old_value, new_value in zip(self.smoothed_positions, raw_positions)
            ]
        return [self._clamp_position(round(value)) for value in self.smoothed_positions]

    def _filtered_bend_values(self, bends):
        raw = {name: self._bend_value(bends, name) for name in HandKinematics.FINGER_NAMES}
        raw = {
            name: 0.0 if value < self.bend_deadzone else value
            for name, value in raw.items()
        }
        if self.filtered_bends is None:
            self.filtered_bends = dict(raw)
            return dict(self.filtered_bends)

        raw = self._suppress_multi_finger_spikes(raw)
        raw = self._suppress_unconfirmed_closure(raw)
        next_values = {}
        for name in HandKinematics.FINGER_NAMES:
            value = raw[name]
            previous = self.filtered_bends.get(name, 0.0)

            if self._looks_like_single_finger_spike(name, value, previous, raw):
                self.spike_candidate_counts[name] += 1
                if self.spike_candidate_counts[name] < self.spike_confirm_frames:
                    value = previous
            else:
                self.spike_candidate_counts[name] = 0

            if self.bend_max_delta > 0.0:
                delta = max(-self.bend_max_delta, min(self.bend_max_delta, value - previous))
                value = previous + delta

            value = (
                self.bend_smoothing * value
                + (1.0 - self.bend_smoothing) * previous
            )
            next_values[name] = max(0.0, min(1.0, value))

        self.filtered_bends = next_values
        return dict(self.filtered_bends)

    def _suppress_unconfirmed_closure(self, raw):
        if self.filtered_bends is None:
            self.closure_candidate_frames = 0
            return raw

        rising_names = []
        for name in ("thumb", "index", "middle", "ring", "pinky"):
            previous = self.filtered_bends.get(name, 0.0)
            value = raw.get(name, 0.0)
            if (
                value >= self.closure_confirm_value
                and value - previous >= self.closure_confirm_delta
            ):
                rising_names.append(name)

        if len(rising_names) < self.closure_confirm_min_fingers:
            self.closure_candidate_frames = 0
            return raw

        self.closure_candidate_frames += 1
        if self.closure_candidate_frames >= self.closure_confirm_frames:
            return raw

        held = dict(raw)
        for name in rising_names:
            held[name] = self.filtered_bends.get(name, 0.0)
        return held

    def _suppress_multi_finger_spikes(self, raw):
        if not self.reject_multi_finger_spikes:
            self.multi_finger_spike_frames = 0
            return raw

        rising_names = []
        for name in ("index", "middle", "ring", "pinky"):
            previous = self.filtered_bends.get(name, 0.0)
            if raw.get(name, 0.0) - previous >= self.multi_finger_spike_delta:
                rising_names.append(name)

        if len(rising_names) < self.multi_finger_spike_min_count:
            self.multi_finger_spike_frames = 0
            return raw

        self.multi_finger_spike_frames += 1
        if self.multi_finger_spike_frames >= self.multi_finger_spike_confirm_frames:
            return raw

        held = dict(raw)
        for name in rising_names:
            held[name] = self.filtered_bends.get(name, 0.0)
        return held

    def _looks_like_single_finger_spike(self, name, value, previous, raw):
        if not self.reject_single_finger_spikes:
            return False
        if name == "thumb":
            return False
        if value - previous < self.single_finger_spike_delta:
            return False
        peer_values = [
            raw[peer_name]
            for peer_name in ("index", "middle", "ring", "pinky")
            if peer_name != name
        ]
        return max(peer_values, default=0.0) < self.single_finger_spike_peer_max

    def _configure_if_requested(self):
        if self.robot is None:
            if (
                self.configure_tool_voltage
                or self.configure_rm_plus_mode
                or self.configure_tool_rs485
                or self.configure_speed_force
            ):
                self.logger.warn(
                    "Skipping RM end-effector configuration because no RM_API "
                    "connection is available; BrainCo CANFD hand control can still run "
                    "if the hand is powered and connected to SocketCAN"
                )
        else:
            with self.sdk_lock:
                if self.configure_tool_voltage:
                    ret = self.robot.rm_set_tool_voltage(self.tool_voltage_type)
                    if ret == 0:
                        self.logger.info(
                            f"Tool power output configured for dexterous hand: "
                            f"voltage_type={self.tool_voltage_type}"
                        )
                    else:
                        self.logger.warn(
                            f"Failed to configure tool power output, code={ret}"
                        )

                if self.configure_rm_plus_mode:
                    ret = self.robot.rm_set_rm_plus_mode(self.rm_plus_mode)
                    if ret == 0:
                        self.logger.info(
                            f"RM Plus end-effector protocol configured: "
                            f"mode={self.rm_plus_mode}"
                        )
                    else:
                        self.logger.warn(
                            f"Failed to configure RM Plus protocol, code={ret}"
                        )

                if self.configure_tool_rs485:
                    ret = self.robot.rm_set_tool_rs485_mode(
                        self.tool_rs485_mode,
                        self.tool_rs485_baudrate,
                    )
                    if ret == 0:
                        self.logger.info(
                            "Tool RS485 configured for dexterous hand: "
                            f"mode={self.tool_rs485_mode}, "
                            f"baudrate={self.tool_rs485_baudrate}"
                        )
                    else:
                        self.logger.warn(f"Failed to configure tool RS485, code={ret}")

                if self.transport == "raw-modbus":
                    self._configure_revo2_raw_modbus()

                if self.configure_speed_force:
                    if self.transport in ("raw-modbus", "brainco-canfd", "brainco-rs485"):
                        self.logger.warn(
                            "Skipping rm_set_hand_speed/rm_set_hand_force because "
                            f"{self.transport} exposes Revo2 position control directly"
                        )
                    else:
                        speed_ret = self.robot.rm_set_hand_speed(self.speed)
                        force_ret = self.robot.rm_set_hand_force(self.force)
                        if speed_ret == 0 and force_ret == 0:
                            self.logger.info(
                                f"RealMan hand speed/force configured: "
                                f"speed={self.speed}, force={self.force}"
                            )
                        else:
                            self.logger.warn(
                                "Failed to configure RealMan hand speed/force: "
                                f"speed_code={speed_ret}, force_code={force_ret}"
                            )

        if self.transport == "brainco-canfd":
            self._configure_brainco_canfd()
        if self.transport == "brainco-rs485":
            self._configure_brainco_rs485()

    def _send_positions(self, positions):
        positions = [self._clamp_position(value) for value in positions[:6]]
        while len(positions) < 6:
            positions.append(self.position_min)

        if self.brainco_canfd is not None:
            revo2_positions = BrainCoCanfdHandClient.rm_order_to_revo2_order(positions)
            self.brainco_canfd.set_positions(revo2_positions)
            return 0

        if self.brainco_rs485 is not None:
            revo2_positions = BrainCoRs485HandClient.rm_order_to_revo2_order(positions)
            self.brainco_rs485.set_positions(revo2_positions)
            return 0

        if not self.sdk_lock.acquire(timeout=max(0.0, self.sdk_lock_timeout_s)):
            self._warn_sdk_busy(
                "Hand command is waiting because the shared RM SDK connection is busy"
            )
            return -1002
        try:
            if self.revo2_modbus is not None:
                revo2_positions = Revo2ToolModbus.rm_order_to_revo2_order(positions)
                self.revo2_modbus.set_positions(revo2_positions)
                return 0
            if self.use_follow_pos:
                return self.robot.rm_set_hand_follow_pos(positions, self.block)
            return self.robot.rm_set_hand_angle(positions, self.block, 1)
        finally:
            self.sdk_lock.release()

    def _configure_revo2_raw_modbus(self):
        if not 0 <= self.raw_modbus_slave_id <= 254:
            raise ValueError("hand_raw_modbus_slave_id must be 0 or in the range 1..254")

        slave_id = self.raw_modbus_slave_id or REVO2_FACTORY_SLAVE_IDS[0]
        self.revo2_modbus = Revo2ToolModbus.create(
            self.robot,
            self.raw_modbus_generation,
            slave_id,
            self.raw_modbus_baudrate,
            logger=self.logger,
        )
        self.revo2_modbus.describe()

        if self.raw_modbus_configure_tool_port:
            self.revo2_modbus.configure()

        if self.raw_modbus_slave_id == 0:
            detected_slave_id = self.revo2_modbus.detect_slave_id()
            self.logger.info(f"Using detected Revo2 slave ID {detected_slave_id}")
        elif self.raw_modbus_probe_on_start:
            self.revo2_modbus.probe()

        if self.raw_modbus_read_position_on_start:
            try:
                positions = self.revo2_modbus.read_positions()
                self.logger.info(f"Revo2 communication OK; current positions={positions}")
            except RuntimeError as exc:
                self.logger.warn(
                    "Revo2 FC04 actual-position read failed during startup, "
                    f"but FC03 probe already succeeded; continuing hand control: {exc}"
                )

    def _configure_brainco_canfd(self):
        if not 0 <= self.canfd_slave_id <= 254:
            raise ValueError("hand_canfd_slave_id must be in the range 0..254")
        self.brainco_canfd = BrainCoCanfdHandClient(
            sdk_path=self.canfd_sdk_path,
            iface=self.canfd_iface,
            slave_id=self.canfd_slave_id,
            master_id=self.canfd_master_id,
            use_builtin_socketcan=self.canfd_use_builtin_socketcan,
            use_durations=self.canfd_use_durations,
            duration_ms=self.canfd_duration_ms,
            speed=self.canfd_speed,
            command_timeout_s=self.canfd_command_timeout_s,
            verify_on_start=self.canfd_verify_on_start,
            logger=self.logger,
        )
        self.logger.info(
            "BrainCo Revo2 CANFD hand transport started: "
            f"iface={self.canfd_iface}, slave=0x{self.canfd_slave_id:02X}, "
            f"master={self.canfd_master_id}, durations={self.canfd_use_durations}, "
            f"duration_ms={self.canfd_duration_ms}, speed={self.canfd_speed}"
        )

    def _configure_brainco_rs485(self):
        if not 0 <= self.rs485_slave_id <= 254:
            raise ValueError("hand_rs485_slave_id must be 0 for auto-detect or in the range 1..254")
        self.brainco_rs485 = BrainCoRs485HandClient(
            sdk_path=self.rs485_sdk_path,
            port=self.rs485_port,
            slave_id=self.rs485_slave_id,
            baudrate=self.rs485_baudrate,
            quick_detect=self.rs485_quick_detect,
            use_durations=self.rs485_use_durations,
            duration_ms=self.rs485_duration_ms,
            speed=self.rs485_speed,
            command_timeout_s=self.rs485_command_timeout_s,
            verify_on_start=self.rs485_verify_on_start,
            logger=self.logger,
        )
        self.logger.info(
            "BrainCo Revo2 RS485 hand transport started: "
            f"port={self.brainco_rs485.port}, slave={self.brainco_rs485.slave_id}, "
            f"baudrate={self.brainco_rs485.baudrate}, "
            f"durations={self.rs485_use_durations}, "
            f"duration_ms={self.rs485_duration_ms}, speed={self.rs485_speed}"
        )

    def _command_name(self):
        if self.transport == "brainco-rs485":
            port = self.brainco_rs485.port if self.brainco_rs485 is not None else self.rs485_port
            slave_id = (
                self.brainco_rs485.slave_id
                if self.brainco_rs485 is not None
                else self.rs485_slave_id
            )
            return f"BrainCo Revo2 direct RS485 {port}/{slave_id}"
        if self.transport == "brainco-canfd":
            return (
                "BrainCo Revo2 SocketCAN CANFD "
                f"{self.canfd_iface}/0x{self.canfd_slave_id:02X}"
            )
        if self.transport == "raw-modbus":
            return "Revo2 raw Modbus registers 1070..1072"
        if self.use_follow_pos:
            return "rm_set_hand_follow_pos"
        return "rm_set_hand_angle"

    def _warn_sdk_busy(self, message):
        now = time.monotonic()
        self.sdk_busy_count += 1
        if now - self.last_sdk_busy_log_time < self.sdk_busy_log_interval_s:
            return
        self.logger.warn(f"{message} (retry_count={self.sdk_busy_count})")
        self.sdk_busy_count = 0
        self.last_sdk_busy_log_time = now

    def _within_deadband(self, positions, last_positions):
        if last_positions is None:
            return False
        deltas = [abs(int(a) - int(b)) for a, b in zip(positions, last_positions)]
        return max(deltas, default=0) < self.deadband

    def _scale_position(self, value):
        value = max(0.0, min(1.0, float(value)))
        span = self.position_max - self.position_min
        return self.position_min + span * value

    def _clamp_position(self, value):
        return max(self.position_min, min(self.position_max, int(value)))

    @staticmethod
    def _bend_value(bends, name):
        return max(0.0, min(1.0, float(bends.get(name, 0.0))))


class HandPalmRm65ControlNode(Node):
    STATE_WAITING = "waiting_for_person"
    STATE_READY = "ready_to_start"
    STATE_TRACKING = "tracking"

    def __init__(self):
        super().__init__("hand_palm_rm65_control_node")
        self.persistent_defaults = load_persistent_defaults(self.get_logger())
        if self.persistent_defaults:
            self.get_logger().info(
                f"Loaded persistent tuning defaults from {PERSISTENT_DEFAULTS_PATH}"
            )

        self.finger_bend_topic = self.declare_parameter(
            "finger_bend_topic",
            "hand_finger_bends",
        ).value
        self.palm_motion_topic = self.declare_parameter(
            "palm_motion_topic",
            "hand_palm_direction_distances",
        ).value
        display_max_width = self.declare_parameter("display_max_width", 960).value
        display_max_height = self.declare_parameter("display_max_height", 540).value
        show_window = bool(self.declare_parameter("show_window", True).value)
        color_width = int(self.declare_parameter("color_width", 1280).value)
        color_height = int(self.declare_parameter("color_height", 720).value)
        color_fps = int(self.declare_parameter("color_fps", 30).value)
        depth_width = int(self.declare_parameter("depth_width", 0).value)
        depth_height = int(self.declare_parameter("depth_height", 0).value)
        depth_fps = int(self.declare_parameter("depth_fps", 0).value)
        mp_hands_model_complexity = int(
            self.declare_parameter("mp_hands_model_complexity", 0).value
        )
        mp_pose_enabled = bool(
            self.declare_parameter("mp_pose_enabled", False).value
        )
        mp_pose_backend = str(
            self.declare_parameter("mp_pose_backend", "auto").value
        ).strip().lower()
        mp_pose_model_complexity = int(
            self.declare_parameter("mp_pose_model_complexity", 1).value
        )
        mp_pose_model_path = str(
            self.declare_parameter("mp_pose_model_path", "").value
        ).strip()
        mp_pose_task_model_path = str(
            self.declare_parameter("mp_pose_task_model_path", "").value
        ).strip()
        mp_detection_confidence = float(
            self.declare_parameter("mp_detection_confidence", 0.65).value
        )
        mp_tracking_confidence = float(
            self.declare_parameter("mp_tracking_confidence", 0.65).value
        )
        self.nn_inference_resize_enabled = bool(
            self.declare_parameter("nn_inference_resize_enabled", True).value
        )
        self.nn_inference_max_side_px = int(
            self.declare_parameter("nn_inference_max_side_px", 384).value
        )
        self.nn_skip_arm_angle_log = bool(
            self.declare_parameter("nn_skip_arm_angle_log", True).value
        )
        depth_enhancement_enabled = bool(
            self.declare_parameter("depth_enhancement_enabled", False).value
        )
        depth_enhancement_backend = str(
            self.declare_parameter("depth_enhancement_backend", "hybrid").value
        ).strip().lower()
        lingbot_depth_repo_path = str(
            self.declare_parameter("lingbot_depth_repo_path", "").value
        ).strip()
        lingbot_depth_model_path = str(
            self.declare_parameter("lingbot_depth_model_path", "").value
        ).strip()
        lingbot_depth_device = str(
            self.declare_parameter("lingbot_depth_device", "auto").value
        ).strip().lower()
        lingbot_depth_max_hz = float(
            self.declare_parameter("lingbot_depth_max_hz", 3.0).value
        )
        lingbot_depth_model_max_side_px = int(
            self.declare_parameter("lingbot_depth_model_max_side_px", 384).value
        )
        lingbot_depth_mixed_precision = bool(
            self.declare_parameter("lingbot_depth_mixed_precision", False).value
        )
        lingbot_depth_apply_mask = bool(
            self.declare_parameter("lingbot_depth_apply_mask", False).value
        )
        depth_enhancement_blend_alpha = float(
            self.declare_parameter("depth_enhancement_blend_alpha", 0.65).value
        )
        depth_enhancement_temporal_alpha = float(
            self.declare_parameter("depth_enhancement_temporal_alpha", 0.45).value
        )
        depth_enhancement_min_depth_m = float(
            self.declare_parameter("depth_enhancement_min_depth_m", 0.2).value
        )
        depth_enhancement_max_depth_m = float(
            self.declare_parameter("depth_enhancement_max_depth_m", 5.0).value
        )
        depth_enhancement_max_model_delta_m = float(
            self.declare_parameter("depth_enhancement_max_model_delta_m", 0.35).value
        )
        depth_enhancement_median_ksize = int(
            self.declare_parameter("depth_enhancement_median_ksize", 5).value
        )
        depth_enhancement_bilateral_d = int(
            self.declare_parameter("depth_enhancement_bilateral_d", 5).value
        )

        self.pub_motion = self.create_publisher(Vector3, "hand_arm_motion", 10)
        self.pub_finger_bends = self.create_publisher(
            Float32MultiArray,
            self.finger_bend_topic,
            10,
        )
        self.pub_palm_motion = self.create_publisher(
            Float32MultiArray,
            self.palm_motion_topic,
            10,
        )

        self.frame_buffer = LatestFrameBuffer(maxsize=2)
        self.camera = OrbbecRgbdSource(
            self.get_logger(),
            self.frame_buffer,
            color_width=color_width,
            color_height=color_height,
            color_fps=color_fps,
            depth_width=depth_width,
            depth_height=depth_height,
            depth_fps=depth_fps,
        )
        self.depth_enhancer = None
        if depth_enhancement_enabled:
            if DepthEnhancerConfig is None or LingbotDepthEnhancer is None:
                self.get_logger().warn(
                    "depth_enhancement_enabled is true, but "
                    "lingbot_depth_feature_pack cannot be imported; using raw depth"
                )
            else:
                self.depth_enhancer = LingbotDepthEnhancer(
                    DepthEnhancerConfig(
                        enabled=True,
                        backend=depth_enhancement_backend,
                        repo_path=lingbot_depth_repo_path,
                        model_path=lingbot_depth_model_path,
                        device=lingbot_depth_device,
                        max_hz=lingbot_depth_max_hz,
                        model_max_side_px=lingbot_depth_model_max_side_px,
                        mixed_precision=lingbot_depth_mixed_precision,
                        apply_mask=lingbot_depth_apply_mask,
                        blend_alpha=depth_enhancement_blend_alpha,
                        temporal_alpha=depth_enhancement_temporal_alpha,
                        min_depth_m=depth_enhancement_min_depth_m,
                        max_depth_m=depth_enhancement_max_depth_m,
                        max_model_delta_m=depth_enhancement_max_model_delta_m,
                        median_ksize=depth_enhancement_median_ksize,
                        bilateral_d=depth_enhancement_bilateral_d,
                    ),
                    logger=self.get_logger(),
                )
        self.person_selector = NearestPersonDepthSelector(self)
        self.detector = MediaPipeLandmarkDetector(
            hands_model_complexity=mp_hands_model_complexity,
            pose_enabled=mp_pose_enabled,
            pose_backend=mp_pose_backend,
            pose_model_complexity=mp_pose_model_complexity,
            pose_model_path=mp_pose_model_path,
            pose_task_model_path=mp_pose_task_model_path,
            detection_confidence=mp_detection_confidence,
            tracking_confidence=mp_tracking_confidence,
            logger=self.get_logger(),
        )
        self.kinematics = NeuralHandKinematics(self)
        self.display = CvDisplay(
            "Hand Palm RM65-B Control",
            display_max_width,
            display_max_height,
            show_window=show_window,
        )
        self.palm_follower = RM65PalmFollower(self)
        self.hand_follower = RMHandFingerFollower(self, self.palm_follower)
        self.display.configure_tuning(
            self.gui_tuning_values(),
            enabled=True,
        )

        self.prev_points_3d = {}
        self.prev_time = None

        now = time.monotonic()
        self.last_frame_log_time = 0.0
        self.last_angle_log_time = 0.0
        self.last_finger_log_time = 0.0
        self.last_palm_log_time = 0.0
        self.last_restart_time = now

        self.angle_log_interval = 0.5
        self.finger_log_interval = 0.5
        self.palm_log_interval = 0.5
        self.frame_stale_timeout = 3.0
        self.restart_cooldown = 6.0
        self.max_restart_attempts = 3
        self.restart_attempts = 0
        self.restart_in_progress = False
        self.is_shutting_down = False
        self.restart_lock = threading.Lock()
        self.tracking_state = self.STATE_WAITING
        self.nearest_candidate = None
        self.target_candidate = None
        self.target_lost_since = None
        self.person_lost_timeout_s = float(
            self.declare_parameter("person_lost_timeout_s", 2.0).value
        )
        self.auto_start_on_ready = bool(
            self.declare_parameter("auto_start_on_ready", False).value
        )
        self.auto_start_delay_s = float(
            self.declare_parameter("auto_start_delay_s", 1.0).value
        )
        self.last_person_log_time = 0.0
        self.auto_start_ready_since = None

        self.camera.start()
        self.get_logger().info(
            "Hardware/software initialization complete. Waiting for the nearest "
            "person; press 's' in the display window to start recognition/control."
        )
        self.get_logger().info(
            "Neural acceleration enabled: "
            f"mp_hands_complexity={mp_hands_model_complexity}, "
            f"mp_pose_enabled={mp_pose_enabled}, "
            f"mp_pose_backend={mp_pose_backend}, "
            f"mp_pose_complexity={mp_pose_model_complexity}, "
            f"mp_pose_model_path={'set' if mp_pose_model_path else 'empty'}, "
            f"mp_pose_task_model_path={'set' if mp_pose_task_model_path else 'empty'}, "
            f"resize_roi={self.nn_inference_resize_enabled}, "
            f"max_side={self.nn_inference_max_side_px}px, "
            f"depth_enhancement="
            f"{self.depth_enhancer.backend if self.depth_enhancer else 'off'}"
        )
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.health_timer = self.create_timer(1.0, self.health_callback)

    def health_callback(self):
        if self.is_shutting_down or self.restart_in_progress:
            return

        self.palm_follower.warn_if_motion_stalled()

        stale_for = self.camera.seconds_since_last_frame()
        if stale_for <= self.frame_stale_timeout:
            return

        self.get_logger().warn(
            f"No camera frames for {stale_for:.1f}s; scheduling camera restart"
        )
        self._request_camera_restart("camera frame stream is stale")

    def timer_callback(self):
        try:
            self._timer_callback_impl()
        except Exception as exc:
            self.get_logger().error(f"Unhandled error in timer callback: {exc}")
            self.get_logger().error(traceback.format_exc())

    def _timer_callback_impl(self):
        if self.is_shutting_down or self.restart_in_progress:
            return

        frames = self.camera.latest_aligned_frames()
        if frames is None:
            return

        try:
            color_image, depth_meters = self.camera.frames_to_images(frames)
        except Exception as exc:
            self.get_logger().error(f"Failed to decode frame: {exc}")
            self.get_logger().error(traceback.format_exc())
            return

        if color_image is None or depth_meters is None:
            return

        if self.depth_enhancer is not None:
            enhanced = self.depth_enhancer.enhance(
                color_image,
                depth_meters,
                camera_intrinsic=self.camera.get_camera_intrinsic(),
            )
            depth_meters = enhanced.depth_meters

        if self.tracking_state != self.STATE_TRACKING:
            self.display.set_tuning_enabled(True)
            self._update_waiting_state(color_image, depth_meters)
            return

        self.display.set_tuning_enabled(False)
        if not self._update_tracked_person(color_image, depth_meters):
            return

        isolated_image, roi = self.person_selector.isolate_color_image(
            color_image,
            depth_meters,
            self.target_candidate,
        )
        points_2d = self._detect_landmarks(isolated_image, offset=roi[:2])
        if not points_2d:
            self._handle_tracking_no_landmarks(color_image)
            return

        points_3d = self._build_points_3d(points_2d, depth_meters)
        finger_bends, finger_source = self._publish_finger_bends(points_3d, points_2d)
        palm_distances, palm_displacement = self._publish_palm_motion(points_3d, points_2d)
        arm_offset = self.palm_follower.update_from_palm_displacement(
            palm_displacement
        )
        hand_positions = self.hand_follower.update_from_bends(
            finger_bends,
            source=finger_source,
        )

        if points_3d:
            self._publish_motion_velocity(points_3d)
            self._log_arm_angles(points_3d, points_2d)
        else:
            self.get_logger().info("No valid 3D points")
            self._log_arm_angles({}, points_2d)
            self._reset_tracking_state()

        display_action = self.display.show(
            color_image,
            points_2d,
            finger_bends=finger_bends,
            finger_source=finger_source,
            palm_distances=palm_distances,
            arm_offset=arm_offset,
            hand_positions=hand_positions,
            status_text=self._status_text(),
            person_roi=self.target_candidate.bbox if self.target_candidate else None,
        )
        self._handle_display_action(display_action)

    def _handle_display_action(self, action):
        if action == "start":
            if self.tracking_state == self.STATE_READY and self.nearest_candidate:
                self._start_tracking(self.nearest_candidate)
            elif self.tracking_state == self.STATE_WAITING:
                self.get_logger().warn("Cannot start yet: no person candidate in view")
        elif action == "save_tuning":
            if self.tracking_state == self.STATE_TRACKING:
                self.get_logger().warn(
                    "GUI tuning is locked while tracking. Press h first, then tune/save."
                )
            else:
                self._save_gui_tuning_defaults()
        elif action == "home":
            if self.palm_follower.home_key_enabled:
                self._reset_tracking_state()
                self.tracking_state = self.STATE_WAITING
                self.nearest_candidate = None
                self.target_candidate = None
                self.target_lost_since = None
                self.auto_start_ready_since = None
                self.display.set_tuning_enabled(True)
                self.hand_follower.pause_control("keyboard_h")
                if self.hand_follower.open_on_home:
                    self.hand_follower.open_hand("keyboard_h")
                self.palm_follower.request_home("keyboard_h")
            else:
                self.get_logger().warn("Home key is disabled by home_key_enabled:=false")
        elif action == "quit":
            self.is_shutting_down = True
            rclpy.shutdown()

    def _detect_landmarks(self, color_image, offset=(0, 0)):
        offset_x, offset_y = offset
        height, width = color_image.shape[:2]
        inference_image = color_image
        scale = 1.0

        max_side = max(64, int(self.nn_inference_max_side_px))
        if (
            self.nn_inference_resize_enabled
            and max(height, width) > max_side
        ):
            scale = max_side / float(max(height, width))
            resized_width = max(1, int(round(width * scale)))
            resized_height = max(1, int(round(height * scale)))
            inference_image = cv2.resize(
                color_image,
                (resized_width, resized_height),
                interpolation=cv2.INTER_AREA,
            )

        image_rgb = cv2.cvtColor(inference_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        points = self.detector.collect(image_rgb, offset=(0, 0))

        if scale == 1.0:
            return {
                key: (int(x + offset_x), int(y + offset_y))
                for key, (x, y) in points.items()
            }

        inverse_scale = 1.0 / scale
        remapped = {}
        for key, (x, y) in points.items():
            original_x = max(0, min(width - 1, int(round(x * inverse_scale))))
            original_y = max(0, min(height - 1, int(round(y * inverse_scale))))
            remapped[key] = (original_x + offset_x, original_y + offset_y)
        return remapped

    def _update_waiting_state(self, color_image, depth_meters):
        candidate = self.person_selector.find_nearest(depth_meters)
        self.nearest_candidate = candidate
        self.target_candidate = None
        self.target_lost_since = None

        now = time.monotonic()
        if candidate is None:
            self.tracking_state = self.STATE_WAITING
            self.auto_start_ready_since = None
            if now - self.last_person_log_time > 1.0:
                self.get_logger().info("Waiting: no nearest-person depth candidate")
                self.last_person_log_time = now
            status = self._waiting_status_text(
                "Waiting for person... tune GUI, press p to save"
            )
            roi = None
        else:
            self.tracking_state = self.STATE_READY
            if self.auto_start_ready_since is None:
                self.auto_start_ready_since = now
            if now - self.last_person_log_time > 1.0:
                self.get_logger().info(
                    "Nearest person candidate ready: "
                    f"{self.person_selector.describe(candidate)}. Press 's' to start."
                )
                self.last_person_log_time = now
            status = self._waiting_status_text(
                f"Ready: nearest person {candidate.median_depth:.2f}m | press s"
            )
            roi = candidate.bbox
            if (
                self.auto_start_on_ready
                and now - self.auto_start_ready_since >= self.auto_start_delay_s
            ):
                self.get_logger().info(
                    "Auto-starting recognition/control for nearest person "
                    f"after {now - self.auto_start_ready_since:.1f}s ready delay"
                )
                self._start_tracking(candidate)
                return

        action = self.display.show(
            color_image,
            {},
            status_text=status,
            person_roi=roi,
        )
        self._handle_display_action(action)

    def _start_tracking(self, candidate):
        self._reset_tracking_state()
        self._apply_pending_gui_tuning("keyboard_s")
        if not self.palm_follower.begin_tracking_cycle("keyboard_s"):
            self.tracking_state = self.STATE_READY
            self.nearest_candidate = candidate
            self.target_candidate = None
            self.target_lost_since = None
            return

        self.tracking_state = self.STATE_TRACKING
        self.target_candidate = candidate
        self.nearest_candidate = None
        self.target_lost_since = None
        self.auto_start_ready_since = None
        self.hand_follower.begin_tracking_cycle("keyboard_s")
        self.get_logger().info(
            "Started tracking/control for nearest person: "
            f"{self.person_selector.describe(candidate)}"
        )

    def gui_tuning_values(self):
        values = self.palm_follower.mit_tuning_values()
        values.update(self.hand_follower.tuning_values())
        return values

    def _apply_pending_gui_tuning(self, reason):
        if self.tracking_state == self.STATE_TRACKING:
            return
        values = self.display.get_tuning_values()
        self.palm_follower.apply_mit_tuning(values, reason)
        self.hand_follower.apply_tuning(values, reason)

    def _save_gui_tuning_defaults(self):
        values = self.display.get_tuning_values()
        self.palm_follower.apply_mit_tuning(values, "gui_save")
        self.hand_follower.apply_tuning(values, "gui_save")
        try:
            save_persistent_defaults(values)
        except Exception as exc:
            self.get_logger().error(f"Failed to save GUI tuning defaults: {exc}")
            return
        self.get_logger().info(
            f"Saved GUI tuning defaults to {PERSISTENT_DEFAULTS_PATH}. "
            "They will be used on the next run when no matching ROS parameter is passed."
        )

    def _update_tracked_person(self, color_image, depth_meters):
        search_bbox = self.target_candidate.bbox if self.target_candidate else None
        candidate = self.person_selector.find_nearest(depth_meters, search_bbox)
        now = time.monotonic()

        if candidate is not None:
            self.target_candidate = candidate
            self.target_lost_since = None
            return True

        if self.target_lost_since is None:
            self.target_lost_since = now

        lost_for = now - self.target_lost_since
        status = f"Target lost {lost_for:.1f}s; pausing recognition/control"
        action = self.display.show(
            color_image,
            {},
            status_text=status,
            person_roi=search_bbox,
        )
        self._handle_display_action(action)

        if lost_for >= self.person_lost_timeout_s:
            self._pause_and_wait_for_next_person("tracked_person_left_view")
        return False

    def _pause_and_wait_for_next_person(self, reason):
        self.get_logger().warn(
            f"Tracked person left the camera view; pausing recognition/control ({reason})"
        )
        self._reset_tracking_state()
        self.palm_follower.pause_control(reason)
        self.hand_follower.pause_control(reason)
        self.hand_follower.notify_hand_lost()
        self.tracking_state = self.STATE_WAITING
        self.nearest_candidate = None
        self.target_candidate = None
        self.target_lost_since = None
        self.auto_start_ready_since = None

    def _handle_tracking_no_landmarks(self, color_image):
        now = time.monotonic()
        if now - self.last_frame_log_time > 1.0:
            self.get_logger().info("Tracked person visible, but no hand/arm landmarks detected")
            self.last_frame_log_time = now
        self._reset_tracking_state()
        action = self.display.show(
            color_image,
            {},
            status_text=self._status_text("no hand/arm landmarks"),
            person_roi=self.target_candidate.bbox if self.target_candidate else None,
        )
        self._handle_display_action(action)

    def _status_text(self, suffix=None):
        text = {
            self.STATE_WAITING: "Waiting for nearest person",
            self.STATE_READY: "Ready - press s to start",
            self.STATE_TRACKING: "Tracking target person",
        }.get(self.tracking_state, self.tracking_state)
        if suffix:
            text = f"{text} | {suffix}"
        return text

    def _waiting_status_text(self, prefix):
        values = self.display.get_tuning_values()
        axis_deadband = values.get("arm_axis_deadband_m", 0.0)
        if isinstance(axis_deadband, (list, tuple, np.ndarray)):
            axis_deadband = max(float(value) for value in axis_deadband[:3])
        return (
            f"{prefix} | p save | "
            f"kp={values.get('arm_mit_kp', 0):.0f} "
            f"kd={values.get('arm_mit_kd', 0):.1f} "
            f"v={values.get('arm_mit_max_velocity_m_s', 0):.2f} "
            f"a={values.get('arm_mit_max_accel_m_s2', 0):.2f} "
            f"palm={values.get('hand_deadband_m', 0) * 1000.0:.0f}mm "
            f"axis={float(axis_deadband) * 1000.0:.0f}mm "
            f"hold={values.get('palm_filter_hold_deadband_m', 0) * 1000.0:.0f}/"
            f"{values.get('palm_filter_release_deadband_m', 0) * 1000.0:.0f}mm "
            f"bend={values.get('hand_bend_deadzone', 0) * 100.0:.0f}% "
            f"handHz={values.get('hand_send_hz', 0):.0f} "
            f"hAlpha={values.get('hand_smoothing', 0) * 100.0:.0f}%"
        )

    def _handle_no_landmarks(self, color_image):
        now = time.monotonic()
        if now - self.last_frame_log_time > 1.0:
            self.get_logger().info("No hand or arm landmarks detected")
            self.last_frame_log_time = now
        self.hand_follower.notify_hand_lost()
        self._reset_tracking_state()
        self._handle_display_action(self.display.show(color_image, {}))

    def _build_points_3d(self, points_2d, depth_meters):
        points_3d = {}
        for key, (x, y) in points_2d.items():
            point = self.camera.get_3d_point(x, y, depth_meters)
            if point is not None:
                points_3d[key] = point
        return points_3d

    def _publish_finger_bends(self, points_3d, points_2d):
        bends, flexion_degrees, source = self.kinematics.finger_bends(
            points_3d,
            points_2d,
        )
        if bends is None:
            return None, None

        msg = Float32MultiArray()
        msg.data = [float(bends[name]) for name in HandKinematics.FINGER_NAMES]
        self.pub_finger_bends.publish(msg)

        now = time.monotonic()
        if now - self.last_finger_log_time >= self.finger_log_interval:
            bend_text = ", ".join(
                f"{name}={bends[name]:.2f}" for name in HandKinematics.FINGER_NAMES
            )
            degree_text = ", ".join(
                f"{name}={flexion_degrees[name]:.0f}deg"
                for name in HandKinematics.FINGER_NAMES
            )
            self.get_logger().info(
                f"Finger bends ({source}, topic={self.finger_bend_topic}): "
                f"{bend_text} [{degree_text}]"
            )
            self.last_finger_log_time = now

        return bends, source

    def _publish_palm_motion(self, points_3d, points_2d):
        distances, displacement = self.kinematics.palm_direction_distances(
            points_3d,
            points_2d,
        )
        if distances is None:
            return None, None

        msg = Float32MultiArray()
        msg.data = [float(distances[name]) for name in HandKinematics.PALM_DIRECTION_NAMES]
        self.pub_palm_motion.publish(msg)

        now = time.monotonic()
        if now - self.last_palm_log_time >= self.palm_log_interval:
            distance_text = ", ".join(
                f"{name}={distances[name]:.3f}m"
                for name in HandKinematics.PALM_DIRECTION_NAMES
            )
            self.get_logger().info(
                f"Palm direction distances (topic={self.palm_motion_topic}): "
                f"{distance_text}"
            )
            self.last_palm_log_time = now

        return distances, displacement

    def _publish_motion_velocity(self, points_3d):
        now = time.monotonic()
        if self.prev_points_3d and self.prev_time is not None:
            common_keys = sorted(set(points_3d).intersection(self.prev_points_3d))
            dt = max(now - self.prev_time, 1e-3)
            if common_keys:
                velocities = [
                    (points_3d[key] - self.prev_points_3d[key]) / dt
                    for key in common_keys
                ]
                motion_vec = np.mean(velocities, axis=0)
                msg = Vector3()
                msg.x = float(motion_vec[0])
                msg.y = float(motion_vec[1])
                msg.z = float(motion_vec[2])
                self.pub_motion.publish(msg)

        self.prev_points_3d = points_3d
        self.prev_time = now

    def _log_arm_angles(self, points_3d, points_2d):
        if self.nn_skip_arm_angle_log:
            return

        now = time.monotonic()
        if now - self.last_angle_log_time < self.angle_log_interval:
            return

        angles = self.kinematics.arm_angles(points_3d)
        source = "3D"
        if not angles:
            angles = self.kinematics.arm_angles(
                HandKinematics.points_2d_to_arrays(points_2d)
            )
            source = "2D fallback"

        if angles:
            text = ", ".join(
                f"{side} elbow={angle:.1f} deg" for side, angle in sorted(angles.items())
            )
            self.get_logger().info(f"Arm bend angle ({source}): {text}")
            self.last_angle_log_time = now

    def _reset_tracking_state(self):
        self.prev_points_3d = {}
        self.prev_time = None
        self.kinematics.reset()
        if self.depth_enhancer is not None:
            self.depth_enhancer.reset()

    def _request_camera_restart(self, reason):
        if self.is_shutting_down:
            return

        now = time.monotonic()
        with self.restart_lock:
            if self.restart_in_progress:
                return
            if now - self.last_restart_time < self.restart_cooldown:
                return
            self.restart_in_progress = True
            self.last_restart_time = now

        thread = threading.Thread(
            target=self._restart_camera_worker,
            args=(reason,),
            daemon=True,
        )
        thread.start()

    def _restart_camera_worker(self, reason):
        try:
            if self.restart_attempts >= self.max_restart_attempts:
                self.get_logger().fatal("Camera failed repeatedly, shutting down node")
                self.is_shutting_down = True
                rclpy.shutdown()
                return

            self.restart_attempts += 1
            attempt = self.restart_attempts
            self.get_logger().error(
                f"Restarting camera because {reason} "
                f"({attempt}/{self.max_restart_attempts})"
            )

            self._reset_tracking_state()
            self.palm_follower.pause_control("camera_restart")
            self.hand_follower.pause_control("camera_restart")
            self.camera.stop()
            time.sleep(min(5.0, 0.5 * attempt))
            self.camera.start()
            self.restart_attempts = 0
        except Exception as exc:
            self.get_logger().error(f"Camera restart failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            if self.restart_attempts >= self.max_restart_attempts:
                self.get_logger().fatal("Camera failed repeatedly, shutting down node")
                self.is_shutting_down = True
                rclpy.shutdown()
        finally:
            with self.restart_lock:
                self.restart_in_progress = False

    def destroy_node(self):
        self.is_shutting_down = True
        try:
            self.detector.close()
        except Exception:
            pass
        self.camera.stop()
        if self.depth_enhancer is not None:
            self.depth_enhancer.close()
        self.hand_follower.close()
        self.palm_follower.close()
        self.display.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandPalmRm65ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
