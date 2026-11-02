#!/usr/bin/env bash
set -Eeuo pipefail

ENV_FILE="${RM_BRAINCO_ORBBEC_ENV_FILE:-/etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env}"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

APP_ROOT="${APP_ROOT:-/home/jetson/ws/rm_brainco_orbbec_demo}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
CONTROL_SCRIPT="${CONTROL_SCRIPT:-}"

log() {
  printf '[rm_brainco_orbbec_demo] %s\n' "$*"
}

bool_is_true() {
  case "${1:-false}" in
    true|True|TRUE|1|yes|YES|y|Y) return 0 ;;
    *) return 1 ;;
  esac
}

wait_for_ping() {
  local host="$1"
  local timeout_s="${2:-30}"
  local start
  local now
  start="$(date +%s)"
  while true; do
    if ping -c 1 -W 1 "${host}" >/dev/null 2>&1; then
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      log "Timed out waiting for ${host}"
      return 1
    fi
    sleep 1
  done
}

wait_for_path() {
  local path="$1"
  local timeout_s="${2:-20}"
  local start
  local now
  start="$(date +%s)"
  while true; do
    if [[ -e "${path}" ]]; then
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      log "Timed out waiting for ${path}"
      return 1
    fi
    sleep 1
  done
}

if [[ -f "${ROS_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
else
  log "ROS setup file not found: ${ROS_SETUP}"
fi

if [[ -n "${CONDA_ENV:-}" && -f "${CONDA_SH:-}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  set -u
elif [[ -n "${VENV_ACTIVATE:-}" && -f "${VENV_ACTIVATE}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${VENV_ACTIVATE}"
  set -u
fi

if [[ -z "${CONTROL_SCRIPT}" ]]; then
  if [[ -f "${APP_ROOT}/scripts/camera_arm_rm65b_control_rtmpose.py" ]]; then
    CONTROL_SCRIPT="${APP_ROOT}/scripts/camera_arm_rm65b_control_rtmpose.py"
  elif [[ -f "${APP_ROOT}/scripts/camera_arm_rm65b_control_pose_hands.py" ]]; then
    CONTROL_SCRIPT="${APP_ROOT}/scripts/camera_arm_rm65b_control_pose_hands.py"
  elif [[ -f "${APP_ROOT}/scripts/camera_arm_rm65b_control_neural.py" ]]; then
    CONTROL_SCRIPT="${APP_ROOT}/scripts/camera_arm_rm65b_control_neural.py"
  else
    CONTROL_SCRIPT="${APP_ROOT}/camera_arm_rm65b_control_neural.py"
  fi
fi

if [[ ! -f "${CONTROL_SCRIPT}" ]]; then
  log "Control script not found: ${CONTROL_SCRIPT}"
  exit 2
fi

export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONPATH="${EXTRA_PYTHONPATH:-}:${RM_API2_PYTHON_PATH:-}:${BRAINCO_HAND_SDK_PATH:-}:${LINGBOT_DEPTH_REPO_PATH:-}:${APP_ROOT}:${PYTHONPATH:-}"

cd "${APP_ROOT}"

if bool_is_true "${WAIT_FOR_ROBOT_IP:-true}" && bool_is_true "${ROBOT_MOTION_ENABLED:-false}"; then
  wait_for_ping "${ROBOT_IP:-192.168.1.18}" "${WAIT_FOR_ROBOT_TIMEOUT_S:-45}"
fi

if bool_is_true "${WAIT_FOR_HAND_RS485_PORT:-true}" && [[ "${HAND_TRANSPORT:-}" == "brainco-rs485" && "${HAND_RS485_PORT:-auto}" != "auto" ]]; then
  wait_for_path "${HAND_RS485_PORT:-/dev/ttyUSB0}" "${WAIT_FOR_HAND_RS485_TIMEOUT_S:-20}"
fi

ARGS=(
  "${CONTROL_SCRIPT}"
  "--ros-args"
  "-p" "show_window:=${SHOW_WINDOW:-false}"
  "-p" "display_max_width:=${DISPLAY_MAX_WIDTH:-960}"
  "-p" "display_max_height:=${DISPLAY_MAX_HEIGHT:-540}"
  "-p" "auto_start_on_ready:=${AUTO_START_ON_READY:-false}"
  "-p" "auto_start_delay_s:=${AUTO_START_DELAY_S:-1.0}"
  "-p" "robot_motion_enabled:=${ROBOT_MOTION_ENABLED:-true}"
  "-p" "robot_ip:=${ROBOT_IP:-192.168.1.18}"
  "-p" "robot_port:=${ROBOT_PORT:-8080}"
  "-p" "arm_control_protocol:=${ARM_CONTROL_PROTOCOL:-canfd}"
  "-p" "canfd_follow:=${CANFD_FOLLOW:-false}"
  "-p" "canfd_trajectory_mode:=${CANFD_TRAJECTORY_MODE:-2}"
  "-p" "canfd_radio:=${CANFD_RADIO:-50}"
  "-p" "arm_command_interval_s:=${ARM_COMMAND_INTERVAL_S:-0.03}"
  "-p" "arm_motion_unit_m:=${ARM_MOTION_UNIT_M:-0.002}"
  "-p" "max_arm_target_delta_m:=${MAX_ARM_TARGET_DELTA_M:-0.008}"
  "-p" "min_command_delta_m:=${MIN_COMMAND_DELTA_M:-0.0015}"
  "-p" "arm_block:=${ARM_BLOCK:-0}"
  "-p" "max_arm_offset_m:=${MAX_ARM_OFFSET_M:-0.08}"
  "-p" "arm_mit_control_enabled:=${ARM_MIT_CONTROL_ENABLED:-true}"
  "-p" "arm_mit_kp:=${ARM_MIT_KP:-70.0}"
  "-p" "arm_mit_kd:=${ARM_MIT_KD:-16.0}"
  "-p" "arm_mit_max_velocity_m_s:=${ARM_MIT_MAX_VELOCITY_M_S:-0.22}"
  "-p" "arm_mit_max_accel_m_s2:=${ARM_MIT_MAX_ACCEL_M_S2:-1.5}"
  "-p" "hand_control_enabled:=${HAND_CONTROL_ENABLED:-true}"
  "-p" "hand_transport:=${HAND_TRANSPORT:-brainco-rs485}"
  "-p" "hand_rs485_sdk_path:=${BRAINCO_HAND_SDK_PATH:-/home/jetson/ws/brainco-hand-sdk-main/python}"
  "-p" "hand_rs485_port:=${HAND_RS485_PORT:-/dev/ttyUSB0}"
  "-p" "hand_rs485_slave_id:=${HAND_RS485_SLAVE_ID:-127}"
  "-p" "hand_rs485_baudrate:=${HAND_RS485_BAUDRATE:-460800}"
  "-p" "hand_rs485_duration_ms:=${HAND_RS485_DURATION_MS:-40}"
  "-p" "hand_rs485_speed:=${HAND_RS485_SPEED:-1000}"
  "-p" "hand_wait_for_first_arm_command:=${HAND_WAIT_FOR_FIRST_ARM_COMMAND:-false}"
  "-p" "hand_pause_while_arm_sdk_busy:=${HAND_PAUSE_WHILE_ARM_SDK_BUSY:-false}"
  "-p" "hand_configure_tool_voltage:=${HAND_CONFIGURE_TOOL_VOLTAGE:-true}"
  "-p" "hand_tool_voltage_type:=${HAND_TOOL_VOLTAGE_TYPE:-3}"
  "-p" "hand_send_hz:=${HAND_SEND_HZ:-40.0}"
  "-p" "hand_smoothing:=${HAND_SMOOTHING:-0.75}"
  "-p" "hand_bend_smoothing:=${HAND_BEND_SMOOTHING:-0.55}"
  "-p" "hand_closure_confirm_frames:=${HAND_CLOSURE_CONFIRM_FRAMES:-3}"
  "-p" "hand_multi_finger_spike_confirm_frames:=${HAND_MULTI_FINGER_SPIKE_CONFIRM_FRAMES:-3}"
  "-p" "color_width:=${COLOR_WIDTH:-1280}"
  "-p" "color_height:=${COLOR_HEIGHT:-720}"
  "-p" "color_fps:=${COLOR_FPS:-30}"
  "-p" "depth_width:=${DEPTH_WIDTH:-0}"
  "-p" "depth_height:=${DEPTH_HEIGHT:-0}"
  "-p" "depth_fps:=${DEPTH_FPS:-0}"
  "-p" "vision_backend:=${VISION_BACKEND:-rtmpose}"
  "-p" "rtmpose_runtime:=${RTMPOSE_RUNTIME:-tensorrt}"
  "-p" "rtmpose_device_name:=${RTMPOSE_DEVICE_NAME:-cuda}"
  "-p" "rtmpose_device_id:=${RTMPOSE_DEVICE_ID:-0}"
  "-p" "rtmdet_engine_path:=${RTMDET_ENGINE_PATH:-${APP_ROOT}/models/tensorrt/rtmdet_nano.engine}"
  "-p" "rtmpose_body_engine_path:=${RTMPOSE_BODY_ENGINE_PATH:-${APP_ROOT}/models/tensorrt/rtmpose_body.engine}"
  "-p" "rtmpose_hand_engine_path:=${RTMPOSE_HAND_ENGINE_PATH:-${APP_ROOT}/models/tensorrt/rtmpose_hand.engine}"
  "-p" "rtmdet_model_dir:=${RTMDET_MODEL_DIR:-${APP_ROOT}/models/mmdeploy/rtmdet_nano}"
  "-p" "rtmpose_body_model_dir:=${RTMPOSE_BODY_MODEL_DIR:-${APP_ROOT}/models/mmdeploy/rtmpose_body}"
  "-p" "rtmpose_hand_model_dir:=${RTMPOSE_HAND_MODEL_DIR:-${APP_ROOT}/models/mmdeploy/rtmpose_hand}"
  "-p" "rtmdet_nms_iou_threshold:=${RTMDET_NMS_IOU_THRESHOLD:-0.60}"
  "-p" "rtmdet_max_detections:=${RTMDET_MAX_DETECTIONS:-100}"
  "-p" "rtmpose_bbox_padding:=${RTMPOSE_BBOX_PADDING:-1.25}"
  "-p" "rtmdet_inference_hz:=${RTMDET_INFERENCE_HZ:-8.0}"
  "-p" "rtmpose_person_score_threshold:=${RTMPOSE_PERSON_SCORE_THRESHOLD:-0.45}"
  "-p" "rtmpose_body_keypoint_threshold:=${RTMPOSE_BODY_KEYPOINT_THRESHOLD:-0.35}"
  "-p" "rtmpose_hand_keypoint_threshold:=${RTMPOSE_HAND_KEYPOINT_THRESHOLD:-0.25}"
  "-p" "rtmpose_hand_roi_scale:=${RTMPOSE_HAND_ROI_SCALE:-2.25}"
  "-p" "rtmpose_hand_roi_wrist_extension:=${RTMPOSE_HAND_ROI_WRIST_EXTENSION:-0.35}"
  "-p" "rtmpose_hand_inference_interval:=${RTMPOSE_HAND_INFERENCE_INTERVAL:-2}"
  "-p" "mp_hands_model_complexity:=${MP_HANDS_MODEL_COMPLEXITY:-0}"
  "-p" "mp_pose_enabled:=${MP_POSE_ENABLED:-true}"
  "-p" "mp_pose_backend:=${MP_POSE_BACKEND:-auto}"
  "-p" "mp_pose_model_complexity:=${MP_POSE_MODEL_COMPLEXITY:-0}"
  "-p" "mp_max_num_hands:=${MP_MAX_NUM_HANDS:-2}"
  "-p" "mp_associate_hands_to_pose:=${MP_ASSOCIATE_HANDS_TO_POSE:-true}"
  "-p" "mp_hand_assignment_max_distance_ratio:=${MP_HAND_ASSIGNMENT_MAX_DISTANCE_RATIO:-0.35}"
  "-p" "mp_handedness_swap:=${MP_HANDEDNESS_SWAP:-false}"
  "-p" "control_hand_side:=${CONTROL_HAND_SIDE:-right}"
  "-p" "control_hand_fallback_enabled:=${CONTROL_HAND_FALLBACK_ENABLED:-false}"
  "-p" "landmark_depth_patch_radius_px:=${LANDMARK_DEPTH_PATCH_RADIUS_PX:-2}"
  "-p" "landmark_depth_min_valid_samples:=${LANDMARK_DEPTH_MIN_VALID_SAMPLES:-3}"
  "-p" "operator_require_torso_pose:=${OPERATOR_REQUIRE_TORSO_POSE:-true}"
  "-p" "operator_max_candidate_tests:=${OPERATOR_MAX_CANDIDATE_TESTS:-1}"
  "-p" "operator_torso_depth_patch_radius_px:=${OPERATOR_TORSO_DEPTH_PATCH_RADIUS_PX:-6}"
  "-p" "operator_torso_depth_min_valid_samples:=${OPERATOR_TORSO_DEPTH_MIN_VALID_SAMPLES:-5}"
  "-p" "operator_torso_min_landmarks:=${OPERATOR_TORSO_MIN_LANDMARKS:-2}"
  "-p" "operator_ready_confirm_frames:=${OPERATOR_READY_CONFIRM_FRAMES:-5}"
  "-p" "operator_ready_max_depth_jump_m:=${OPERATOR_READY_MAX_DEPTH_JUMP_M:-0.25}"
  "-p" "operator_ready_max_center_jump_px:=${OPERATOR_READY_MAX_CENTER_JUMP_PX:-160.0}"
  "-p" "mp_detection_confidence:=${MP_DETECTION_CONFIDENCE:-0.65}"
  "-p" "mp_tracking_confidence:=${MP_TRACKING_CONFIDENCE:-0.65}"
  "-p" "nn_inference_resize_enabled:=${NN_INFERENCE_RESIZE_ENABLED:-true}"
  "-p" "nn_inference_max_side_px:=${NN_INFERENCE_MAX_SIDE_PX:-384}"
  "-p" "nn_skip_arm_angle_log:=${NN_SKIP_ARM_ANGLE_LOG:-true}"
  "-p" "depth_enhancement_enabled:=${DEPTH_ENHANCEMENT_ENABLED:-false}"
  "-p" "depth_enhancement_backend:=${DEPTH_ENHANCEMENT_BACKEND:-filter}"
  "-p" "lingbot_depth_repo_path:=${LINGBOT_DEPTH_REPO_PATH:-/home/jetson/ws/lingbot-depth}"
  "-p" "lingbot_depth_model_path:=${LINGBOT_DEPTH_MODEL_PATH:-/home/jetson/ws/lingbot-depth/model.pt}"
  "-p" "lingbot_depth_device:=${LINGBOT_DEPTH_DEVICE:-cuda}"
  "-p" "lingbot_depth_max_hz:=${LINGBOT_DEPTH_MAX_HZ:-3.0}"
  "-p" "lingbot_depth_model_max_side_px:=${LINGBOT_DEPTH_MODEL_MAX_SIDE_PX:-384}"
  "-p" "lingbot_depth_mixed_precision:=${LINGBOT_DEPTH_MIXED_PRECISION:-false}"
  "-p" "depth_enhancement_blend_alpha:=${DEPTH_ENHANCEMENT_BLEND_ALPHA:-0.65}"
  "-p" "depth_enhancement_temporal_alpha:=${DEPTH_ENHANCEMENT_TEMPORAL_ALPHA:-0.45}"
)

if [[ -n "${MP_POSE_TASK_MODEL_PATH:-}" ]]; then
  ARGS+=("-p" "mp_pose_task_model_path:=${MP_POSE_TASK_MODEL_PATH}")
fi

if [[ -n "${EXTRA_ROS_PARAMS:-}" ]]; then
  # shellcheck disable=SC2206
  EXTRA_ARGS=(${EXTRA_ROS_PARAMS})
  ARGS+=("${EXTRA_ARGS[@]}")
fi

log "Starting: ${PYTHON_BIN} ${ARGS[*]}"
exec "${PYTHON_BIN}" "${ARGS[@]}"
