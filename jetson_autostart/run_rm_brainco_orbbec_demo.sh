#!/usr/bin/env bash
set -Eeuo pipefail

ENV_FILE="${RM_BRAINCO_ORBBEC_ENV_FILE:-/etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env}"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

APP_ROOT="${APP_ROOT:-/home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT}"
CONTROL_SCRIPT="${CONTROL_SCRIPT:-${APP_ROOT}/scripts/camera_arm_rm65b_control_rtmpose.py}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

log() { printf '[rm_brainco_orbbec_demo] %s\n' "$*"; }

bool_is_true() {
  case "${1:-false}" in
    true|True|TRUE|1|yes|YES|y|Y) return 0 ;;
    *) return 1 ;;
  esac
}

wait_for_ping() {
  local host="$1" timeout_s="${2:-45}" start now
  start="$(date +%s)"
  while ! ping -c 1 -W 1 "${host}" >/dev/null 2>&1; do
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      log "Timed out waiting for robot at ${host}"
      return 1
    fi
    sleep 1
  done
}

wait_for_path() {
  local path="$1" timeout_s="${2:-20}" start now
  start="$(date +%s)"
  while [[ ! -e "${path}" ]]; do
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
  exit 2
fi

if [[ ! -f "${CONTROL_SCRIPT}" ]]; then
  log "Control script not found: ${CONTROL_SCRIPT}"
  exit 2
fi

for model_path in \
  "${RTMDET_ENGINE_PATH:-}" \
  "${RTMPOSE_BODY_ENGINE_PATH:-}" \
  "${RTMPOSE_HAND_ENGINE_PATH:-}"; do
  if [[ -z "${model_path}" || ! -f "${model_path}" ]]; then
    log "TensorRT engine not found: ${model_path:-<empty>}"
    exit 2
  fi
done

export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONPATH="${EXTRA_PYTHONPATH:-}:${BRAINCO_HAND_SDK_PATH:-}:${APP_ROOT}:${APP_ROOT}/scripts:${PYTHONPATH:-}"

cd "${APP_ROOT}"

if bool_is_true "${WAIT_FOR_ROBOT_IP:-true}" && bool_is_true "${ROBOT_MOTION_ENABLED:-true}"; then
  wait_for_ping "${ROBOT_IP:-192.168.1.18}" "${WAIT_FOR_ROBOT_TIMEOUT_S:-45}"
fi

if bool_is_true "${WAIT_FOR_HAND_RS485_PORT:-false}" \
  && [[ "${HAND_TRANSPORT:-brainco-rs485}" == "brainco-rs485" ]] \
  && [[ "${HAND_RS485_PORT:-auto}" != "auto" ]]; then
  wait_for_path "${HAND_RS485_PORT}" "${WAIT_FOR_HAND_RS485_TIMEOUT_S:-20}"
fi

ARGS=("${CONTROL_SCRIPT}" "--ros-args")
add_param() { ARGS+=("-p" "$1:=$2"); }

add_param vision_backend "${VISION_BACKEND:-rtmpose}"
add_param rtmpose_runtime "${RTMPOSE_RUNTIME:-tensorrt}"
add_param rtmpose_device_name "${RTMPOSE_DEVICE_NAME:-cuda}"
add_param rtmpose_device_id "${RTMPOSE_DEVICE_ID:-0}"
add_param rtmdet_engine_path "${RTMDET_ENGINE_PATH}"
add_param rtmpose_body_engine_path "${RTMPOSE_BODY_ENGINE_PATH}"
add_param rtmpose_hand_engine_path "${RTMPOSE_HAND_ENGINE_PATH}"
add_param rtmpose_body_keypoint_threshold "${RTMPOSE_BODY_KEYPOINT_THRESHOLD:-0.25}"
add_param rtmpose_hand_keypoint_threshold "${RTMPOSE_HAND_KEYPOINT_THRESHOLD:-0.25}"
add_param rtmpose_hand_keypoint_tracking_threshold "${RTMPOSE_HAND_KEYPOINT_TRACKING_THRESHOLD:-0.06}"
add_param rtmpose_hand_keypoint_hold_frames "${RTMPOSE_HAND_KEYPOINT_HOLD_FRAMES:-3}"
add_param rtmpose_hand_keypoint_smoothing "${RTMPOSE_HAND_KEYPOINT_SMOOTHING:-0.18}"
add_param rtmpose_hand_keypoint_fast_motion_px "${RTMPOSE_HAND_KEYPOINT_FAST_MOTION_PX:-20.0}"
add_param rtmpose_hand_roi_scale "${RTMPOSE_HAND_ROI_SCALE:-2.25}"
add_param rtmpose_hand_roi_wrist_extension "${RTMPOSE_HAND_ROI_WRIST_EXTENSION:-0.35}"
add_param rtmpose_hand_tracking_roi_scale "${RTMPOSE_HAND_TRACKING_ROI_SCALE:-1.70}"
add_param rtmpose_hand_roi_min_size_px "${RTMPOSE_HAND_ROI_MIN_SIZE_PX:-96}"
add_param rtmpose_hand_roi_max_size_px "${RTMPOSE_HAND_ROI_MAX_SIZE_PX:-700}"
add_param rtmpose_hand_inference_interval "${RTMPOSE_HAND_INFERENCE_INTERVAL:-1}"

add_param finger_segmentation_enabled "${FINGER_SEGMENTATION_ENABLED:-true}"
add_param finger_segmentation_corridor_width_ratio "${FINGER_SEGMENTATION_CORRIDOR_WIDTH_RATIO:-0.08}"
add_param finger_segmentation_color_mad_scale "${FINGER_SEGMENTATION_COLOR_MAD_SCALE:-4.0}"
add_param finger_segmentation_minimum_chroma_tolerance "${FINGER_SEGMENTATION_MINIMUM_CHROMA_TOLERANCE:-9.0}"
add_param finger_segmentation_debug_overlay "${FINGER_SEGMENTATION_DEBUG_OVERLAY:-true}"
add_param finger_segmentation_fusion_enabled "${FINGER_SEGMENTATION_FUSION_ENABLED:-true}"
add_param finger_segmentation_min_confidence "${FINGER_SEGMENTATION_MIN_CONFIDENCE:-0.45}"
add_param finger_segmentation_straight_threshold "${FINGER_SEGMENTATION_STRAIGHT_THRESHOLD:-0.78}"
add_param finger_segmentation_full_straight_threshold "${FINGER_SEGMENTATION_FULL_STRAIGHT_THRESHOLD:-0.92}"
add_param finger_segmentation_bend_cap "${FINGER_SEGMENTATION_BEND_CAP:-0.12}"

add_param control_hand_side "${CONTROL_HAND_SIDE:-right}"
add_param control_hand_fallback_enabled "${CONTROL_HAND_FALLBACK_ENABLED:-false}"
add_param operator_require_torso_pose "${OPERATOR_REQUIRE_TORSO_POSE:-true}"
add_param operator_ready_confirm_frames "${OPERATOR_READY_CONFIRM_FRAMES:-12}"
add_param person_lost_timeout_s "${PERSON_LOST_TIMEOUT_S:-0.80}"
add_param depth_enhancement_enabled "${DEPTH_ENHANCEMENT_ENABLED:-false}"
add_param nn_skip_arm_angle_log "${NN_SKIP_ARM_ANGLE_LOG:-true}"

add_param robot_motion_enabled "${ROBOT_MOTION_ENABLED:-true}"
add_param robot_ip "${ROBOT_IP:-192.168.1.18}"
add_param robot_port "${ROBOT_PORT:-8080}"
add_param robot_level "${ROBOT_LEVEL:-3}"
add_param robot_thread_mode "${ROBOT_THREAD_MODE:-2}"
add_param arm_control_protocol "${ARM_CONTROL_PROTOCOL:-canfd}"
add_param canfd_follow "${CANFD_FOLLOW:-false}"
add_param canfd_trajectory_mode "${CANFD_TRAJECTORY_MODE:-2}"
add_param canfd_radio "${CANFD_RADIO:-50}"
add_param arm_speed "${ARM_SPEED:-5}"
add_param arm_block "${ARM_BLOCK:-0}"
add_param arm_recover_motion_state_on_start "${ARM_RECOVER_MOTION_STATE_ON_START:-false}"
add_param arm_recover_motion_state_on_tracking_start "${ARM_RECOVER_MOTION_STATE_ON_TRACKING_START:-false}"
add_param arm_slow_stop_on_pause "${ARM_SLOW_STOP_ON_PAUSE:-true}"
add_param arm_log_command_start "${ARM_LOG_COMMAND_START:-true}"
add_param auto_home_on_start "${AUTO_HOME_ON_START:-false}"
add_param home_key_enabled "${HOME_KEY_ENABLED:-true}"
add_param safe_home_joints "${SAFE_HOME_JOINTS:-[-1.341,-91.165,101.410,-13.002,23.072,-0.046]}"
add_param joint_safety_enabled "${JOINT_SAFETY_ENABLED:-true}"
add_param target_ik_safety_enabled "${TARGET_IK_SAFETY_ENABLED:-true}"
add_param singularity_safety_enabled "${SINGULARITY_SAFETY_ENABLED:-true}"
add_param wrist_singularity_joint5_min_abs_deg "${WRIST_SINGULARITY_JOINT5_MIN_ABS_DEG:-15.0}"
add_param joint_limit_margin_deg "${JOINT_LIMIT_MARGIN_DEG:-3.0}"
add_param target_safety_check_interval_s "${TARGET_SAFETY_CHECK_INTERVAL_S:-0.20}"
add_param target_safety_allow_cached_on_read_error "${TARGET_SAFETY_ALLOW_CACHED_ON_READ_ERROR:-false}"

add_param arm_workspace_scan_enabled "${ARM_WORKSPACE_SCAN_ENABLED:-true}"
add_param arm_workspace_scan_max_distance_m "${ARM_WORKSPACE_SCAN_MAX_DISTANCE_M:-[0.600,0.600,0.600]}"
add_param arm_workspace_scan_step_m "${ARM_WORKSPACE_SCAN_STEP_M:-0.010}"
add_param arm_workspace_scan_refine_iterations "${ARM_WORKSPACE_SCAN_REFINE_ITERATIONS:-6}"
add_param arm_workspace_scan_safety_ratio "${ARM_WORKSPACE_SCAN_SAFETY_RATIO:-0.80}"
add_param arm_workspace_scan_min_effective_m "${ARM_WORKSPACE_SCAN_MIN_EFFECTIVE_M:-0.005}"
add_param arm_state_read_retries "${ARM_STATE_READ_RETRIES:-5}"
add_param arm_state_read_retry_interval_s "${ARM_STATE_READ_RETRY_INTERVAL_S:-0.20}"

add_param camera_to_arm_matrix "${CAMERA_TO_ARM_MATRIX:-[0.0,0.0,1.0,-1.0,0.0,0.0,0.0,-1.0,0.0]}"
add_param palm_to_arm_scale "${PALM_TO_ARM_SCALE:-0.20}"
add_param arm_axis_scale "${ARM_AXIS_SCALE:-[1.2,1.6,1.2]}"
add_param arm_axis_deadband_m "${ARM_AXIS_DEADBAND_M:-[0.004,0.005,0.004]}"
add_param hand_deadband_m "${HAND_DEADBAND_M:-0.006}"
add_param palm_filter_enabled "${PALM_FILTER_ENABLED:-true}"
add_param palm_filter_alpha "${PALM_FILTER_ALPHA:-0.25}"
add_param palm_filter_still_alpha "${PALM_FILTER_STILL_ALPHA:-0.05}"
add_param palm_filter_hold_deadband_m "${PALM_FILTER_HOLD_DEADBAND_M:-0.008}"
add_param palm_filter_release_deadband_m "${PALM_FILTER_RELEASE_DEADBAND_M:-0.014}"
add_param palm_filter_velocity_deadband_m_s "${PALM_FILTER_VELOCITY_DEADBAND_M_S:-0.025}"
add_param arm_motion_unit_m "${ARM_MOTION_UNIT_M:-0.001}"
add_param max_arm_axis_offset_m "${MAX_ARM_AXIS_OFFSET_M:-[0.180,0.200,0.180]}"
add_param arm_axis_max_velocity_m_s "${ARM_AXIS_MAX_VELOCITY_M_S:-[0.150,0.180,0.150]}"
add_param arm_axis_max_accel_m_s2 "${ARM_AXIS_MAX_ACCEL_M_S2:-[0.180,0.220,0.180]}"
add_param max_arm_target_delta_axis_m "${MAX_ARM_TARGET_DELTA_AXIS_M:-[0.004,0.005,0.004]}"
add_param min_command_delta_m "${MIN_COMMAND_DELTA_M:-0.0015}"
add_param arm_command_interval_s "${ARM_COMMAND_INTERVAL_S:-0.12}"
add_param arm_mit_control_enabled "${ARM_MIT_CONTROL_ENABLED:-true}"
add_param arm_mit_kp "${ARM_MIT_KP:-35.0}"
add_param arm_mit_kd "${ARM_MIT_KD:-16.0}"

add_param hand_control_enabled "${HAND_CONTROL_ENABLED:-true}"
add_param hand_transport "${HAND_TRANSPORT:-brainco-rs485}"
add_param hand_rs485_sdk_path "${BRAINCO_HAND_SDK_PATH:-/home/jetson/ws/brainco-hand-sdk-main/python}"
add_param hand_rs485_port "${HAND_RS485_PORT:-auto}"
add_param hand_rs485_slave_id "${HAND_RS485_SLAVE_ID:-0}"
add_param hand_rs485_baudrate "${HAND_RS485_BAUDRATE:-460800}"
add_param hand_rs485_duration_ms "${HAND_RS485_DURATION_MS:-200}"
add_param hand_rs485_speed "${HAND_RS485_SPEED:-300}"
add_param hand_position_min "${HAND_POSITION_MIN:-0}"
add_param hand_position_max "${HAND_POSITION_MAX:-1000}"
add_param hand_bend_calibration_enabled "${HAND_BEND_CALIBRATION_ENABLED:-true}"
add_param hand_bend_open_calibration "${HAND_BEND_OPEN_CALIBRATION:-[0.18,0.08,0.13,0.09,0.13]}"
add_param hand_bend_closed_calibration "${HAND_BEND_CLOSED_CALIBRATION:-[0.56,0.92,0.91,0.86,0.87]}"
add_param hand_bend_gamma "${HAND_BEND_GAMMA:-[1.0,1.0,1.0,1.0,1.0]}"
add_param hand_bend_deadzone "${HAND_BEND_DEADZONE:-0.12}"
add_param hand_bend_release_threshold "${HAND_BEND_RELEASE_THRESHOLD:-0.05}"
add_param hand_bend_jitter_deadband "${HAND_BEND_JITTER_DEADBAND:-0.025}"
add_param hand_send_hz "${HAND_SEND_HZ:-10.0}"
add_param hand_smoothing "${HAND_SMOOTHING:-0.50}"
add_param hand_bend_smoothing "${HAND_BEND_SMOOTHING:-0.60}"
add_param hand_bend_max_delta "${HAND_BEND_MAX_DELTA:-0.30}"
add_param hand_deadband "${HAND_DEADBAND:-10}"
add_param hand_require_3d_bends_for_control "${HAND_REQUIRE_3D_BENDS_FOR_CONTROL:-true}"

add_param hand_thumb_rotate_mode "${HAND_THUMB_ROTATE_MODE:-opposition}"
add_param hand_thumb_opposition_open_ratio "${HAND_THUMB_OPPOSITION_OPEN_RATIO:-1.05}"
add_param hand_thumb_opposition_closed_ratio "${HAND_THUMB_OPPOSITION_CLOSED_RATIO:-0.35}"
add_param hand_thumb_opposition_gamma "${HAND_THUMB_OPPOSITION_GAMMA:-1.0}"
add_param hand_thumb_rotate_scale "${HAND_THUMB_ROTATE_SCALE:-0.35}"
add_param hand_thumb_straight_lock_enabled "${HAND_THUMB_STRAIGHT_LOCK_ENABLED:-true}"
add_param hand_thumb_auto_zero_enabled "${HAND_THUMB_AUTO_ZERO_ENABLED:-true}"
add_param hand_thumb_auto_zero_samples "${HAND_THUMB_AUTO_ZERO_SAMPLES:-5}"
add_param hand_thumb_auto_zero_margin "${HAND_THUMB_AUTO_ZERO_MARGIN:-0.05}"
add_param hand_thumb_bend_engage_threshold "${HAND_THUMB_BEND_ENGAGE_THRESHOLD:-0.25}"
add_param hand_thumb_bend_release_threshold "${HAND_THUMB_BEND_RELEASE_THRESHOLD:-0.08}"
add_param hand_thumb_rotate_engage_threshold "${HAND_THUMB_ROTATE_ENGAGE_THRESHOLD:-0.32}"
add_param hand_thumb_rotate_release_threshold "${HAND_THUMB_ROTATE_RELEASE_THRESHOLD:-0.12}"
add_param hand_thumb_state_confirm_frames "${HAND_THUMB_STATE_CONFIRM_FRAMES:-3}"
add_param hand_thumb_jitter_deadband "${HAND_THUMB_JITTER_DEADBAND:-0.05}"

add_param hand_reject_single_finger_spikes "${HAND_REJECT_SINGLE_FINGER_SPIKES:-true}"
add_param hand_single_finger_spike_delta "${HAND_SINGLE_FINGER_SPIKE_DELTA:-0.22}"
add_param hand_single_finger_spike_peer_max "${HAND_SINGLE_FINGER_SPIKE_PEER_MAX:-0.22}"
add_param hand_spike_confirm_frames "${HAND_SPIKE_CONFIRM_FRAMES:-2}"
add_param hand_reject_multi_finger_spikes "${HAND_REJECT_MULTI_FINGER_SPIKES:-true}"
add_param hand_multi_finger_spike_delta "${HAND_MULTI_FINGER_SPIKE_DELTA:-0.30}"
add_param hand_multi_finger_spike_min_count "${HAND_MULTI_FINGER_SPIKE_MIN_COUNT:-2}"
add_param hand_multi_finger_spike_confirm_frames "${HAND_MULTI_FINGER_SPIKE_CONFIRM_FRAMES:-2}"
add_param hand_closure_confirm_frames "${HAND_CLOSURE_CONFIRM_FRAMES:-2}"
add_param hand_closure_confirm_delta "${HAND_CLOSURE_CONFIRM_DELTA:-0.25}"
add_param hand_closure_confirm_value "${HAND_CLOSURE_CONFIRM_VALUE:-0.45}"
add_param hand_closure_confirm_min_fingers "${HAND_CLOSURE_CONFIRM_MIN_FINGERS:-2}"
add_param hand_open_on_start "${HAND_OPEN_ON_START:-true}"
add_param hand_open_on_home "${HAND_OPEN_ON_HOME:-true}"
add_param hand_open_on_lost "${HAND_OPEN_ON_LOST:-true}"
add_param hand_lost_open_delay_s "${HAND_LOST_OPEN_DELAY_S:-1.0}"
add_param hand_open_on_exit "${HAND_OPEN_ON_EXIT:-true}"
add_param hand_configure_tool_voltage "${HAND_CONFIGURE_TOOL_VOLTAGE:-false}"

add_param color_width "${COLOR_WIDTH:-1280}"
add_param color_height "${COLOR_HEIGHT:-720}"
add_param color_fps "${COLOR_FPS:-30}"
add_param depth_width "${DEPTH_WIDTH:-0}"
add_param depth_height "${DEPTH_HEIGHT:-0}"
add_param depth_fps "${DEPTH_FPS:-0}"
add_param auto_start_on_ready "${AUTO_START_ON_READY:-false}"
add_param auto_start_delay_s "${AUTO_START_DELAY_S:-1.0}"
add_param show_window "${SHOW_WINDOW:-true}"
add_param display_max_width "${DISPLAY_MAX_WIDTH:-1280}"
add_param display_max_height "${DISPLAY_MAX_HEIGHT:-720}"

if [[ -n "${EXTRA_ROS_PARAMS:-}" ]]; then
  # shellcheck disable=SC2206
  EXTRA_ARGS=(${EXTRA_ROS_PARAMS})
  ARGS+=("${EXTRA_ARGS[@]}")
fi

log "Starting current RM65-B + BrainCo visual controller"
log "Home joints: ${SAFE_HOME_JOINTS:-[-1.341,-91.165,101.410,-13.002,23.072,-0.046]}"
log "Controls: S=start/resume, H=Home and pause, Q/Esc=quit"
exec "${PYTHON_BIN}" "${ARGS[@]}"
