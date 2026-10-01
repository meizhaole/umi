#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
UMI_WORKSPACE="$(cd -- "${WORKSPACE_ROOT}/../universal_manipulation_interface" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
WORLD_FILE="${WORKSPACE_ROOT}/src/rebotarm_bringup/worlds/rebotarm_cup.sdf"
GAZEBO_LAUNCH_PID=""

if [[ -t 1 ]]; then
  COLOR_GREEN=$'\033[0;32m'
  COLOR_YELLOW=$'\033[0;33m'
  COLOR_RED=$'\033[0;31m'
  COLOR_RESET=$'\033[0m'
else
  COLOR_GREEN=""
  COLOR_YELLOW=""
  COLOR_RED=""
  COLOR_RESET=""
fi

info() {
  printf '%b%s%b\n' "${COLOR_GREEN}" "$1" "${COLOR_RESET}"
}

warn() {
  printf '%b%s%b\n' "${COLOR_YELLOW}" "$1" "${COLOR_RESET}"
}

fail() {
  printf '%b错误：%s%b\n' "${COLOR_RED}" "$1" "${COLOR_RESET}" >&2
  exit 1
}

world_is_ready() {
  local services
  services="$(gz service -l 2>/dev/null)" || return 1
  grep -Fxq '/world/empty/create' <<< "${services}" &&
    grep -Fxq '/world/empty/control' <<< "${services}"
}

robot_is_spawned() {
  gz model --list 2>/dev/null | grep -Eq '^[[:space:]]*-[[:space:]]rebot_arm$'
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "${GAZEBO_LAUNCH_PID}" ]] && kill -0 "${GAZEBO_LAUNCH_PID}" 2>/dev/null; then
    info "正在关闭本脚本启动的 Gazebo。"
    kill -INT -- "-${GAZEBO_LAUNCH_PID}" 2>/dev/null || true
    wait "${GAZEBO_LAUNCH_PID}" 2>/dev/null || true
  fi

  exit "${exit_code}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

[[ -r "${ROS_SETUP}" ]] || fail "找不到 ROS 2 Jazzy 环境：${ROS_SETUP}"
[[ -r "${INSTALL_SETUP}" ]] || fail "找不到工作区 install/setup.bash，请先构建 ROS 包。"
[[ -f "${UMI_WORKSPACE}/scripts/umi_sim_replay_worker.py" ]] || fail "找不到 UMI 仿真推理脚本：${UMI_WORKSPACE}"
[[ -r "${WORLD_FILE}" ]] || fail "找不到 Gazebo 场景文件：${WORLD_FILE}"

set +u
source "${ROS_SETUP}"
source "${INSTALL_SETUP}"
set -u
command -v gz >/dev/null 2>&1 || fail "找不到 Gazebo 命令 gz。"
ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 || fail "找不到 ros_gz_sim，请安装 ROS 2 Jazzy 的 ros_gz_sim。"
ros2 pkg prefix gz_ros2_control >/dev/null 2>&1 || fail "缺少 Gazebo 控制插件，请先运行：sudo apt install ros-jazzy-gz-ros2-control"
ros2 pkg prefix rebotarm_agent >/dev/null 2>&1 || fail "找不到 rebotarm_agent，请先构建 ROS 工作区。"

export GZ_SIM_RESOURCE_PATH="${WORKSPACE_ROOT}/src${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

if ! world_is_ready; then
  command -v setsid >/dev/null 2>&1 || fail "找不到 setsid 命令。"
  info "正在启动 ReBot RS 水杯场景。"
  setsid ros2 launch ros_gz_sim gz_sim.launch.py "gz_args:=-r ${WORLD_FILE}" &
  GAZEBO_LAUNCH_PID=$!

  world_ready=false
  for ((attempt = 0; attempt < 60; attempt += 1)); do
    if world_is_ready; then
      world_ready=true
      break
    fi
    if ! kill -0 "${GAZEBO_LAUNCH_PID}" 2>/dev/null; then
      fail "Gazebo 启动进程已退出，请查看上方日志。"
    fi
    sleep 1
  done
  [[ "${world_ready}" == true ]] || fail "等待 Gazebo 的 empty 世界超时。"
else
  if ! gz model --list 2>/dev/null | grep -Eq '^[[:space:]]*-[[:space:]]water_cup$'; then
    fail "当前 Gazebo 是旧空世界。请关闭旧 Gazebo 窗口后重新运行本脚本，加载水杯场景。"
  fi
  warn "检测到已载入水杯场景的 empty 世界，将复用当前 Gazebo。"
fi

if robot_is_spawned; then
  fail "当前世界已有旧版 rebot_arm。请关闭旧 Gazebo，再运行本脚本以加载 ros2_control 插件。"
fi

if ! gz service -s /world/empty/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: false' >/dev/null; then
  fail "无法启动 Gazebo 仿真步进，请检查 empty 世界服务。"
fi

info "启动 ReBot RS 控制器、MoveIt 逆解和官方权重回放。"
ros2 launch rebotarm_moveit_config gazebo_rs_sim.launch.py \
  umi_workspace:="${UMI_WORKSPACE}" \
  episode_index:=-1 \
  start_step:=15 \
  max_chunks:="${MAX_CHUNKS:-1}" \
  device:=cuda