#!/usr/bin/env bash
# Navigation 启停：bash navigation.sh start|stop|status|restart
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="${NAV_RUNTIME_DIR:-$ROOT_DIR/runtime}"
LOG_DIR="$RUNTIME_DIR/logs"
PID_FILE="$RUNTIME_DIR/navigation.pid"
LOG_FILE="$LOG_DIR/navigation.log"
CONFIG_FILE="${NAV_CONFIG:-$ROOT_DIR/config/navigation.json}"
PYTHON_BIN="${PYTHON_BIN:-/home/nvidia/miniconda3/envs/tianji-robot/bin/python}"
LOG_LEVEL="${NAV_LOG_LEVEL:-INFO}"
STOP_TIMEOUT_SEC="${NAV_STOP_TIMEOUT_SEC:-8}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/nvidia/sxy/TianJi/ros2_ws/install/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"

mkdir -p "$LOG_DIR"

log() { echo "[$(date '+%F %T')] $*"; }

pid_alive() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

is_nav_pid() {
  local pid="$1"
  local cmd
  [[ -n "$pid" ]] || return 1
  pid_alive "$pid" || return 1
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  [[ "$cmd" == *" -m navigation"* ]] || [[ "$cmd" == *"navigation.app"* ]]
}

read_pid() {
  [[ -f "$PID_FILE" ]] || return 1
  tr -d ' \t\r\n' <"$PID_FILE"
}

source_ros() {
  if [[ ! -f "$ROS_SETUP" ]]; then
    return 0
  fi
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
  fi
  set -u
  export ROS_DOMAIN_ID
}

do_start() {
  if [[ ! -x "$PYTHON_BIN" ]]; then
    PYTHON_BIN="$(command -v python3 2>/dev/null || true)"
  fi
  if [[ -z "$PYTHON_BIN" || ! -x "$PYTHON_BIN" ]]; then
    log "ERROR: python not found"
    exit 1
  fi
  if [[ ! -f "$CONFIG_FILE" ]]; then
    log "ERROR: config not found: $CONFIG_FILE"
    exit 1
  fi
  do_stop

  source_ros
  cd "$ROOT_DIR"
  export PYTHONPATH="$ROOT_DIR:$ROOT_DIR/apps${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONDONTWRITEBYTECODE=1
  nohup "$PYTHON_BIN" -m navigation --config "$CONFIG_FILE" --log-level "$LOG_LEVEL" \
    >>"$LOG_FILE" 2>&1 &
  local pid=$!
  disown "$pid" || true
  echo "$pid" >"$PID_FILE"
  sleep 0.4
  if ! is_nav_pid "$pid"; then
    log "ERROR: navigation exited immediately; see $LOG_FILE"
    rm -f "$PID_FILE"
    exit 1
  fi
  log "navigation started pid=$pid python=$PYTHON_BIN config=$CONFIG_FILE log=$LOG_FILE"
}

list_nav_pids() {
  pgrep -f -- '-m navigation' 2>/dev/null || true
}

do_stop() {
  local pids left waited
  pids="$(list_nav_pids)"
  if [[ -z "${pids//[$' \t\r\n']/}" ]]; then
    rm -f "$PID_FILE"
    log "navigation stopped"
    return 0
  fi
  log "stopping navigation pid=$(echo "$pids" | tr '\n' ' ')"
  # shellcheck disable=SC2086
  kill -TERM $pids 2>/dev/null || true
  waited=0
  while [[ -n "$(list_nav_pids)" ]] && (( waited < STOP_TIMEOUT_SEC )); do
    sleep 1
    waited=$((waited + 1))
  done
  left="$(list_nav_pids)"
  if [[ -n "${left//[$' \t\r\n']/}" ]]; then
    log "navigation still running, killing pid=$(echo "$left" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill -KILL $left 2>/dev/null || true
    sleep 0.3
  fi
  left="$(list_nav_pids)"
  rm -f "$PID_FILE"
  if [[ -n "${left//[$' \t\r\n']/}" ]]; then
    log "ERROR: navigation still running pid=$(echo "$left" | tr '\n' ' ')"
    exit 1
  fi
  log "navigation stopped"
}

do_status() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if is_nav_pid "$pid"; then
    log "navigation running pid=$pid log=$LOG_FILE"
    ps -p "$pid" -o pid=,etime=,args=
    return 0
  fi
  if [[ -n "$pid" ]]; then
    log "navigation not running (stale pid=$pid)"
    return 1
  fi
  log "navigation not running"
  return 1
}

case "${1:-}" in
  start) do_start ;;
  stop) do_stop ;;
  status) do_status ;;
  restart)
    do_start
    ;;
  *)
    echo "Usage: $0 {start|stop|status|restart}"
    exit 2
    ;;
esac
