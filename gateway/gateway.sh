#!/usr/bin/env bash
# Gateway 启停：bash gateway.sh start|stop|status|restart
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
resolve_runtime_dir() {
  local raw="${GATEWAY_RUNTIME_DIR:-}"
  if [[ -z "$raw" ]]; then
    raw="$ROOT_DIR/runtime"
  fi
  if [[ "$raw" != /* ]]; then
    raw="$ROOT_DIR/$raw"
  fi
  printf '%s\n' "$raw"
}
RUNTIME_DIR="$(resolve_runtime_dir)"
LOG_DIR="$RUNTIME_DIR/logs"
PID_FILE="$RUNTIME_DIR/gateway.pid"
LOG_FILE="$LOG_DIR/gateway.log"
CONFIG_FILE="${GATEWAY_CONFIG:-$ROOT_DIR/config/gateway.json}"
PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if [[ -x /home/admin/miniconda3/envs/smt/bin/python ]]; then
    PYTHON_BIN="/home/admin/miniconda3/envs/smt/bin/python"
  elif [[ -x /home/nvidia/miniconda3/envs/tianji-robot/bin/python ]]; then
    PYTHON_BIN="/home/nvidia/miniconda3/envs/tianji-robot/bin/python"
  else
    PYTHON_BIN="$(command -v python3 || true)"
  fi
fi
LOG_LEVEL="${GATEWAY_LOG_LEVEL:-INFO}"
STOP_TIMEOUT_SEC="${GATEWAY_STOP_TIMEOUT_SEC:-8}"

mkdir -p "$LOG_DIR"

log() { echo "[$(date '+%F %T')] $*"; }

pid_alive() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

is_gateway_pid() {
  local pid="$1"
  local cmd
  [[ -n "$pid" ]] || return 1
  pid_alive "$pid" || return 1
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  [[ "$cmd" == *" -m gateway"* ]] || [[ "$cmd" == *"gateway.app"* ]]
}

read_pid() {
  [[ -f "$PID_FILE" ]] || return 1
  tr -d ' \t\r\n' <"$PID_FILE"
}

find_live_pid() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if is_gateway_pid "$pid"; then
    printf '%s\n' "$pid"
    return 0
  fi
  local live
  live="$(pgrep -f '[p]ython(3)? -m gateway' | head -n 1 || true)"
  if is_gateway_pid "$live"; then
    printf '%s\n' "$live"
    return 0
  fi
  return 1
}

clear_stale_pid() {
  local pid
  pid="$(read_pid 2>/dev/null || true)"
  if [[ -z "$pid" ]]; then
    rm -f "$PID_FILE"
    return 0
  fi
  if is_gateway_pid "$pid"; then
    return 1
  fi
  log "stale pid file pid=$pid"
  rm -f "$PID_FILE"
  return 0
}

do_start() {
  if [[ ! -f "$CONFIG_FILE" ]]; then
    log "ERROR: config not found: $CONFIG_FILE"
    exit 1
  fi
  if [[ ! -x "$PYTHON_BIN" ]]; then
    log "ERROR: python not found: $PYTHON_BIN"
    exit 1
  fi
  if ! clear_stale_pid; then
    log "gateway already running pid=$(read_pid) log=$LOG_FILE"
    exit 0
  fi
  local live
  live="$(find_live_pid || true)"
  if [[ -n "$live" ]]; then
    log "gateway already running pid=$live log=$LOG_FILE"
    exit 0
  fi

  cd "$ROOT_DIR"
  export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONDONTWRITEBYTECODE=1
  # stdin must be /dev/null so a closing SSH session cannot kill the daemon.
  nohup "$PYTHON_BIN" -m gateway --config "$CONFIG_FILE" --log-level "$LOG_LEVEL" \
    < /dev/null >>"$LOG_FILE" 2>&1 &
  local pid=$!
  disown "$pid" 2>/dev/null || true
  echo "$pid" >"$PID_FILE"
  sleep 0.4
  if ! is_gateway_pid "$pid"; then
    log "ERROR: gateway exited immediately; see $LOG_FILE"
    rm -f "$PID_FILE"
    exit 1
  fi
  log "gateway started pid=$pid python=$PYTHON_BIN config=$CONFIG_FILE log=$LOG_FILE"
}

do_stop() {
  local pid
  pid="$(find_live_pid || true)"
  if [[ -z "$pid" ]]; then
    rm -f "$PID_FILE"
    log "gateway not running"
    return 0
  fi

  log "stopping gateway pid=$pid"
  kill -TERM "$pid" 2>/dev/null || true
  local waited=0
  while pid_alive "$pid" && (( waited < STOP_TIMEOUT_SEC )); do
    sleep 0.5
    waited=$((waited + 1))
  done
  if pid_alive "$pid"; then
    log "gateway did not exit in ${STOP_TIMEOUT_SEC}s, sending KILL pid=$pid"
    kill -KILL "$pid" 2>/dev/null || true
    sleep 0.2
  fi
  rm -f "$PID_FILE"
  if pid_alive "$pid"; then
    log "ERROR: gateway still running pid=$pid"
    exit 1
  fi
  log "gateway stopped pid=$pid"
}

do_status() {
  local pid
  pid="$(find_live_pid || true)"
  if [[ -n "$pid" ]]; then
    log "gateway running pid=$pid log=$LOG_FILE"
    ps -p "$pid" -o pid=,etime=,args=
    return 0
  fi
  local stale
  stale="$(read_pid 2>/dev/null || true)"
  if [[ -n "$stale" ]]; then
    log "gateway not running (stale pid=$stale)"
    return 1
  fi
  log "gateway not running"
  return 1
}

usage() {
  echo "Usage: $0 {start|stop|status|restart|exec}"
}

do_exec() {
  if [[ ! -f "$CONFIG_FILE" ]]; then
    log "ERROR: config not found: $CONFIG_FILE"
    exit 1
  fi
  if [[ ! -x "$PYTHON_BIN" ]]; then
    log "ERROR: python not found: $PYTHON_BIN"
    exit 1
  fi
  cd "$ROOT_DIR"
  export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONUNBUFFERED=1
  export GATEWAY_RUNTIME_DIR="$RUNTIME_DIR"
  exec "$PYTHON_BIN" -m gateway --config "$CONFIG_FILE" --log-level "$LOG_LEVEL"
}

case "${1:-}" in
  start) do_start ;;
  stop) do_stop ;;
  status) do_status ;;
  exec) do_exec ;;
  restart)
    do_stop
    do_start
    ;;
  *)
    usage
    exit 2
    ;;
esac
