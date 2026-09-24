#!/usr/bin/env bash
# Vision 启停：bash vision.sh start|stop|status|restart
# 默认同时拉起 camera(:8003) 与 media(:8005)
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="${VISION_RUNTIME_DIR:-$ROOT_DIR/runtime}"
LOG_DIR="$RUNTIME_DIR/logs"
CONFIG_FILE="${VISION_CONFIG:-$ROOT_DIR/config/vision.json}"
# Prefer active conda env；可用 PYTHON_BIN 覆盖
if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
    PYTHON_BIN="${CONDA_PREFIX}/bin/python"
  else
    PYTHON_BIN="python3"
  fi
fi
LOG_LEVEL="${VISION_LOG_LEVEL:-INFO}"
STOP_TIMEOUT_SEC="${VISION_STOP_TIMEOUT_SEC:-8}"
START_MEDIA="${VISION_START_MEDIA:-1}"
START_ROKAE_OWNER="${VISION_START_ROKAE_OWNER:-auto}"

mkdir -p "$LOG_DIR"

log() { echo "[$(date '+%F %T')] $*"; }

pid_alive() {
  local pid="$1" state
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -p "$pid" -o stat= 2>/dev/null)" || return 1
  [[ -n "$state" && "$state" != Z* ]]
}

is_mod_pid() {
  local pid="$1" needle="$2"
  local cmd
  [[ -n "$pid" ]] || return 1
  pid_alive "$pid" || return 1
  cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  [[ "$cmd" == *"$needle"* ]]
}

read_pid_file() {
  local file="$1"
  [[ -f "$file" ]] || return 1
  tr -d ' \t\r\n' <"$file"
}

clear_stale() {
  local file="$1" needle="$2"
  local pid
  pid="$(read_pid_file "$file" 2>/dev/null || true)"
  if [[ -z "$pid" ]]; then
    rm -f "$file"
    return 0
  fi
  if is_mod_pid "$pid" "$needle"; then
    return 1
  fi
  log "stale pid file pid=$pid file=$file"
  rm -f "$file"
  return 0
}

start_one() {
  local name="$1" module="$2" pid_file="$3" log_file="$4" needle="$5"
  if ! clear_stale "$pid_file" "$needle"; then
    log "$name already running pid=$(read_pid_file "$pid_file") log=$log_file"
    return 0
  fi
  cd "$ROOT_DIR"
  export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONDONTWRITEBYTECODE=1
  nohup "$PYTHON_BIN" -m "$module" --config "$CONFIG_FILE" --log-level "$LOG_LEVEL" \
    >>"$log_file" 2>&1 &
  local pid=$!
  disown "$pid" || true
  echo "$pid" >"$pid_file"
  sleep 0.4
  if ! is_mod_pid "$pid" "$needle"; then
    log "ERROR: $name exited immediately; see $log_file"
    rm -f "$pid_file"
    return 1
  fi
  log "$name started pid=$pid python=$PYTHON_BIN log=$log_file"
}

stop_one() {
  local name="$1" pid_file="$2" needle="$3"
  local pid
  pid="$(read_pid_file "$pid_file" 2>/dev/null || true)"
  if [[ -z "$pid" ]] || ! is_mod_pid "$pid" "$needle"; then
    rm -f "$pid_file"
    log "$name not running"
    return 0
  fi
  log "stopping $name pid=$pid"
  kill -TERM "$pid" 2>/dev/null || true
  local deadline=$((SECONDS + STOP_TIMEOUT_SEC))
  while pid_alive "$pid" && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if pid_alive "$pid"; then
    kill -KILL "$pid" 2>/dev/null || true
    sleep 0.2
  fi
  rm -f "$pid_file"
  if pid_alive "$pid"; then
    log "ERROR: $name still running pid=$pid"
    exit 1
  fi
  log "$name stopped pid=$pid"
}

config_adapter_is_rokae() {
  "$PYTHON_BIN" - "$CONFIG_FILE" <<'PY'
import json,sys
try:
    cfg=json.load(open(sys.argv[1], encoding="utf-8"))
except Exception:
    sys.exit(1)
sys.exit(0 if str(cfg.get("adapter") or "").strip().lower()=="rokae" else 1)
PY
}

should_start_rokae_owner() {
  case "$START_ROKAE_OWNER" in
    1|true|TRUE|yes|YES) return 0 ;;
    0|false|FALSE|no|NO) return 1 ;;
    auto|AUTO|"")
      config_adapter_is_rokae
      return $?
      ;;
    *) return 1 ;;
  esac
}

do_start() {
  local resolved_python
  if ! resolved_python="$(command -v -- "$PYTHON_BIN")" || [[ ! -x "$resolved_python" ]]; then
    log "ERROR: python not found: $PYTHON_BIN"
    return 1
  fi
  PYTHON_BIN="$resolved_python"
  if should_start_rokae_owner; then
    start_one rokae_owner vision.rokae_runtime "$RUNTIME_DIR/rokae_owner.pid" "$LOG_DIR/rokae_owner.log" "vision.rokae_runtime" || exit 1
  fi
  start_one camera vision.camera_app "$RUNTIME_DIR/camera.pid" "$LOG_DIR/camera.log" "vision.camera_app" || exit 1
  if [[ "$START_MEDIA" == "1" ]]; then
    start_one media vision.media_app "$RUNTIME_DIR/media.pid" "$LOG_DIR/media.log" "vision.media_app" || exit 1
  fi
  log "if camera port is busy, stop the process that holds it, then restart"
}

do_stop() {
  stop_one media "$RUNTIME_DIR/media.pid" "vision.media_app"
  stop_one camera "$RUNTIME_DIR/camera.pid" "vision.camera_app"
  stop_one rokae_owner "$RUNTIME_DIR/rokae_owner.pid" "vision.rokae_runtime"
}

do_status() {
  local ok=1
  local pid
  pid="$(read_pid_file "$RUNTIME_DIR/rokae_owner.pid" 2>/dev/null || true)"
  if is_mod_pid "$pid" "vision.rokae_runtime"; then
    log "rokae_owner running pid=$pid log=$LOG_DIR/rokae_owner.log"
    ps -p "$pid" -o pid=,etime=,args=
  else
    log "rokae_owner not running"
    if should_start_rokae_owner; then ok=0; fi
  fi
  pid="$(read_pid_file "$RUNTIME_DIR/camera.pid" 2>/dev/null || true)"
  if is_mod_pid "$pid" "vision.camera_app"; then
    log "camera running pid=$pid log=$LOG_DIR/camera.log"
    ps -p "$pid" -o pid=,etime=,args=
  else
    log "camera not running"
    ok=0
  fi
  pid="$(read_pid_file "$RUNTIME_DIR/media.pid" 2>/dev/null || true)"
  if is_mod_pid "$pid" "vision.media_app"; then
    log "media running pid=$pid log=$LOG_DIR/media.log"
    ps -p "$pid" -o pid=,etime=,args=
  else
    log "media not running"
    [[ "$START_MEDIA" == "1" ]] && ok=0
  fi
  return $((1 - ok))
}

case "${1:-}" in
  start) do_start ;;
  stop) do_stop ;;
  status) do_status ;;
  restart)
    do_stop
    do_start
    ;;
  *)
    echo "Usage: $0 {start|stop|status|restart}"
    exit 2
    ;;
esac
