#!/usr/bin/env bash
# Management script for production 25540 (SKU spatial localization service).
#
# Subcommands:
#   start     [-] launch the service detached if not already running
#   stop      [-] gracefully stop the running service (SIGTERM, waits for port release)
#   restart   [-] stop + start
#   status    [-] show PID, uptime, health snapshot
#   --foreground  run in the foreground (manual debugging; same env as `start`)
#
# 2026-09-24: env block updated to single-card serial production configuration
# (SAM3 multipart_segment backend via SAM3_URL,
# FIT_BOOTSTRAP_BOOTS=4, FIT_BOOTSTRAP_WORKERS=6, FIT_ANALYTIC_JAC=1,
# FIT_MAIN_FIT_POOL=1, FRONT_PANEL_RANSAC_MAX_ITER=200 + SKIP_MEDIAN, BLAS pins).
# Launch method is detached (setsid nohup, parent=1) matching how the service runs
# since it moved off tmux; the old tmux branch is intentionally gone.
set -eu

ROOT="/home/quinn/cosmetics_pose"
DEPLOY="$ROOT/deploy"
VENV_PY="$DEPLOY/.venv/bin/python"
PORT=25540
LOG_DIR="$DEPLOY/logs"
LOG="$LOG_DIR/25540_service.log"
PIDFILE="$LOG_DIR/25540_service.pid"
HEALTH_URL="http://127.0.0.1:$PORT/health"

apply_env() {
    export MPLBACKEND=Agg
    export SAM3_BACKEND=multipart_segment
    export SAM3_URL=http://127.0.0.1:25541/api/v1/segment
    export SAM3_MASK_THRESHOLD=0.5
    export SAM3_TIMEOUT_S=180
    export BASKET_FP_REGISTERED_CAD=1
    export FIT_SCRIPT="$DEPLOY/algorithms/fit_bottle_axis.py"
    export ESTEE_FIT_SCRIPT="$DEPLOY/algorithms/fit_estee_box_top_surface.py"
    export TUBE_FIT_SCRIPT="$DEPLOY/algorithms/fit_tube_top_edge.py"
    export FRONT_PANEL_FIT_SCRIPT="$DEPLOY/algorithms/fit_front_panel_plane.py"
    export BOX_SELECTION_SCRIPT="$DEPLOY/pipeline/box_selection.py"
    export BOX_GEOMETRIC_SCRIPT="${BOX_GEOMETRIC_SCRIPT:-$DEPLOY/algorithms/box_geometric_center.py}"
    export DEFAULT_BOX_PROMPT="${DEFAULT_BOX_PROMPT:-each individual open cardboard box}"
    export AXIS_SERVICE_OUTPUT="$ROOT/requests"
    export FIT_BOOTSTRAP_WORKERS=6
    export FIT_BOOTSTRAP_BOOTS=4
    export FIT_ANALYTIC_JAC=1
    export FIT_MAIN_FIT_POOL=1
    export FRONT_PANEL_RANSAC_MAX_ITER=200
    export FRONT_PANEL_RANSAC_SKIP_MEDIAN=1
    export ENABLE_3D_RENDER=1
    export ASYNC_3D_RENDER=1
    export OMP_NUM_THREADS=1
    export OPENBLAS_NUM_THREADS=1
    export MKL_NUM_THREADS=1
    export NUMEXPR_NUM_THREADS=1
    export VECLIB_MAXIMUM_THREADS=1
}

listener_pid() {
    ss -tlnp 2>/dev/null | grep ":$PORT " | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -1
}

port_busy() {
    ss -tln 2>/dev/null | grep -q ":$PORT "
}

wait_health() {
    local i
    for i in $(seq 1 40); do
        sleep 1
        if curl -sf -m 5 "$HEALTH_URL" > /dev/null 2>&1; then
            return 0
        fi
    done
    return 1
}

do_start() {
    local PID
    PID=$(listener_pid || true)
    if [ -n "$PID" ]; then
        echo "already running: pid=$PID port=$PORT"
        curl -sS -m 5 "$HEALTH_URL" | head -c 120 || true
        echo
        return 0
    fi
    mkdir -p "$LOG_DIR"
    apply_env
    cd /home/quinn
    setsid nohup "$VENV_PY" "$DEPLOY/server.py" --host 0.0.0.0 --port "$PORT" >> "$LOG" 2>&1 &
    echo $! > "$PIDFILE"
    if wait_health; then
        PID=$(listener_pid || true)
        echo "started: pid=$PID port=$PORT log=$LOG"
        curl -sS -m 5 "$HEALTH_URL" | head -c 120 || true
        echo
    else
        echo "ERROR: health check failed after start (see $LOG)" >&2
        exit 1
    fi
}

do_stop() {
    local PID
    PID=$(listener_pid || true)
    if [ -z "$PID" ]; then
        echo "not running (port $PORT free)"
        rm -f "$PIDFILE"
        return 0
    fi
    echo "stopping pid=$PID ..."
    kill -TERM "$PID" 2>/dev/null || true
    local i st
    for i in $(seq 1 15); do
        st=$(ps -o stat= -p "$PID" 2>/dev/null | tr -d ' ') || st=""
        [ -z "$st" ] && break
        case "$st" in Z*) break;; esac
        sleep 1
    done
    if port_busy; then
        echo "ERROR: port $PORT still busy after stop" >&2
        exit 1
    fi
    rm -f "$PIDFILE"
    echo "stopped; port $PORT free"
}

do_status() {
    local PID
    PID=$(listener_pid || true)
    if [ -z "$PID" ]; then
        echo "status: NOT RUNNING (port $PORT free)"
        return 0
    fi
    echo "status: RUNNING pid=$PID start=$(ps -o lstart= -p "$PID" 2>/dev/null)"
    echo "  cmd: $(ps -o args= -p "$PID" 2>/dev/null)"
    curl -sS -m 5 "$HEALTH_URL" | head -c 200 || true
    echo
}

case "${1:-start}" in
    start)
        do_start
        ;;
    stop)
        do_stop
        ;;
    restart)
        do_stop
        do_start
        ;;
    status)
        do_status
        ;;
    --foreground)
        apply_env
        cd "$DEPLOY"
        exec "$VENV_PY" "$DEPLOY/server.py" --host 0.0.0.0 --port "$PORT"
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|--foreground}" >&2
        exit 1
        ;;
esac
