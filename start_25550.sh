#!/usr/bin/env bash
# Start FoundationPose HTTP service on GPU 0.
set -eu

ACTION="${1:-start}"
PORT="${PORT:-25550}"
SAM3_API_URL="${SAM3_API_URL:-http://127.0.0.1:25551/infer}"

ROOT="${FOUNDATIONPOSE_ROOT:-/data/quinn/foundationpose}"
DEPLOY="$ROOT/current/deploy"
ENV_PY="${FOUNDATIONPOSE_PYTHON:-$ROOT/env/bin/python}"
LOG_DIR="$ROOT/logs"
PIDFILE="$LOG_DIR/foundationpose_${PORT}.pid"
LOG="$LOG_DIR/foundationpose_${PORT}.log"
CAD_MESH="${CAD_MESH:-$ROOT/cad/Basket/Basket.obj}"
HEALTH_URL="http://127.0.0.1:$PORT/health"
SERVER_COMMAND=("$ENV_PY" "$DEPLOY/http_server.py"
    --host 0.0.0.0 --port "$PORT"
    --mesh-file "$CAD_MESH" --mesh-scale 0.001)

apply_env() {
    [[ -x "$ENV_PY" ]] || { echo "Python not executable: $ENV_PY" >&2; return 1; }
    [[ -f "$DEPLOY/http_server.py" ]] || { echo "Server not found: $DEPLOY/http_server.py" >&2; return 1; }
    [[ -f "$CAD_MESH" ]] || { echo "CAD mesh not found: $CAD_MESH" >&2; return 1; }
    export PYTHONPATH="$DEPLOY:$ROOT/current:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="$DEPLOY/lib:${LD_LIBRARY_PATH:-}"
    export CUDA_VISIBLE_DEVICES=0
    export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
    export SAM3_API_URL="$SAM3_API_URL"
    cd "$ROOT/current"
}

do_start() {
    if ss -tln | grep -q ":$PORT "; then
        echo "Port $PORT already busy (FoundationPose already running)"
        return 0
    fi
    mkdir -p "$LOG_DIR"
    apply_env
    nohup "${SERVER_COMMAND[@]}" > "$LOG" 2>&1 &
    echo $! > "$PIDFILE"
    echo "FoundationPose started, pid=$(cat "$PIDFILE") port=$PORT"
    for i in $(seq 1 30); do
        sleep 1
        if curl -sf -m 2 "$HEALTH_URL" > /dev/null 2>&1; then
            echo "FoundationPose ready on port $PORT"
            return 0
        fi
    done
    echo "WARN: FoundationPose did not respond within 30s (see $LOG)"
}

do_stop() {
    if [ -f "$PIDFILE" ]; then
        local PID=$(cat "$PIDFILE")
        echo "Stopping FoundationPose pid=$PID..."
        kill -TERM "$PID" 2>/dev/null || true
        for i in $(seq 1 10); do
            kill -0 "$PID" 2>/dev/null || break
            sleep 1
        done
        rm -f "$PIDFILE"
    fi
    echo "Port $PORT free"
}

do_status() {
    if ss -tln | grep -q ":$PORT "; then
        local PID="unknown"
        [ -f "$PIDFILE" ] && PID=$(cat "$PIDFILE")
        echo "FoundationPose ($PORT): RUNNING (pid=$PID)"
        curl -s -m 2 "$HEALTH_URL" | head -c 160 || true
        echo
    else
        echo "FoundationPose ($PORT): STOPPED"
    fi
}

case "$ACTION" in
    --foreground)
        # The combined launcher owns detachment, PID records and log files.
        apply_env
        exec "${SERVER_COMMAND[@]}"
        ;;
    start)
        do_start
        ;;
    stop)
        do_stop
        ;;
    restart)
        do_stop
        sleep 1
        do_start
        ;;
    status)
        do_status
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|--foreground}" >&2
        exit 1
        ;;
esac
