#!/usr/bin/env bash
# Ubuntu launcher for perception (25546) and estimation (25540).
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RUNTIME="$ROOT/.runtime/perception-estimation"
SERVICE_HOST=${SERVICE_HOST:-0.0.0.0}
PERCEPTION_PORT=${PERCEPTION_PORT:-25546}
ESTIMATION_PORT=${ESTIMATION_PORT:-25540}
START_TIMEOUT=${START_TIMEOUT:-60}
STOP_TIMEOUT=${STOP_TIMEOUT:-20}
SAM3_BACKEND=${SAM3_BACKEND:-multipart_segment}
declare -A PYTHONS PORTS URLS
STARTED=()

usage() {
    cat <<'EOF'
Usage: bash start_services.sh [start|stop|restart|status|install|help]

  start    Start both services in the background (default).
  stop     Stop only processes started by this script.
  restart  Stop both services, then start them again.
  status   Check both processes and their HTTP health endpoints.
  install  Create separate .venv environments and install requirements.

Configuration (environment variables):
  PERCEPTION_PYTHON / ESTIMATION_PYTHON  Existing Python executable paths
  SERVICE_HOST                         Bind address (default: 0.0.0.0)
  PERCEPTION_PORT / ESTIMATION_PORT     Ports (default: 25546 / 25540)
  SAM3_URL                             Shared SAM3 endpoint
  SAM3_BACKEND                          multipart_segment or legacy_18003
  START_TIMEOUT / STOP_TIMEOUT          Wait limits in seconds (60 / 20)

SAM3 must already be running. Basket inference also needs FoundationPose.
Logs and PID records: .runtime/perception-estimation/
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

ACTION=${1:-start}
[[ $# -le 1 ]] || { usage >&2; exit 2; }
case "$ACTION" in
    help|-h|--help) usage; exit 0 ;;
    start|stop|restart|status|install) ;;
    *) usage >&2; exit 2 ;;
esac

[[ $(uname -s) == Linux ]] || die 'Run this script on Ubuntu/Linux.'
for tool in flock curl ss setsid nohup; do
    command -v "$tool" >/dev/null || die "Missing $tool. Install: sudo apt-get install curl iproute2 util-linux coreutils"
done
for value in "$PERCEPTION_PORT" "$ESTIMATION_PORT"; do
    [[ $value =~ ^[1-9][0-9]{0,4}$ ]] && (( value <= 65535 )) || die "Invalid port: $value"
done
[[ $PERCEPTION_PORT != "$ESTIMATION_PORT" ]] || die 'The two services need different ports.'
for value in "$START_TIMEOUT" "$STOP_TIMEOUT"; do
    [[ $value =~ ^[1-9][0-9]{0,4}$ ]] || die "Invalid timeout: $value"
done

case "$SAM3_BACKEND" in
    multipart_segment)
        SAM3_URL=${SAM3_URL:-http://127.0.0.1:25541/api/v1/segment}
        PERCEPTION_BACKEND=segment ;;
    legacy_18003)
        SAM3_URL=${SAM3_URL:-http://127.0.0.1:25551/infer}
        PERCEPTION_BACKEND=infer ;;
    *) die 'SAM3_BACKEND must be multipart_segment or legacy_18003.' ;;
esac

HEALTH_HOST=$SERVICE_HOST
case "$HEALTH_HOST" in
    0.0.0.0) HEALTH_HOST=127.0.0.1 ;;
    ::) HEALTH_HOST='[::1]' ;;
    *:*) HEALTH_HOST="[$HEALTH_HOST]" ;;
esac
PORTS[perception]=$PERCEPTION_PORT
PORTS[estimation]=$ESTIMATION_PORT
URLS[perception]="http://$HEALTH_HOST:$PERCEPTION_PORT/perception/health"
URLS[estimation]="http://$HEALTH_HOST:$ESTIMATION_PORT/health"

mkdir -p -- "$RUNTIME"
exec 9>"$RUNTIME/launcher.lock"
flock -n 9 || die 'Another launcher operation is in progress.'

resolve_python() {
    local name=$1 override venv python
    if [[ $name == perception ]]; then
        override=${PERCEPTION_PYTHON:-}
    else
        override=${ESTIMATION_PYTHON:-}
    fi
    venv="$ROOT/$name/.venv"
    if [[ $name == estimation && ! -x "$venv/bin/python" && -x "$ROOT/estimation/deploy/.venv/bin/python" ]]; then
        venv="$ROOT/estimation/deploy/.venv"
    fi
    if [[ -n $override ]]; then
        python=$(command -v -- "$override") || die "Python not found: $override"
    else
        if [[ $ACTION == install && ! -x "$venv/bin/python" ]]; then
            python3 -m venv "$venv" || die 'Cannot create venv. Install Python 3.10+ and python3-venv first.'
        fi
        if [[ -x "$venv/bin/python" ]]; then
            python="$venv/bin/python"
        else
            python=$(command -v python3) || die 'Python not found. Run install or set the service Python paths.'
        fi
    fi
    # Resolve relative executable paths before switching to each service directory.
    python=$(cd -- "$(dirname -- "$python")" && printf '%s/%s' "$PWD" "$(basename -- "$python")")
    "$python" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else "Python 3.10+ is required")'
    PYTHONS[$name]=$python
}

# Linux start time distinguishes our process from a recycled PID.
process_token() {
    local pid=$1 stat
    local -a fields
    [[ -r /proc/$pid/stat ]] || return 1
    stat=$(cat "/proc/$pid/stat" 2>/dev/null) || return 1
    read -r -a fields <<< "${stat##*) }"
    [[ ${fields[0]:-Z} != Z && ${fields[0]:-X} != X ]] || return 1
    printf '%s\n' "${fields[19]}"
}

owned_pid() {
    local name=$1 pid token current
    [[ -f $RUNTIME/$name.pid ]] || return 1
    read -r pid token < "$RUNTIME/$name.pid" || return 1
    [[ $pid =~ ^[1-9][0-9]*$ && $pid -gt 1 && $token =~ ^[0-9]+$ ]] || return 1
    current=$(process_token "$pid") || return 1
    [[ $current == "$token" ]] || return 1
    printf '%s\n' "$pid"
}

healthy() {
    curl --noproxy '*' --fail --silent --max-time 2 "${URLS[$1]}" >/dev/null
}

check_service() {
    local name=$1 pid listeners
    if pid=$(owned_pid "$name"); then
        healthy "$name" || die "$name is running (PID $pid) but unhealthy. See $RUNTIME/$name.log"
        return 0
    fi
    listeners=$(ss -H -ltn "sport = :${PORTS[$name]}") || die 'Cannot inspect listening ports.'
    [[ -z $listeners ]] || die "Port ${PORTS[$name]} is already occupied outside this launcher; stop that service yourself or choose a different port."
    resolve_python "$name"
}

stop_service() {
    local name=$1 pid deadline
    if ! pid=$(owned_pid "$name"); then
        rm -f -- "$RUNTIME/$name.pid"
        printf '%s: not running under this launcher\n' "$name"
        return 0
    fi
    # setsid makes the Python PID its process group ID; stop its workers too.
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    deadline=$((SECONDS + STOP_TIMEOUT))
    while owned_pid "$name" >/dev/null; do
        if (( SECONDS >= deadline )); then
            printf 'ERROR: %s did not stop; leaving its PID record for inspection.\n' "$name" >&2
            return 1
        fi
        sleep 0.2
    done
    rm -f -- "$RUNTIME/$name.pid"
    printf '%s: stopped\n' "$name"
}

rollback() {
    local code=$? name
    trap - EXIT
    if (( code != 0 )); then
        for name in "${STARTED[@]}"; do
            stop_service "$name" || true
        done
    fi
    exit "$code"
}

start_service() {
    local name=$1 pid token deadline
    local -a command
    if pid=$(owned_pid "$name"); then
        printf '%s: already running (PID %s)\n' "$name" "$pid"
        return 0
    fi
    if [[ $name == perception ]]; then
        command=(env "SAM3_URL=$SAM3_URL" "SAM3_BACKEND=$PERCEPTION_BACKEND"
            "SKU_API_URL=${SKU_API_URL:-http://$HEALTH_HOST:$ESTIMATION_PORT}"
            "PYTHONUNBUFFERED=1" "${PYTHONS[$name]}"
            -m uvicorn main:app --host "$SERVICE_HOST" --port "$PERCEPTION_PORT")
    else
        command=(env "SAM3_URL=$SAM3_URL" "SAM3_BACKEND=$SAM3_BACKEND"
            "MPLBACKEND=Agg" "PYTHONUNBUFFERED=1"
            "FIT_BOOTSTRAP_WORKERS=${FIT_BOOTSTRAP_WORKERS:-6}"
            "FIT_BOOTSTRAP_BOOTS=${FIT_BOOTSTRAP_BOOTS:-4}"
            "FIT_ANALYTIC_JAC=${FIT_ANALYTIC_JAC:-1}"
            "FIT_MAIN_FIT_POOL=${FIT_MAIN_FIT_POOL:-1}"
            "FRONT_PANEL_RANSAC_MAX_ITER=${FRONT_PANEL_RANSAC_MAX_ITER:-200}"
            "FRONT_PANEL_RANSAC_SKIP_MEDIAN=${FRONT_PANEL_RANSAC_SKIP_MEDIAN:-1}"
            "OMP_NUM_THREADS=${OMP_NUM_THREADS:-1}" "OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}"
            "MKL_NUM_THREADS=${MKL_NUM_THREADS:-1}" "NUMEXPR_NUM_THREADS=${NUMEXPR_NUM_THREADS:-1}"
            "VECLIB_MAXIMUM_THREADS=${VECLIB_MAXIMUM_THREADS:-1}"
            "${PYTHONS[$name]}" "$ROOT/estimation/deploy/server.py"
            --host "$SERVICE_HOST" --port "$ESTIMATION_PORT")
    fi
    (
        cd -- "$ROOT/$name"
        exec nohup setsid "${command[@]}"
    ) >>"$RUNTIME/$name.log" 2>&1 < /dev/null 9>&- &
    pid=$!
    token=$(process_token "$pid") || die "$name exited during launch. See $RUNTIME/$name.log"
    printf '%s %s\n' "$pid" "$token" > "$RUNTIME/$name.pid"
    STARTED=("$name" "${STARTED[@]}")
    deadline=$((SECONDS + START_TIMEOUT))
    while (( SECONDS < deadline )); do
        owned_pid "$name" >/dev/null || break
        if healthy "$name" && owned_pid "$name" >/dev/null; then
            printf '%s: ready (PID %s) %s\n' "$name" "$pid" "${URLS[$name]}"
            return 0
        fi
        sleep 0.5
    done
    tail -n 30 -- "$RUNTIME/$name.log" >&2 || true
    die "$name failed to become ready. See $RUNTIME/$name.log"
}

case "$ACTION" in
    install)
        for name in estimation perception; do
            [[ -f $ROOT/$name/requirements.txt ]] || die "Missing $name/requirements.txt"
            resolve_python "$name"
            "${PYTHONS[$name]}" -m pip install -r "$ROOT/$name/requirements.txt"
        done
        printf 'Dependencies installed. Start with: bash "%s/start_services.sh"\n' "$ROOT"
        ;;
    stop)
        result=0
        stop_service perception || result=1
        stop_service estimation || result=1
        exit "$result"
        ;;
    status)
        result=0
        for name in estimation perception; do
            if pid=$(owned_pid "$name"); then
                if healthy "$name"; then
                    printf '%s: ready (PID %s) %s\n' "$name" "$pid" "${URLS[$name]}"
                else
                    printf '%s: running but unhealthy (PID %s)\n' "$name" "$pid"
                    result=1
                fi
            else
                printf '%s: not running under this launcher\n' "$name"
                result=1
            fi
        done
        exit "$result"
        ;;
    start|restart)
        [[ -f $ROOT/perception/main.py && -f $ROOT/estimation/deploy/server.py ]] || die 'Service code is missing; use a checkout containing both services.'
        if [[ $ACTION == restart ]]; then
            stop_service perception
            stop_service estimation
        fi
        check_service estimation
        check_service perception
        trap rollback EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM
        start_service estimation
        start_service perception
        printf 'Logs: %s/{estimation,perception}.log\n' "$RUNTIME"
        ;;
esac
