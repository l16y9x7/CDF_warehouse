#!/usr/bin/env bash
# Ubuntu launcher for SAM3, FoundationPose, estimation, and perception.
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RUNTIME="$ROOT/.runtime/perception-estimation"
SERVICE_HOST=${SERVICE_HOST:-0.0.0.0}
PERCEPTION_PORT=${PERCEPTION_PORT:-25546}
ESTIMATION_PORT=${ESTIMATION_PORT:-25540}
START_TIMEOUT=${START_TIMEOUT:-60}
STOP_TIMEOUT=${STOP_TIMEOUT:-20}
SAM3_BACKEND=${SAM3_BACKEND:-multipart_segment}
START_SAM3=${START_SAM3:-1}
SAM3_DIR=${SAM3_DIR:-/data/steven/sam3_api}
SAM3_CONDA_ENV=${SAM3_CONDA_ENV:-sam3}
SAM3_START_TIMEOUT=${SAM3_START_TIMEOUT:-300}
SAM3_HEALTH_URL=${SAM3_HEALTH_URL:-}
START_FOUNDATIONPOSE=${START_FOUNDATIONPOSE:-1}
FOUNDATIONPOSE_SCRIPT=${FOUNDATIONPOSE_SCRIPT:-$ROOT/start_25550.sh}
FOUNDATIONPOSE_START_TIMEOUT=${FOUNDATIONPOSE_START_TIMEOUT:-300}
FOUNDATIONPOSE_HEALTH_URL=${FOUNDATIONPOSE_HEALTH_URL:-http://127.0.0.1:25550/health}
FOUNDATIONPOSE_URL=${BASKET_FP_URL:-http://127.0.0.1:25550/infer}
declare -A PYTHONS PORTS URLS DIRS
SAM3_COMMAND=()
STARTED=()

usage() {
    cat <<'EOF'
Usage: bash start_services.sh [start|stop|restart|status|install|help]

  start    Start SAM3, FoundationPose, estimation, then perception (default).
  stop     Stop only processes started by this script.
  restart  Stop managed services, then start them again.
  status   Check the services and their HTTP endpoints.
  install  Install perception/estimation dependencies (model services preinstalled).

Configuration (environment variables):
  PERCEPTION_PYTHON / ESTIMATION_PYTHON  Existing Python executable paths
  SERVICE_HOST                         Bind address (default: 0.0.0.0)
  PERCEPTION_PORT / ESTIMATION_PORT     Ports (default: 25546 / 25540)
  SAM3_DIR                             SAM3 checkout (/data/steven/sam3_api)
  SAM3_CONDA_ENV                       Existing Conda environment (sam3)
  SAM3_CONDA_SH                        Optional path to conda.sh
  SAM3_PYTHON                          Explicit Python instead of Conda activation
  START_SAM3                           1: manage local SAM3; 0: use external SAM3
  SAM3_URL                             Shared SAM3 endpoint
  SAM3_BACKEND                          multipart_segment or legacy_18003
  SAM3_HEALTH_URL                      Optional dedicated health URL
  SAM3_START_TIMEOUT                   SAM3 startup wait limit (300 seconds)
  START_FOUNDATIONPOSE                 1: invoke local script; 0: use external service
  FOUNDATIONPOSE_SCRIPT                Bundled start_25550.sh (--foreground)
  FOUNDATIONPOSE_ROOT                  Model deployment root (/data/quinn/foundationpose)
  FOUNDATIONPOSE_PYTHON                Optional Python override (ROOT/env/bin/python)
  FOUNDATIONPOSE_START_TIMEOUT         FoundationPose wait limit (300 seconds)
  FOUNDATIONPOSE_HEALTH_URL            http://127.0.0.1:25550/health
  BASKET_FP_URL                        FoundationPose endpoint (local port 25550)
  START_TIMEOUT / STOP_TIMEOUT          Wait limits in seconds (60 / 20)

Prepare model services and weights before starting. SAM3 uses fixed port 25541.
FoundationPose runs through start_25550.sh --foreground with PORT=25550.
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
for value in "$START_TIMEOUT" "$STOP_TIMEOUT" "$SAM3_START_TIMEOUT" "$FOUNDATIONPOSE_START_TIMEOUT"; do
    [[ $value =~ ^[1-9][0-9]{0,4}$ ]] || die "Invalid timeout: $value"
done
[[ $START_SAM3 == 0 || $START_SAM3 == 1 ]] || die 'START_SAM3 must be 0 or 1.'
[[ $START_FOUNDATIONPOSE == 0 || $START_FOUNDATIONPOSE == 1 ]] || die 'START_FOUNDATIONPOSE must be 0 or 1.'
SERVICES=(estimation perception)
if [[ $START_FOUNDATIONPOSE == 1 ]]; then
    [[ $PERCEPTION_PORT != 25550 && $ESTIMATION_PORT != 25550 ]] || die 'Port 25550 is reserved for FoundationPose.'
    SERVICES=(foundationpose "${SERVICES[@]}")
fi
if [[ $START_SAM3 == 1 ]]; then
    [[ $PERCEPTION_PORT != 25541 && $ESTIMATION_PORT != 25541 ]] || die 'Port 25541 is reserved for managed SAM3.'
    SERVICES=(sam3 "${SERVICES[@]}")
fi

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
PORTS[sam3]=25541
PORTS[foundationpose]=25550
URLS[perception]="http://$HEALTH_HOST:$PERCEPTION_PORT/perception/health"
URLS[estimation]="http://$HEALTH_HOST:$ESTIMATION_PORT/health"
URLS[sam3]=${SAM3_HEALTH_URL:-$SAM3_URL}
URLS[foundationpose]=${FOUNDATIONPOSE_HEALTH_URL:-$FOUNDATIONPOSE_URL}
DIRS[perception]="$ROOT/perception"
DIRS[estimation]="$ROOT/estimation"

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

resolve_sam3_command() {
    local python conda_sh=${SAM3_CONDA_SH:-} conda_base candidate
    [[ -f $SAM3_DIR/backend/app.py ]] || die "Missing $SAM3_DIR/backend/app.py. Set SAM3_DIR to your installed sam3_api checkout."
    DIRS[sam3]=$(cd -- "$SAM3_DIR" && pwd)
    if [[ -n ${SAM3_PYTHON:-} ]]; then
        python=$(command -v -- "$SAM3_PYTHON") || die "Python not found: $SAM3_PYTHON"
        python=$(cd -- "$(dirname -- "$python")" && printf '%s/%s' "$PWD" "$(basename -- "$python")")
        SAM3_COMMAND=("$python" -u)
    else
        if [[ -z $conda_sh ]]; then
            if [[ -n ${CONDA_EXE:-} && -x $CONDA_EXE ]]; then
                conda_base=$("$CONDA_EXE" info --base) || die 'Cannot locate Conda base.'
                conda_sh="$conda_base/etc/profile.d/conda.sh"
            elif command -v conda >/dev/null; then
                conda_base=$(conda info --base) || die 'Cannot locate Conda base.'
                conda_sh="$conda_base/etc/profile.d/conda.sh"
            else
                for candidate in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
                    if [[ -f $candidate/etc/profile.d/conda.sh ]]; then
                        conda_sh="$candidate/etc/profile.d/conda.sh"
                        break
                    fi
                done
            fi
        fi
        [[ -f $conda_sh ]] || die 'Conda activation script not found. Set SAM3_CONDA_SH or SAM3_PYTHON.'
        conda_sh=$(cd -- "$(dirname -- "$conda_sh")" && printf '%s/%s' "$PWD" "$(basename -- "$conda_sh")")
        # Activate in the child so Conda's library paths and activation hooks apply.
        # exec preserves the tracked PID when Bash hands over to Python.
        SAM3_COMMAND=(bash -c 'set -e; source "$1"; conda activate "$2"; shift 2; exec python -u "$@"'
            sam3 "$conda_sh" "$SAM3_CONDA_ENV")
    fi
    "${SAM3_COMMAND[@]}" -c 'import sys' || die 'Cannot run Python in the SAM3 environment.'
    SAM3_COMMAND+=("${DIRS[sam3]}/backend/app.py")
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
    local code
    if [[ $1 == sam3 && -z $SAM3_HEALTH_URL ]]; then
        # SAM3 only documents a POST endpoint, not /health. A GET 405
        # confirms the route is served without submitting a GPU inference job.
        code=$(curl --noproxy '*' --silent --output /dev/null --write-out '%{http_code}' \
            --max-time 2 "${URLS[$1]}") || return 1
        [[ $code == 2[0-9][0-9] || $code == 405 ]]
        return
    fi
    curl --noproxy '*' --fail --silent --max-time 2 "${URLS[$1]}" >/dev/null
}

check_service() {
    local name=$1 pid listeners
    if pid=$(owned_pid "$name"); then
        healthy "$name" || die "$name is running (PID $pid) but unhealthy. See $RUNTIME/$name.log"
        return 0
    fi
    listeners=$(ss -H -ltn "sport = :${PORTS[$name]}") || die 'Cannot inspect listening ports.'
    if [[ -n $listeners ]]; then
        if [[ $name == sam3 ]]; then
            die 'SAM3 port 25541 is already occupied outside this launcher. Use START_SAM3=0 to keep using an externally managed SAM3.'
        fi
        if [[ $name == foundationpose ]]; then
            die 'FoundationPose port 25550 is already occupied outside this launcher. Use START_FOUNDATIONPOSE=0 to keep using an externally managed service.'
        fi
        die "Port ${PORTS[$name]} is already occupied outside this launcher; stop that service yourself or choose a different port."
    fi
    if [[ $name == sam3 ]]; then
        resolve_sam3_command
    elif [[ $name == foundationpose ]]; then
        resolve_foundationpose_script
    else
        resolve_python "$name"
    fi
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

resolve_foundationpose_script() {
    [[ -f $FOUNDATIONPOSE_SCRIPT && -r $FOUNDATIONPOSE_SCRIPT ]] || die "FoundationPose script not found or unreadable: $FOUNDATIONPOSE_SCRIPT"
    DIRS[foundationpose]=$(cd -- "$(dirname -- "$FOUNDATIONPOSE_SCRIPT")" && pwd)
    FOUNDATIONPOSE_SCRIPT="${DIRS[foundationpose]}/$(basename -- "$FOUNDATIONPOSE_SCRIPT")"
}

start_service() {
    local name=$1 pid token deadline timeout=$START_TIMEOUT
    local -a command
    if pid=$(owned_pid "$name"); then
        printf '%s: already running (PID %s)\n' "$name" "$pid"
        return 0
    fi
    if [[ $name == sam3 ]]; then
        (( ${#SAM3_COMMAND[@]} )) || resolve_sam3_command
        command=(env PYTHONUNBUFFERED=1 "${SAM3_COMMAND[@]}")
        timeout=$SAM3_START_TIMEOUT
    elif [[ $name == foundationpose ]]; then
        resolve_foundationpose_script
        command=(env PORT=25550 PYTHONUNBUFFERED=1 bash "$FOUNDATIONPOSE_SCRIPT" --foreground)
        timeout=$FOUNDATIONPOSE_START_TIMEOUT
    elif [[ $name == perception ]]; then
        [[ -n ${PYTHONS[$name]:-} ]] || resolve_python "$name"
        command=(env "SAM3_URL=$SAM3_URL" "SAM3_BACKEND=$PERCEPTION_BACKEND"
            "SKU_API_URL=${SKU_API_URL:-http://$HEALTH_HOST:$ESTIMATION_PORT}"
            "PYTHONUNBUFFERED=1" "${PYTHONS[$name]}"
            -m uvicorn main:app --host "$SERVICE_HOST" --port "$PERCEPTION_PORT")
    else
        [[ -n ${PYTHONS[$name]:-} ]] || resolve_python "$name"
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
        cd -- "${DIRS[$name]}"
        exec nohup setsid "${command[@]}"
    ) >>"$RUNTIME/$name.log" 2>&1 < /dev/null 9>&- &
    pid=$!
    token=$(process_token "$pid") || die "$name exited during launch. See $RUNTIME/$name.log"
    printf '%s %s\n' "$pid" "$token" > "$RUNTIME/$name.pid"
    STARTED=("$name" "${STARTED[@]}")
    deadline=$((SECONDS + timeout))
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
        printf 'Perception/estimation dependencies installed. SAM3 and FoundationPose need their existing environments and weights.\n'
        printf 'Start with: bash "%s/start_services.sh"\n' "$ROOT"
        ;;
    stop)
        result=0
        stop_service perception || result=1
        stop_service estimation || result=1
        stop_service foundationpose || result=1
        stop_service sam3 || result=1
        exit "$result"
        ;;
    status)
        result=0
        for name in "${SERVICES[@]}"; do
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
        if [[ $START_FOUNDATIONPOSE == 1 ]]; then
            case "${FOUNDATIONPOSE_URL%/}" in
                http://127.0.0.1:25550/infer|http://localhost:25550/infer) ;;
                *) die 'Local FoundationPose startup requires http://127.0.0.1:25550/infer. Use START_FOUNDATIONPOSE=0 for a remote service.' ;;
            esac
            export BASKET_FP_URL="$FOUNDATIONPOSE_URL"
        fi
        if [[ $START_SAM3 == 1 ]]; then
            [[ $SAM3_BACKEND == multipart_segment ]] || die 'Managed SAM3 requires multipart_segment. Use START_SAM3=0 for an external TRT backend.'
            case "${SAM3_URL%/}" in
                http://127.0.0.1:25541/api/v1/segment|http://localhost:25541/api/v1/segment) ;;
                *) die 'Managed SAM3 requires the local endpoint http://127.0.0.1:25541/api/v1/segment. Use START_SAM3=0 for a remote SAM3.' ;;
            esac
        fi
        if [[ $ACTION == restart ]]; then
            stop_service perception
            stop_service estimation
            stop_service foundationpose
            stop_service sam3
        fi
        for name in "${SERVICES[@]}"; do
            check_service "$name"
        done
        trap rollback EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM
        for name in "${SERVICES[@]}"; do
            start_service "$name"
        done
        printf 'Logs: %s/ (sam3.log, foundationpose.log, estimation.log, perception.log)\n' "$RUNTIME"
        ;;
esac
