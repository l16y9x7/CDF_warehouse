#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=agent-common.sh
source "${SCRIPT_DIR}/agent-common.sh"

if pid=$(read_agent_pid); then
    if agent_process_matches "${pid}"; then
        printf 'Agent 服务已在运行（PID %s）。\n' "${pid}"
        exit 0
    fi
    printf '发现失效的 PID 文件，正在清理：%s\n' "${PID_FILE}"
    remove_stale_pid_file || true
elif [[ -e "${PID_FILE}" ]]; then
    printf '发现无效的 PID 文件，正在清理：%s\n' "${PID_FILE}"
    remove_stale_pid_file || true
fi

build_agent_command
mkdir -p -- "$(dirname -- "${PID_FILE}")"

cd "${PROJECT_ROOT}"
nohup "${AGENT_COMMAND[@]}" >/dev/null 2>&1 < /dev/null &
pid=$!
printf '%s\n' "${pid}" > "${PID_FILE}"

# Catch configuration errors and occupied ports while keeping startup quick.
for _ in {1..20}; do
    if ! agent_process_is_running "${pid}"; then
        rm -f -- "${PID_FILE}"
        printf '错误：Agent 服务启动失败，请检查项目运行日志或前台启动服务排查。\n' >&2
        exit 1
    fi
    sleep 0.1
done

if ! agent_process_matches "${pid}"; then
    rm -f -- "${PID_FILE}"
    printf '错误：启动的进程与 Agent 服务不匹配。\n' >&2
    exit 1
fi

printf 'Agent 服务已在后台启动（PID %s）。\n' "${pid}"
