#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=agent-common.sh
source "${SCRIPT_DIR}/agent-common.sh"

if pid=$(read_mock_pid); then
    if mock_process_matches "${pid}"; then
        printf 'Mock 服务已在运行（PID %s）。\n' "${pid}"
        exit 0
    fi
    printf '发现失效的 PID 文件，正在清理：%s\n' "${MOCK_PID_FILE}"
    remove_stale_mock_pid_file || true
elif [[ -e "${MOCK_PID_FILE}" ]]; then
    printf '发现无效的 PID 文件，正在清理：%s\n' "${MOCK_PID_FILE}"
    remove_stale_mock_pid_file || true
fi

build_mock_command
mkdir -p -- "$(dirname -- "${MOCK_PID_FILE}")"

cd "${PROJECT_ROOT}"
nohup "${MOCK_COMMAND[@]}" >/dev/null 2>&1 < /dev/null &
pid=$!
printf '%s\n' "${pid}" > "${MOCK_PID_FILE}"

# Catch configuration errors and occupied ports while keeping startup quick.
for _ in {1..20}; do
    if ! agent_process_is_running "${pid}"; then
        rm -f -- "${MOCK_PID_FILE}"
        printf '错误：Mock 服务启动失败，请前台执行 agent-mocks 排查。\n' >&2
        exit 1
    fi
    sleep 0.1
done

if ! mock_process_matches "${pid}"; then
    rm -f -- "${MOCK_PID_FILE}"
    printf '错误：启动的进程与 Mock 服务不匹配。\n' >&2
    exit 1
fi

printf 'Mock 服务已在后台启动（PID %s）。\n' "${pid}"
