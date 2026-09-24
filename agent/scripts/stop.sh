#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=agent-common.sh
source "${SCRIPT_DIR}/agent-common.sh"

STOP_TIMEOUT=${AGENT_STOP_TIMEOUT:-30}
if [[ ! "${STOP_TIMEOUT}" =~ ^[0-9]+$ ]]; then
    printf '错误：AGENT_STOP_TIMEOUT 必须是非负整数。\n' >&2
    exit 2
fi

if ! pid=$(read_agent_pid); then
    if [[ -e "${PID_FILE}" ]]; then
        printf '发现无效的 PID 文件，已清理：%s\n' "${PID_FILE}"
        rm -f -- "${PID_FILE}"
    else
        printf 'Agent 服务未运行。\n'
    fi
    exit 0
fi

if ! agent_process_matches "${pid}"; then
    printf 'PID %s 不是本项目正在运行的 Agent 服务，未发送信号。\n' "${pid}" >&2
    rm -f -- "${PID_FILE}"
    exit 0
fi

printf '正在停止 Agent 服务（PID %s）...\n' "${pid}"
kill -TERM "${pid}"

deadline=$((SECONDS + STOP_TIMEOUT))
while agent_process_is_running "${pid}"; do
    if (( SECONDS >= deadline )); then
        printf '优雅停止超时，正在强制停止 Agent 服务（PID %s）。\n' "${pid}" >&2
        kill -KILL "${pid}" 2>/dev/null || true
        break
    fi
    sleep 0.2
done

rm -f -- "${PID_FILE}"
printf 'Agent 服务已停止。\n'
