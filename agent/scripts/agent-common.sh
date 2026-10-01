#!/usr/bin/env bash

# Shared process-management helpers for the Agent service scripts.

set -o nounset

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)
PID_FILE=${AGENT_PID_FILE:-"${PROJECT_ROOT}/run/agent-server.pid"}
MOCK_PID_FILE=${AGENT_MOCK_PID_FILE:-"${PROJECT_ROOT}/run/agent-mocks.pid"}

read_agent_pid() {
    local pid

    [[ -f "${PID_FILE}" ]] || return 1
    IFS= read -r pid < "${PID_FILE}" || return 1
    [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
    printf '%s\n' "${pid}"
}

agent_process_is_running() {
    local pid=$1
    kill -0 "${pid}" 2>/dev/null
}

agent_process_matches() {
    local pid=$1
    local process_cwd process_command

    agent_process_is_running "${pid}" || return 1

    # Refuse to signal an unrelated process if a stale PID file was reused.
    if [[ -r "/proc/${pid}/cmdline" ]]; then
        process_command=$(tr '\0' ' ' < "/proc/${pid}/cmdline")
        case "${process_command}" in
            *agent-server*|*agent.server*) ;;
            *) return 1 ;;
        esac
    fi
    if [[ -L "/proc/${pid}/cwd" ]]; then
        process_cwd=$(readlink "/proc/${pid}/cwd")
        [[ "${process_cwd}" == "${PROJECT_ROOT}" ]] || return 1
    fi
}

remove_stale_pid_file() {
    local pid

    if pid=$(read_agent_pid) && agent_process_matches "${pid}"; then
        return 1
    fi
    rm -f -- "${PID_FILE}"
}

build_agent_command() {
    AGENT_COMMAND=()

    if [[ -n "${AGENT_SERVER_EXECUTABLE:-}" ]]; then
        if [[ ! -x "${AGENT_SERVER_EXECUTABLE}" ]]; then
            printf '错误：AGENT_SERVER_EXECUTABLE 不可执行：%s\n' "${AGENT_SERVER_EXECUTABLE}" >&2
            return 1
        fi
        AGENT_COMMAND=("${AGENT_SERVER_EXECUTABLE}")
    elif [[ -x "${PROJECT_ROOT}/.venv/bin/agent-server" ]]; then
        AGENT_COMMAND=("${PROJECT_ROOT}/.venv/bin/agent-server")
    elif command -v agent-server >/dev/null 2>&1; then
        AGENT_COMMAND=("$(command -v agent-server)")
    elif command -v uv >/dev/null 2>&1; then
        AGENT_COMMAND=("$(command -v uv)" run agent-server)
    else
        printf '错误：未找到 agent-server。请先执行 pip install -e . 或安装 uv。\n' >&2
        return 1
    fi
}

read_mock_pid() {
    local pid

    [[ -f "${MOCK_PID_FILE}" ]] || return 1
    IFS= read -r pid < "${MOCK_PID_FILE}" || return 1
    [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
    printf '%s\n' "${pid}"
}

mock_process_matches() {
    local pid=$1
    local process_cwd process_command

    agent_process_is_running "${pid}" || return 1

    if [[ -r "/proc/${pid}/cmdline" ]]; then
        process_command=$(tr '\0' ' ' < "/proc/${pid}/cmdline")
        case "${process_command}" in
            *agent-mocks*|*mocks.server*) ;;
            *) return 1 ;;
        esac
    fi
    if [[ -L "/proc/${pid}/cwd" ]]; then
        process_cwd=$(readlink "/proc/${pid}/cwd")
        [[ "${process_cwd}" == "${PROJECT_ROOT}" ]] || return 1
    fi
}

remove_stale_mock_pid_file() {
    local pid

    if pid=$(read_mock_pid) && mock_process_matches "${pid}"; then
        return 1
    fi
    rm -f -- "${MOCK_PID_FILE}"
}

build_mock_command() {
    MOCK_COMMAND=()

    if [[ -n "${AGENT_MOCK_EXECUTABLE:-}" ]]; then
        if [[ ! -x "${AGENT_MOCK_EXECUTABLE}" ]]; then
            printf '错误：AGENT_MOCK_EXECUTABLE 不可执行：%s\n' "${AGENT_MOCK_EXECUTABLE}" >&2
            return 1
        fi
        MOCK_COMMAND=("${AGENT_MOCK_EXECUTABLE}")
    elif [[ -x "${PROJECT_ROOT}/.venv/bin/agent-mocks" ]]; then
        MOCK_COMMAND=("${PROJECT_ROOT}/.venv/bin/agent-mocks")
    elif command -v agent-mocks >/dev/null 2>&1; then
        MOCK_COMMAND=("$(command -v agent-mocks)")
    elif command -v uv >/dev/null 2>&1; then
        MOCK_COMMAND=("$(command -v uv)" run agent-mocks)
    else
        printf '错误：未找到 agent-mocks。请先执行 pip install -e . 或安装 uv。\n' >&2
        return 1
    fi
}
