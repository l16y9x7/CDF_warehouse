#!/usr/bin/env bash
# 本机跑：A → B → A → B …  Ctrl+C 停脚本（车可能还在走，需要的话再 curl /stop）
set -euo pipefail

BASE="${BASE:-http://192.168.71.51:8001}"
TIMEOUT="${TIMEOUT:-90}"
n=0

json_field() {
  python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get(sys.argv[1]) or "")' "$1"
}

goto_wait() {
  local st="$1"
  n=$((n + 1))
  local rid="LOOP-${st}-${n}"
  local key="loop-${st}-${n}-$(date +%s)-${RANDOM}"
  echo ">>> goto ${st}  ${rid}"
  local acc
  acc="$(curl -sS -X POST "${BASE}/goto" \
    -H 'Content-Type: application/json' \
    -d "{\"task_id\":\"LOOP\",\"request_id\":\"${rid}\",\"timeout_sec\":${TIMEOUT},\"idempotency_key\":\"${key}\",\"station_id\":\"${st}\"}")"
  echo "${acc}"
  local accepted state
  accepted="$(printf '%s' "${acc}" | json_field accepted)"
  if [[ "${accepted}" != "True" && "${accepted}" != "true" ]]; then
    echo "goto 未接受，停止"
    return 1
  fi
  local i
  for i in $(seq 1 $((TIMEOUT + 15))); do
    sleep 1
    local body
    body="$(curl -sS "${BASE}/status/${rid}")"
    state="$(printf '%s' "${body}" | json_field terminal_state)"
    [[ -z "${state}" || "${state}" == "None" ]] && state="$(printf '%s' "${body}" | json_field state)"
    echo "    ${st}  ${state}"
    case "${state}" in
      SUCCEEDED) echo "<<< arrived ${st}"; return 0 ;;
      FAILED|REJECTED|CANCELLED|TIMED_OUT)
        echo "${body}"
        echo "失败，停止"
        return 1
        ;;
    esac
  done
  echo "等 ${st} 超时，停止"
  return 1
}

trap 'echo; echo "已停脚本"; exit 0' INT
echo "loop ${BASE}  A↔B  timeout=${TIMEOUT}s  Ctrl+C 结束"
while true; do
  goto_wait A
  goto_wait B
done
