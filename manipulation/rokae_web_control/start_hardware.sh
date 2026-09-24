#!/usr/bin/env bash
set -euo pipefail

if (( $# > 1 )); then
  echo "用法：$0 [start|stop|restart|status]" >&2
  exit 64
fi

case "${1:-start}" in
  start|stop|restart)
    exec systemctl --user "${1:-start}" mui.target
    ;;
  status)
    exec systemctl --user --no-pager status mui.target mui-control.service mui-web.service
    ;;
  -h|--help)
    echo "用法：$0 [start|stop|restart|status]"
    echo "统一管理网页和运控核心；SDK 上报及相机保持独立。"
    ;;
  *)
    echo "不支持的操作：$1；使用 start、stop、restart 或 status。" >&2
    exit 64
    ;;
esac
