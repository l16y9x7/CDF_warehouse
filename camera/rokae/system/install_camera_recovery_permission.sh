#!/usr/bin/env bash
# Only authorize the existing health task; do not touch /home/admin/vision.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo '请使用 sudo 运行此脚本。'; exit 1; }
/usr/bin/systemctl cat vision-head-health.service >/dev/null
target=/etc/sudoers.d/rokae-web-camera-recovery
rule='admin ALL=(root) NOPASSWD: /usr/bin/systemctl start vision-head-health.service'
temporary=$(mktemp /etc/sudoers.d/.rokae-web-camera-recovery.XXXXXX)
trap 'rm -f -- "$temporary"' EXIT
printf '%s\n' "$rule" >"$temporary"
chmod 0440 "$temporary"
/usr/sbin/visudo -cf "$temporary"
if [[ -e "$target" ]]; then
    cmp -s "$temporary" "$target" || { echo "已有不同的权限文件 $target，停止以免覆盖。"; exit 1; }
else
    /usr/bin/install -o root -g root -m 0440 "$temporary" "$target"
fi
echo '仅已授权现有摄像头异常恢复任务；未执行重启。'
