#!/usr/bin/env bash
# Install the exact camera-only commands and restore the original wrist unit.
# This does not restart anything or change boot enable/disable settings.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo '请使用 sudo 运行此脚本；仅安装权限和右腕服务定义，不重启设备。'; exit 1; }
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
units='vision-head-rgbd.service vision-head-synced-rgbd.service vision-head-owner.service vision-head-camera.service vision-head-media.service vision-left-wrist-rgb.service'
for unit in $units; do
    [[ $(/usr/bin/systemctl show "$unit" --property=LoadState --value) == loaded ]] || {
        echo "服务缺失或已屏蔽：$unit；未安装。"; exit 1;
    }
done
uid=$(id -u admin)
user_units=/home/admin/.config/systemd/user
unit_source="$project/system/mui-right-wrist-rgb.service"
unit_target="$user_units/mui-right-wrist-rgb.service"
[[ -f "$unit_source" ]] || { echo "缺少右腕服务定义：$unit_source"; exit 1; }
if [[ -e "$unit_target" ]] && ! cmp -s "$unit_source" "$unit_target"; then
    echo "现有右腕服务与原开机定义不同：$unit_target；未覆盖。"; exit 1
fi
target=/etc/sudoers.d/rokae-web-camera-restart
temporary=$(mktemp /etc/sudoers.d/.rokae-web-camera-restart.XXXXXX)
trap 'rm -f -- "$temporary"' EXIT
printf 'admin ALL=(root) NOPASSWD: /usr/bin/systemctl restart %s\n' "$units" >"$temporary"
printf 'admin ALL=(root) NOPASSWD: /usr/bin/systemctl show %s --property=Id\\,LoadState\\,ActiveState\\,SubState\\,MainPID\n' "$units" >>"$temporary"
chmod 0440 "$temporary"
/usr/sbin/visudo -cf "$temporary"
if [[ -e "$target" ]] && ! cmp -s "$temporary" "$target"; then
    echo "已有不同权限文件：$target；未覆盖。"; exit 1
fi
/usr/bin/install -d -o admin -g admin -m 0755 "$user_units"
if [[ ! -e "$unit_target" ]]; then
    /usr/bin/install -o admin -g admin -m 0644 "$unit_source" "$unit_target"
fi
/usr/sbin/runuser -u admin -- env XDG_RUNTIME_DIR="/run/user/$uid" /usr/bin/systemctl --user daemon-reload
/usr/bin/install -o root -g root -m 0440 "$temporary" "$target"
echo '三路摄像头重启授权已安装，右腕服务定义已恢复；没有重启、启动或启用自启动。'
