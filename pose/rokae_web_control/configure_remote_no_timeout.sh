#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

BACKUP_DIR="$SCRIPT_DIR/logs/remote_access_backups/$(date +%Y%m%d-%H%M%S)"
SSHD_DROP_IN="/etc/ssh/sshd_config.d/99-rokae-keepalive.conf"
mkdir -p "$BACKUP_DIR"

if [[ -f "$SSHD_DROP_IN" ]]; then
  cp -a "$SSHD_DROP_IN" "$BACKUP_DIR/"
fi

install -D -m 0644 \
  "$SCRIPT_DIR/system/99-rokae-keepalive.conf" \
  "$SSHD_DROP_IN"

/usr/sbin/sshd -t
systemctl reload ssh.service

while IFS=: read -r connection connection_type device; do
  if [[ "$connection_type" == "802-11-wireless" && -n "$device" ]]; then
    nmcli connection modify "$connection" 802-11-wireless.powersave 2
    if command -v iw >/dev/null 2>&1; then
      iw dev "$device" set power_save off || true
    fi
    printf 'Wi-Fi 省电已关闭: %s (%s)\n' "$connection" "$device"
  fi
done < <(nmcli -t -f NAME,TYPE,DEVICE connection show --active)

systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target >/dev/null

printf 'SSH 空闲断开已禁用，30 秒心跳已启用。\n'
printf '系统自动休眠入口已屏蔽，配置备份目录: %s\n' "$BACKUP_DIR"
printf '当前 SSH 会话无需重连，设置已即时生效。\n'
