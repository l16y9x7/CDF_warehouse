#!/usr/bin/env bash
# Install head-only vision services; invoke with sudo on the robot.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run with sudo'; exit 1; }
VISION_ROOT="${1:-/home/admin/vision}"
VISION_USER="${2:-admin}"
[[ "$VISION_ROOT" =~ ^/[a-zA-Z0-9_./-]+$ && "$VISION_USER" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
/usr/bin/python3 - "$VISION_ROOT/config/vision.json" <<'PY'
import json,sys
c=json.load(open(sys.argv[1]))
owner=c['rokae']['owner']; cameras=owner['cameras'];push=c['media']['push']
assert c['adapter']=='rokae'
assert [k for k,v in cameras.items() if v.get('enabled') is True]==['head'], 'Enable head only'
h=cameras['head'];assert (h['width'],h['height'],h['fps'])==(1280,720,15)
s=[s for s in push['streams'] if s.get('enabled',True)]
assert len(s)==1 and s[0]['camera_id']=='head'
assert (s[0]['width'],s[0]['height'],s[0]['fps'])==(1280,720,15)
assert push.get('enabled') and push.get('autostart') and push.get('device_sn')
assert push.get('stream_server_url'), 'Configure deployment target first'
assert owner.get('source') == 'ros', 'This installer requires native ROS capture'
assert cameras['head']['match']['value'], 'Exact head serial required'
PY
# Preserve existing service definitions before replacing them.
VISION_BACKUP="$VISION_ROOT/runtime/service-backups/$(date +%Y%m%d-%H%M%S-%N)"
mkdir -p "$VISION_BACKUP"
for component in owner camera media; do
    case "$component" in
        owner) module=vision.rokae_runtime ;;
        camera) module=vision.camera_app ;;
        media) module=vision.media_app ;;
    esac
    unit="vision-head-$component.service"
    if [[ -f /etc/systemd/system/$unit ]]; then cp -a "/etc/systemd/system/$unit" "$VISION_BACKUP/"; fi
    cat > "/etc/systemd/system/$unit" <<EOF
[Unit]
Description=Vision head $component 1280x720 15fps
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=0
[Service]
Type=simple
User=$VISION_USER
WorkingDirectory=$VISION_ROOT
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash -c 'source $VISION_ROOT/config/ros/setup.bash && exec /usr/bin/python3 -m $module --config $VISION_ROOT/config/vision.json'
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=15
[Install]
WantedBy=multi-user.target
EOF
 done
for component in rgbd synced-rgbd; do
    unit="vision-head-$component.service"
    if [[ -f /etc/systemd/system/$unit ]]; then cp -a "/etc/systemd/system/$unit" "$VISION_BACKUP/"; fi
    if [[ "$component" == rgbd ]]; then
        command="-m vision.rokae_runtime.driver_supervisor --config $VISION_ROOT/config/vision.json --camera head"
    else
        command="-m vision.tianji_runtime.synced_rgbd_forwarder --ros-args -p synced_config_file:=$VISION_ROOT/config/ros/head-synced-rgbd.yaml"
    fi
    cat > "/etc/systemd/system/$unit" <<EOF
[Unit]
Description=Vision head ROS $component
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=0
[Service]
Type=simple
User=$VISION_USER
WorkingDirectory=$VISION_ROOT
ExecStart=/bin/bash -c 'source $VISION_ROOT/config/ros/setup.bash && exec /usr/bin/python3 $command'
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=20
[Install]
WantedBy=multi-user.target
EOF
 done
for unit in vision-head-health.service vision-head-health.timer; do
    if [[ -f /etc/systemd/system/$unit ]]; then cp -a "/etc/systemd/system/$unit" "$VISION_BACKUP/"; fi
 done
cat > /etc/systemd/system/vision-head-health.service <<EOF
[Unit]
Description=Check vision source frame and actual encoder output progress
After=vision-head-owner.service vision-head-camera.service vision-head-media.service
[Service]
Type=oneshot
WorkingDirectory=$VISION_ROOT
ExecStart=/usr/bin/python3 $VISION_ROOT/scripts/head-stream-check.py --config $VISION_ROOT/config/vision.json --repair
TimeoutStartSec=50
EOF
cat > /etc/systemd/system/vision-head-health.timer <<'EOF'
[Unit]
Description=Periodic vision head stream health check
[Timer]
OnBootSec=45
OnUnitInactiveSec=20
AccuracySec=1
Unit=vision-head-health.service
[Install]
WantedBy=timers.target
EOF
systemctl daemon-reload
systemctl enable vision-head-owner.service vision-head-camera.service vision-head-media.service vision-head-rgbd.service vision-head-synced-rgbd.service vision-head-health.timer
systemctl restart vision-head-owner.service vision-head-camera.service vision-head-media.service vision-head-rgbd.service vision-head-synced-rgbd.service vision-head-health.timer
printf 'Installed head-only services at %s; backups: %s\n' "$VISION_ROOT" "$VISION_BACKUP"
