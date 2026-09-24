"""Check real input and encoder output. Nonzero exit means not healthy."""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

REPAIR_AFTER_CONSECUTIVE_FAILURES = 3

def fetch(url, binary=False):
    with urllib.request.urlopen(url, timeout=3) as response:
        data = response.read(8 * 1024 * 1024 + 1)
    return data if binary else json.loads(data)


def check(config):
    owner_port = config['rokae']['owner'].get('port', 8085)
    owner = f'http://127.0.0.1:{owner_port}'
    media = f"http://127.0.0.1:{config['media'].get('port',8005)}"
    result = {'ok': False, 'checks': {}, 'errors': []}
    restart = set()
    try:
        listing = fetch(owner + '/camera/list')
        enabled = [
            row for row in (listing if isinstance(listing, list) else listing.get('cameras', [])) if row.get('enabled', True)
        ]
        if not enabled:
            raise ValueError('NO_ENABLED_OWNER_CAMERA')
        not_ready = [row for row in enabled if not row.get('ready')]
        if not_ready:
            details = ','.join(
                f"{row.get('id', '?')}:{row.get('error') or 'NOT_READY'}"
                for row in not_ready
            )
            raise ValueError('OWNER_CAMERA_NOT_READY:' + details)
        # Periodic health trusts Owner metadata. MJPEG decode and RGB-D capture
        # belong to explicit acceptance, not a timer that runs every few seconds.
        result['checks']['owner'] = True
        result['checks']['source'] = True
        result['owner_cameras'] = [row.get('id') for row in enabled]
    except Exception as exc:
        result['errors'].append('source:' + str(exc))
        # A reachable owner with a missing camera already retries itself.
        if not isinstance(exc, (ValueError,AssertionError)):
            restart.add('owner')
    try:
        first_status = fetch(media + '/push')
        time.sleep(1.2)
        second_status = fetch(media + '/push')
        first_by_id = {row['camera_id']: row for row in first_status.get('streams', [])}
        enabled_streams = [
            row for row in second_status.get('streams', []) if row.get('enabled', True)
        ]
        assert enabled_streams, 'NO_ENABLED_MEDIA_STREAM'
        unavailable = [
            row for row in enabled_streams
            if not row.get('desired') or not row.get('online')
        ]
        assert not unavailable, 'PUSH_NOT_ONLINE:' + ','.join(
            f"{row.get('camera_id', '?')}:{row.get('reason', '')}" for row in unavailable
        )
        comparable = [
            row for row in enabled_streams
            if 'frames_sent' in row and 'frames_sent' in first_by_id.get(row['camera_id'], {})
        ]
        missing_counters = [
            row['camera_id'] for row in enabled_streams if row not in comparable
        ]
        assert not missing_counters, 'PUSH_PROGRESS_COUNTER_UNAVAILABLE:' + ','.join(
            missing_counters
        )
        stalled = [
            row['camera_id'] for row in comparable
            if row['frames_sent'] <= first_by_id[row['camera_id']]['frames_sent']
        ]
        assert not stalled, 'OUTPUT_NOT_ADVANCING:' + ','.join(stalled)
        result['media_streams'] = {
            row['camera_id']: {
                'frames_sent': row['frames_sent'],
                'progress_age_sec': row.get('progress_age_sec'),
            }
            for row in comparable
        }
        result['checks']['push_progress'] = True
        result['checks']['push'] = True
    except Exception as exc:
        result['errors'].append('push:' + str(exc))
        if not isinstance(exc,AssertionError) or result['checks'].get('source'):restart.add('media')
    result['ok'] = not result['errors']
    return result, restart


def repair_service_names(config, repair_units):
    health = config.get('health') or {}
    prefix = str(health.get('service_prefix') or 'vision-head-')
    mapping = health.get('repair_units') or {}
    names = []
    for unit in repair_units:
        targets = mapping.get(unit, unit)
        if isinstance(targets, str):
            targets = [targets]
        for target in targets:
            name = str(target).strip()
            if not name:
                continue
            if not name.endswith('.service'):
                name = prefix + name + '.service'
            names.append(name)
    return names


def update_repair_state(path, result, repair_units, *, config=None, now=None, run=subprocess.run):
    """Persist health state and restart only after bounded consecutive failures."""
    now = time.time() if now is None else now
    state_path = path.parent.parent / 'runtime/head-stream-health.json'
    try:
        previous = json.loads(state_path.read_text())
    except (OSError, ValueError):
        previous = {}
    count = 0 if result['ok'] else previous.get('consecutive_failures', 0) + 1
    last_repair = previous.get('last_repair', 0)
    ordered_units = []
    if (
        count >= REPAIR_AFTER_CONSECUTIVE_FAILURES
        and now - last_repair > 120
        and repair_units
    ):
        start_order = ('rgbd', 'head-source', 'hand_left-source', 'hand_right-source',
                       'owner', 'camera', 'synced-rgbd', 'media')
        ordered_units = sorted(
            repair_units,
            key=lambda unit: (start_order.index(unit) if unit in start_order else len(start_order), unit),
        )
        for service in repair_service_names(config or {}, ordered_units):
            run(['systemctl', 'restart', service], check=True, timeout=20)
        last_repair = now
        result['repair_requested'] = ordered_units
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps(dict(
        result,
        checked_at=now,
        consecutive_failures=count,
        last_repair=last_repair,
    ), indent=2) + '\n')
    return count, ordered_units


def check_ros(config, path, result, repair_units, *, run=subprocess.run):
    if config['rokae']['owner'].get('source') != 'ros':
        return
    root = path.parent.parent
    try:
        probe_command = [
            '/bin/bash', '-c', 'source "$1" && exec "$2" "$3" --config "$4"',
            'vision-probe', str(root/'config/ros/setup.bash'), sys.executable,
            str(root/'scripts/head-stream-ros-check.py'), str(path),
        ]
        if os.geteuid() == 0:
            # The camera ROS participants run as admin. Root can see an empty
            # DDS graph on this host even when the stream is healthy.
            probe_command = ['/usr/sbin/runuser', '-u', 'admin', '--', *probe_command]
        probe = run(probe_command, capture_output=True, text=True, timeout=15)
        if not probe.stdout.strip():
            raise RuntimeError('empty ROS probe output: ' + probe.stderr.strip()[:300])
        ros = json.loads(probe.stdout)
        result['ros'] = ros
        if not ros['ok']:
            result['errors'].append('ROS_RGBD_UNHEALTHY')
            cameras = ros.get('cameras') or {}
            if cameras:
                for camera in cameras.values():
                    if not camera.get('raw_ok'):
                        repair_units.add(camera.get('repair_unit') or 'rgbd')
                    if not camera.get('synced_ok'):
                        repair_units.add('synced-rgbd')
            else:
                if not ros.get('raw_ok'):
                    repair_units.add('rgbd')
                if not ros.get('synced_ok'):
                    repair_units.add('synced-rgbd')
    except Exception as exc:
        result['errors'].append('ROS_PROBE_FAILED:' + type(exc).__name__ + ':' + str(exc)[:300])
        # A broken probe is not evidence that a healthy RGB-D stream should
        # be interrupted. Leave the camera running and report the failure.
    result['ok'] = not result['errors']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--repair', action='store_true')
    args = parser.parse_args()
    path = Path(args.config).resolve()
    config = json.loads(path.read_text())
    result, repair_units = check(config)
    check_ros(config, path, result, repair_units)
    if args.repair:
        update_repair_state(path, result, repair_units, config=config)
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
