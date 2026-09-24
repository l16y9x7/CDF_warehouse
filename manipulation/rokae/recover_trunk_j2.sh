#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="/home/admin/mui/xCoreSDK-Python-AR-v0.7.1.ar_4/rokae_xcore"

export LD_LIBRARY_PATH="$SDK_DIR:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$SDK_DIR:${PYTHONPATH:-}"

exec /usr/bin/python3 "$SCRIPT_DIR/tools/recover_trunk_j2.py" "$@"
