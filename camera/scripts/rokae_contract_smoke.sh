#!/usr/bin/env bash
# 合同冒烟：Owner + Camera + 可选 Media（基址均可配）
# 用法:
#   bash vision.sh start
#   bash scripts/rokae_contract_smoke.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OWNER_BASE="${OWNER_BASE:-http://127.0.0.1:8085}"
CAMERA_BASE="${CAMERA_BASE:-http://127.0.0.1:8003}"
MEDIA_BASE="${MEDIA_BASE:-http://127.0.0.1:8005}"
CHECK_MEDIA="${CHECK_MEDIA:-1}"
SNAP_DIR="${VISION_SMOKE_SNAP_DIR:-$ROOT_DIR/runtime/smoke-snaps}"
mkdir -p "$SNAP_DIR"

curl_json() {
  local url="$1"
  echo "== GET $url"
  curl -fsS --max-time 5 "$url"
  echo
}

curl_json "$OWNER_BASE/camera/health"
curl_json "$OWNER_BASE/camera/list"
for cam in head hand_left hand_right; do
  code="$(curl -sS -o "$SNAP_DIR/vision-snap-$cam.jpg" -w '%{http_code}' --max-time 5 \
    "$OWNER_BASE/camera/snapshot?camera=$cam&type=color" || true)"
  echo "snapshot $cam -> HTTP $code bytes=$(wc -c <"$SNAP_DIR/vision-snap-$cam.jpg" 2>/dev/null || echo 0)"
done

curl_json "$CAMERA_BASE/health"
curl_json "$CAMERA_BASE/list"
curl_json "$CAMERA_BASE/frame/head"

if [[ "$CHECK_MEDIA" == "1" ]]; then
  curl_json "$MEDIA_BASE/health" || true
  curl_json "$MEDIA_BASE/stream/head" || true
  curl_json "$MEDIA_BASE/push" || true
fi

echo "OK contract smoke finished"
