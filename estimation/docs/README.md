# No-action bottle / box / tube Spatial Localization HTTP Service

> **2026-09-21 strict protocol switch**: the formal category field is `sku_typ` with values `bottle`/`box`/`tube` (Avene→bottle, estee→box, origins→tube). Requests containing `sku_id` or `class_name` are rejected with HTTP 400; responses output `sku_typ` and no longer include `sku_id`; `/health` reports `supported_sku_types`. Dated sections below keep their historical names.

## Unified SKU recognition

Every supported `target_type=sku` request uses the same recognition pipeline: box SAM3, image-left/right box selection, product SAM3 on the full RGB, selected-box ROI filtering, optional chassis-space front-row filtering, then the SKU-specific geometry fit. `CLASS_CONFIG` contains only per-SKU prompts and thresholds; there is no `direct`/`box_first` mode switch.

- `bottle`: product prompt `The main cylindrical body of each white bottle`, then the existing known-radius cylinder fit. The formal result is `reference_point_camera_mm`, defined as the midpoint of the robust visible fitted body-axis segment; `z_ref_mm` is no longer used for the formal bottle point.
- `box`: product prompt for visible blue-green box tops, then the existing top-surface fit. The formal result is `top_point_camera_mm`.
- `tube`: product prompt for individual gray-green package ends, then the existing top-edge fit. The formal result is `top_edge_center_camera_mm` with matching endpoints.

All SKUs default to `full_image_filter`: the second SAM3 call sees the complete RGB, then complete masks are filtered by centroid membership and at least 90% area inside the selected box ROI. `crop_box` remains an explicit diagnostic option. A cardboard mask is never used to erase product pixels.

Each request may optionally include `front_rule` (`front_axis_chassis`, optional `front_origin_chassis`, and `front_band_mm`). When omitted, the server uses the regression defaults `[1,0,0]`, `[0,0,0]`, and `100000 mm`; explicit values override them. After box-ROI membership, each product mask's valid-depth centroid is transformed to `chassis_link`; candidates outside the configured band are discarded before category selection. These defaults are a wide integration-test band, not a site-calibrated production rule. A front rule is a scene configuration, not an automatically inferred ground or box-front plane.

With `front_rule`, the selected existing box mask is also reused—there is no third SAM3 call—to fit a near, chassis-vertical front-panel plane. The service returns the visible panel upper-edge midpoint and plane in both camera and chassis coordinates under separate `front_panel_valid` gating.

The class table supplies defaults for SAM3 prompts and thresholds; SKU-specific geometry settings retain their existing server defaults. Request overrides are optional and apply only to that request.

The deprecated `box_selection.enabled` field is accepted only for request compatibility and recorded as `legacy_enabled_requested`; even `false` cannot bypass the required box stage. Prefer `side=LEFT/RIGHT`, or supply `box_selection.target_box=1/2` in diagnostic calls.

Responses report the actual `sam3_call_count`; `recognition_mode` was removed. Every SKU uses 2 calls after successful box selection and 1 call when box selection fails. No branch automatically retries, changes targets, changes geometry methods, or falls back to another instance.

## Runtime and boundaries

The service accepts current RGB, aligned float depth in millimetres, K, and per-frame `T_chassis_camera`. It calls the existing SAM3 service at `http://127.0.0.1:18003/infer` and imports the existing bottle/box geometry modules; it does not load another SAM3 model and never commands a robot.

On 4090-1 the wrapper listens on port 18082. Do not restart the 18003 upstream when updating this wrapper.

`test_client.py` is the only client test entry point:

```powershell
$py = 'C:\Users\14817\miniconda3\envs\pt\python.exe'
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' --sku-typ bottle
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' --sku-typ box --target-box 1
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' --sku-typ box --target-box 2
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' --sku-typ tube --target-box 1
```

For a one-command endpoint selection on Windows, use the thin wrapper below. It only selects the HTTP URL; it does not start, stop, or reconfigure either service. The default is the 4090-1 service:

```powershell
& 'D:\TermiTech\Cosmetics_Sort\deploy\run_test_client.ps1' -Target 4090-1 -SkuTyp bottle
& 'D:\TermiTech\Cosmetics_Sort\deploy\run_test_client.ps1' -Target 4090-1 -SkuTyp box -TargetBox 1
& 'D:\TermiTech\Cosmetics_Sort\deploy\run_test_client.ps1' -Target board -SkuTyp bottle
```

As of 2026-09-18, `192.168.130.136:18082` is healthy and the board endpoint `192.168.130.105:18082` is not listening. Therefore use `-Target 4090-1`; the board target is retained only for an explicit future diagnostic service.

For `tube`, the returned point is the midpoint of the fitted **visible upper edge segment**, using valid depth only within the configured inward band from the mask upper contour. It is a diagnostic near-edge interior reference, not a reconstructed physical seal center or a fixed `chassis_link` height.

Exit codes: `0` means the server quality gates and client-side selection/finite-XYZ/class geometry checks all passed; `2` means HTTP/network/request/response-schema failure; `3` means localization or client verification failed. HTTP 200 alone is never treated as localization success.

Field definitions and examples are in [ROBOT_API_HANDOFF.md](ROBOT_API_HANDOFF.md). This remains a diagnostic localization interface, not proof of robot grasp success, physical millimetre accuracy, or a validated ground/grasp height.

## Shared geometry pipeline (2026-09-18)

Every request now creates one request-scoped geometry context. The context builds the aligned RGB-D image's camera-coordinate point map once (depth remains millimetres), applies the request's `T_chassis_camera` once, and reuses the resulting `chassis_link` point map for the selected mask, erosion variants, and edge pixels. Category modules still fit in their established geometry:

```text
RGB + aligned depth + K + T_chassis_camera
        -> validate/reshape inputs
        -> one camera point map (mm)
        -> one chassis_link point map (mm)
        -> class-specific mask/ROI filtering
        -> bottle cylinder / box top-plane / tube top-edge fit
        -> category quality gates
        -> camera-frame XYZ plus diagnostics/artifacts
```

The cache changes only preprocessing; it does not merge the fitting algorithms, change their thresholds, or convert a chassis point a second time. Responses include `diagnostics.geometry_preprocess` counters. A reported `ok=true` still means only that the selected category's geometric quality gates and client checks passed; it is not a robot-action or physical-accuracy guarantee.

## SAM3 backend selection and board migration status

`server.py` has two explicit upstream modes selected before process start:

```bash
SAM3_BACKEND=legacy_18003
SAM3_URL=http://127.0.0.1:18003/infer
```

or:

```bash
SAM3_BACKEND=multipart_segment
SAM3_URL=http://211.137.21.33:25541/api/v1/segment
SAM3_MASK_THRESHOLD=0.5
SAM3_TIMEOUT_S=180
```

`multipart_segment` sends only lossless PNG image bytes plus `prompt`, `threshold`, and `mask_threshold`. Depth, K, transforms, robot state, and credentials are not sent upstream. The service does not automatically fall back between backends.

A no-action diagnostic instance is installed on `192.168.130.105:18082`, with geometry and artifacts executed on the board. The original Avene 0/5 migration result was an invalid configuration test: it sent the class name `Avene` as the SAM3 prompt. With the corrected prompt, a 2026-09-17 five-run check detected seven body instances on every run, selected #3 at score `0.7521561`, and passed the axis/reference/client gates 5/5. Origins is still conservatively rejected because the box response includes a merged two-box proposal, so the three-class migration is **not accepted as the default workflow**. The Windows default URL remains the previous 4090-1 service and the board's pre-migration `test_client.py` remains the default client. Use `--url http://192.168.130.105:18082/infer` only for explicit migration diagnostics.

The new upstream is plain HTTP. Image transport is not encrypted, and queueing/network availability contributes to latency.

The board diagnostic instance was started with the following effective command (manually managed, not systemd):

```bash
cd /home/admin/sam3/deploy
nohup env MPLBACKEND=Agg \
  SAM3_BACKEND=multipart_segment \
  SAM3_URL=http://211.137.21.33:25541/api/v1/segment \
  SAM3_MASK_THRESHOLD=0.5 SAM3_TIMEOUT_S=180 \
  FIT_SCRIPT=/home/admin/sam3/deploy/fit_bottle_axis.py \
  ESTEE_FIT_SCRIPT=/home/admin/sam3/deploy/fit_estee_box_top_surface.py \
  TUBE_FIT_SCRIPT=/home/admin/sam3/deploy/fit_tube_top_edge.py \
  BOX_SELECTION_SCRIPT=/home/admin/sam3/deploy/box_selection.py \
  AXIS_SERVICE_OUTPUT=/home/admin/sam3/deploy/requests \
  PYTHONPATH=/home/admin/sam3/deploy/vendor \
  python3 /home/admin/sam3/deploy/server.py --host 0.0.0.0 --port 18082 \
  > /home/admin/sam3/deploy/logs/server_18082.log 2>&1 &
```

## 2026-09-18：A800 `quinn-server` 25540 手动启动验证

新的 A800 服务目录为 `/home/quinn/cosmetics_pose/deploy`（2026-09-19 由 `/home/quinn/pose` 改名），对外定位端口为 `25540`，上游 SAM3 为同机 `127.0.0.1:25541`。`start_25540.sh` 已改为可直接创建 tmux 会话：

```bash
cd /home/quinn/cosmetics_pose/deploy
./start_25540.sh
```

会话名为 `pose_25540`；查看使用 `tmux attach -t pose_25540`，停止使用 `tmux kill-session -t pose_25540`。这是手动 tmux 启动，不等同于系统开机自启动。

Windows 端通过 `http://211.137.21.33:25540/infer` 实测 Avene 可见轴段中心模式请求通过：HTTP 200、2 次 SAM3、7 个候选、选择 #3（score `0.7521561384`），轴线和可见轴段中心点质量门通过，客户端复核 `accepted=true`。本次参考点转换到 chassis 后 Z 约 `1188.74 mm`，不再要求等于固定 `1200 mm`。这只证明 Avene 当前无动作诊断链路，不代表 estee/origins 或机器人动作验收完成。
