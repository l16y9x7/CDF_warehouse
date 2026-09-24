# `GET /camera/capture`

## 参数

| 字段 | 必填 | 默认 | 取值 |
| --- | --- | --- | --- |
| `camera` | 是 | — | `head` \| `left_wrist` \| `hand_wrist`；兼容 `hand_right` / `right_wrist` / `right` 输入 |
| `streams` | 否 | `color` | `color` \| `depth` \| `color,depth`（顺序无关） |
| `format` | 否 | `raw` | 仅含 depth 时生效：`raw` \| `preview`；纯 color 忽略 |

```http
GET /camera/capture?camera=head
GET /camera/capture?camera=head&streams=depth
GET /camera/capture?camera=hand_wrist&streams=color,depth
```

## 响应

成功时分配唯一 `capture_id`，写入 `/shared/frames/{capture_id}/`；未请求的流为 `null`。  
`streams=color,depth` 时必须同次（`same_shot=true`），禁止跨次拼接。  
含 depth 时必须是对齐深度（`aligned=true`）；未对齐直接失败，不静默回退。

短暂无帧：服务端最多重试 3 次（间隔约 80ms）。

```json
{
  "ok": true,
  "capture_id": "capture-002",
  "camera": "hand_wrist",
  "same_shot": true,
  "color": {"path": "/shared/frames/capture-002/rgb.jpg", "format": "jpeg", "width": 1280, "height": 720},
  "depth": {"path": "/shared/frames/capture-002/depth_mm.npy", "format": "raw", "width": 1280, "height": 720, "aligned": true}
}
```

- color 固定 jpeg；depth `raw` 推荐 `depth_mm.npy`（毫米）。

## 失败

```json
{
  "ok": false,
  "error_code": "CAMERA_NOT_READY",
  "message": "camera not ready",
  "camera": "left_wrist"
}
```

| error_code | 含义 |
| --- | --- |
| `CAMERA_NOT_FOUND` | `camera` 非法或不支持 |
| `INVALID_STREAMS` | `streams` 非法（空串、未知项、非法组合） |
| `INVALID_FORMAT` | `format` 不是 `raw` / `preview` |
| `CAMERA_NOT_READY` | 相机未就绪、禁用或 Topic 无帧（已短重试仍失败） |
| `DEPTH_NOT_ALIGNED` | 仅有未对齐 native 深度，拒绝冒充对齐 |
| `CAPTURE_FAILED` | 写入失败或部分流失败；双流须原子：任一侧失败则整体失败，不得半成功 |
