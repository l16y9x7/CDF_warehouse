# 中免相机验收清单（主入口 `:8085`）

**冻结决策（2026-09-18）：** 中免 Agent / 相机合同只认 Owner HTTP `:8085`。  
`:8003`（Camera Adapter）与 `:8005`（Media 推流）**不进入**中免验收主路径。

| 项 | 值 |
| --- | --- |
| 设备 | ROBOT_011 / `192.168.130.105` |
| 代码目录 | `/home/admin/vision` |
| Owner | `vision-head-owner.service` → `0.0.0.0:8085`，`source=ros` |
| 相机 | 头部 Orbbec Gemini 335L；腕部现场 disabled |
| 合同角色 | `head` / `left_wrist` / `hand_wrist`（右腕兼容旧输入别名） |

## 验收项（仅此三口）

| # | 接口 | 通过标准 | 2026-09-18 13:18+08 证据 |
| --- | --- | --- | --- |
| 1 | `GET /camera/list` | `ok=true`；`head` ready/online；对外腕部名；`depth.aligned` 可见 | 通过：head color/depth online，aligned=true，1280×720 |
| 2 | `GET /camera/stream?camera=head&type=color` | `multipart/x-mixed-replace`，可抽出 JPEG SOF/EOF | 通过：HTTP 200，约 6.9MB 连续帧含 JPEG |
| 3a | `GET /camera/capture?camera=head` | `ok=true`，`color.path` 落盘 `/shared/frames/{id}/rgb.jpg` | 通过：`capture-2cd3d5514c2f` |
| 3b | `GET /camera/capture?camera=head&streams=color,depth` | `same_shot=true`，depth `aligned=true`，双文件同目录 | 通过：`capture-86d51f195e41`（rgb.jpg + depth_mm.npy，1280×720） |
| — | `GET /camera/health` | 辅助：`status=READY`（非中免主合同，但现场可用） | READY / ok / all_ready |

## 非验收（明确排除）

- `:8003` `/list` `/frame/{id}` — Gateway/内部兼容，非中免主入口  
- `:8005` `/push*` — 上云推流；现场已切 `push.source=ros`（2026-09-18），与 capture 合同独立  
- `stream?type=depth` — 深度不走 stream，仅 capture / ROS Topic  

## 复测命令（本机）

```bash
BASE=http://127.0.0.1:8085
curl -fsS "$BASE/camera/list" | python3 -m json.tool | head
curl -fsS -m 5 -o /tmp/mjpeg.bin -D - "$BASE/camera/stream?camera=head&type=color" | head
curl -fsS "$BASE/camera/capture?camera=head&streams=color,depth" | python3 -m json.tool
```

飞书报告：https://ks2ynpxs58.feishu.cn/wiki/RmU3wV5Ici8Dw7kabHRctepXnTc
