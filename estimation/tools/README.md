# 本地 SAM3 调试页面

Windows PowerShell 启动（后台运行，仅监听本机）：

```powershell
./estimation/tools/start_sam3_debug.ps1
```

打开 <http://127.0.0.1:8765>。默认直连
`http://192.168.3.107:25541/api/v1/segment`，忽略代理环境变量；浏览器通过本地 Python 服务转发请求。
需要 Python 环境包含 `requests`、`numpy`、`opencv-python`。启动器优先使用当前 Conda 环境，
其次尝试本机 `miniconda3/envs/py11/python.exe`，也可以显式指定 `-PythonPath`。
参数 `-Port` 和 `-Sam3Url` 可以更换本地端口和上游地址。启动器输出 PID 及停止命令；
运行信息、标准输出及错误日志位于 `logs/sam3_debug/runtime/`。

也可以前台运行，按 Ctrl+C 停止：

```bash
python estimation/tools/sam3_debug_server.py --port 8765
```

页面支持上传 JPG/PNG/WebP、载入最近 40 条 estimation 日志图片、从当前 SKU 库填入参数、
编辑 prompt/检测阈值/mask 阈值、显示 mask 和检测框、单实例高亮、缩放、导出图片/单个 mask/JSON，
以及恢复本次页面会话最近 6 次结果。点击刷新按钮可重新读取 SKU 库和日志列表。
SKU 下拉框只填入参数，实际请求以输入框为准；不会保存或覆盖 SKU 库。

这里显示的是 SAM3 **原始实例**，不应用 estimation 的纸箱 ROI、面积过滤或位姿估计。
日志图片若带有已配置 `sku_id`，默认使用该 SKU **当前库配置**，而不是历史请求中的旧 prompt。
计时中的 SAM3 往返包含上传、排队、推理及响应传输，并非纯 GPU 推理耗时。

# 在 155 上复现 estimation 上传

把 `mock_estimation_request.py` 复制到 155，使用有 NumPy 的 Python 环境运行。
脚本独立运行，不需要复制整个仓库；发送的是实际 `/infer` 请求，每次只调用一次。

```bash
python3 mock_estimation_request.py \
  --rgb /shared/frames/capture-5e39297885c7/rgb.jpg \
  --depth /shared/frames/capture-5e39297885c7/depth_mm.npy
```

默认地址 `http://192.168.3.107:25540/infer`，类别 `bottle`，选择 `RIGHT`。
默认直连，忽略 `HTTP_PROXY` / `HTTPS_PROXY` / `ALL_PROXY` / `NO_PROXY` 等环境变量。
显式走之前的代理进行对比，加 `--proxy http://192.168.3.107:17891`。
系统 TUN、透明代理和路由仍可能影响直连流量。

默认 K / T 来自旧请求 `20261001_084306_f205f149`，仅用于测速 mock。
它们不是这次图像经过确认的相机参数，不能用默认输出判断当前帧定位精度。
传入 `--metadata /path/to/current_frame_metadata.json` 可使用当前帧真实参数；格式见
`../docs/ROBOT_API_PICK_POSE_INTERFACE.md` 的 `/infer` 请求示例，省略两个 Base64 字段即可。
也可使用已有 `request.json` / `request_metadata.json`：图像字段会替换，服务端添加的
`class_name` / 下划线开头的内部字段会移除，其余参数保留。
`--target-type basket` 切换篮筐，`--sku-typ box --side LEFT` 切换商品类别及箱侧。

`--sku-id demo_cream_box --side LEFT` 可测试服务器 SKU 库；不提供 metadata 时会让服务器
从 ID 推导类别，不强制使用 bottle 样例类别。省略 `--sku-id` 保持旧调用方式。
存在 metadata 时保留其 `sku_typ`，若与 ID 冲突服务端会报错；篮筐模式移除 SKU 字段。

深度必须是与 RGB 对齐的二维 NPY，原始数值单位为 mm；不会从米自动换算。
默认转成 float32 再保存为 NPY 后 Base64 编码，匹配先前请求的深度类型。
因此 uint16 深度可能从约 1.8 MiB 变为 3.5 MiB，Base64 还会再增加约三分之一。
`--depth-dtype preserve` 可保留原浮点文件字节，但服务不接受整数深度。

每次结果写入当前目录下的 `logs/estimation_mock/<时间_随机ID>/`：

- `timing.json`：文件读取、NPY 转换、Base64、JSON、连接、socket 写入、响应等待计时，以及服务器阶段计时。
- `response.json`：完整接口响应，含服务端 `request_id`，可对应 107 上的请求日志。
- `response_body.bin`：原始 HTTP 响应体，也便于检查代理返回的非 JSON 错误。
- `request_metadata.json`：本次发送的参数，不含大体积 Base64。
- `--save-request` 额外保存完整请求；`--prepare-only` 只构造并保存请求，不访问服务。

`body_socket_write` 只量到所有字节交给本地 socket。网络慢时写入会阻塞，但写入完成时
仍可能有字节留在内核发送缓冲区。`wait_response_headers` 因此可能包含剩余上传、
服务器计算及响应传输，不能当作纯推理时间。服务端 `body_read` 是服务器等待读完请求体
的时间；这些时段相互重叠，不应将客户端与服务端阶段直接相加。
`socket_write_chunks` 记录每个 64 KiB 写入的耗时，块间没有人为 sleep；这不是 HTTP chunked 编码。
默认 socket 单次阻塞超时 300 秒，可通过 `--timeout` 调整。超时不会自动重试。
HTTP 非 200 或业务 `ok=false` 时保留响应和计时并返回退出码 1。
