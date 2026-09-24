# 服务启停与日常开发

地址 `192.168.130.105`，以下 `systemctl --user` 命令在机器人上以 `admin` 用户运行。

## 常驻层和开发层

| 服务 | 职责 | 日常抓取/扫码/放置开发 |
| --- | --- | --- |
| `mui-sdk.service`（用户服务） | 唯一SDK连接所有者；基础SDK指令转发；三路状态采样与8092缓存查询 | 保持运行，不跟随网页/接口重启 |
| `sr-amr-control.service`（系统服务） | 独立开机底盘ROS节点 | 保持运行 |
| 原三路相机服务 | 独立摄像头驱动与采集 | 保持运行 |
| `mui-control.service`（用户服务） | 抓取/扫码/放置规划及执行流程、8082定位接口、8086动作接口、网页后台API | 后台代码修改后重启 |
| `mui-web.service`（用户服务） | 8091页面及HTTP代理 | 需要时单独重启；静态JS/HTML修改通常刷新即可 |
| `mui.target` | 只打包上面两个开发层服务 | 推荐用它统一启停网页与接口 |

SDK与上报目前在同一常驻进程，这是为了复用同一套控制器连接。上报客户端只读缓存，不建立第二套SDK连接。常驻服务是开机自动运行、故障自动拉起，并非永远不需要维护；SDK通信层、上报采集器和其配置更新才需要单独重启它。

`mui-control`的`Requires=mui-sdk`只会在SDK未启动时拉起它；重启/停止`mui-control`或`mui.target`不会反向停止或重启已经运行的SDK。接口脚本已取消备用底盘子进程启动和退出清理，只订阅现有底盘服务；底盘缺失时显示异常，不代启或重启底盘。相机驱动也不由这两个开发服务管理。

## 推荐命令：网页和接口一起

```bash
# 启动：8091网页、8082定位接口、8086动作接口
systemctl --user start mui.target
# 重启：加载后台流程/接口代码更新
systemctl --user restart mui.target
# 停止：关闭网页和接口，常驻SDK/上报、底盘、相机继续运行
systemctl --user stop mui.target
# 检查
systemctl --user --no-pager status mui.target mui-control.service mui-web.service
```

等价入口：`cd /home/admin/mui/rokae_web_control` 后运行 `./start_hardware.sh start|restart|stop|status`，不带参数为start。

## 单独管理

```bash
# 只重启/停止网页，8082和8086接口继续运行
systemctl --user restart mui-web.service
systemctl --user stop mui-web.service
# 启动网页；如果接口未运行，依赖关系会同时拉起接口
systemctl --user start mui-web.service

# 只启动接口，不启动网页
systemctl --user start mui-control.service
# 停止接口；依赖它的网页也会被停止
systemctl --user stop mui-control.service
# 仅接口开发且本来就不使用网页时
systemctl --user restart mui-control.service
# 需要网页和接口都恢复时，使用统一的 restart mui.target
```

`mui-control.service`就是8082/8086共用的接口服务，并不存在需要额外重启的另一份Agent服务。仅重启8091网页无法加载抓取/扫码/放置的Python修改。

## 重启边界

日常按钮、流程距离/顺序、规划算法、视觉返回解析、动作接口修改，重启`mui.target`即可。修改静态网页通常刷新即可；改变接口环境/端口仍只重启开发层。修改现有动作JSON应按具体读取时机处理，统一重启开发层可确保重新加载。

SDK原生库、`hardware_service.py`、常驻broker协议/采集实现、控制器连接配置及`manipulator_state`采样设置，属于常驻层维护，需要单独说明并确认后重启`mui-sdk.service`。相机驱动/底盘驱动配置修改也分别确认，不纳入日常开发重启组。新SDK方法若不在现有broker允许列表中，需要常驻层更新，不能只重启接口就生效。

重启接口会终止其当前动作会话，应在动作结束后执行。SDK常驻连接保持不变，但退出会话的既有停止/清理逻辑仍保留。`Restart=on-failure`用于故障恢复，手动stop不会自动再启动。

本次上报升级首次加载需一次维护：先停止`mui.target`，重启`mui-sdk.service`，只读检查新上报，再恢复原先运行的网页/接口。此操作必须先取得用户确认；安装代码不等于已重启。现有底盘和相机不需要重启。
