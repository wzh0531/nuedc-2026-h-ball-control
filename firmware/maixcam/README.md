# model-310817：WebRTC录像 + 标定位置 + 可选零点 + UART1

本目录已升级为P24 MaixCAM固定热点版。P17 RTSP/VLC版本仍保留在
`../rtsp_ball_hdemo_uart_310817_roi/`，没有修改。

正式入口为`main.py`。本版不启动RTSP，不使用`JpegStreamer`，也不执行
`to_jpeg()`。

官方接口参考：

- `https://wiki.sipeed.com/maixpy/doc/zh/video/webrtc_streaming.html`
- `https://wiki.sipeed.com/maixpy/api/maix/webrtc.html`

## 1. 图像链与通道所有权

```text
GC4653
 ├─ 主通道：1280x720 NV21 @ 30fps
 │    └─ WebRTC硬件H.264 → 浏览器完整原始画面
 │
 └─ 副通道：640x360 RGB888 @ 30fps
      └─ 每帧一个448x252搜索/跟随ROI
           └─ model-310817
                └─ 框尺寸过滤
                     └─ 标定后轴线过滤
                          └─ 跳变/位置滤波 → UART1
```

程序严格执行：

```python
web_cam = camera.Camera(...FMT_YVU420SP...)
ai_cam = web_cam.add_channel(...FMT_RGB888...)
server.bind_camera(web_cam)
```

必须先创建AI副通道，再绑定WebRTC主通道。`bind_camera()`以后，程序永远不调用
`web_cam.read()`，只读取`ai_cam`。

网页看到的是完整1280×720原始画面。正式配置`ENABLE_DISPLAY=False`，不执行
MaixCAM/MaixVision本机画框显示，以保证约24fps AI性能；检测结果和UART仍正常。

## 2. 三个网络端口

| 端口 | 用途 | 是否录像 |
|---|---|---|
| `8000` | MaixPy官方WebRTC播放页 | 仅观看 |
| `8001` | WebRTC信令，页面自动连接 | 不要手工打开 |
| `8080` | 录像和零点控制页 | 录像、回放、保存、设置/恢复零点 |

`8080`页复用了MaixPy官方页的实际信令协议，直接录制
`video.srcObject`里的远端WebRTC流，不录浏览器按钮，也不对相机主通道做第二次
读取。录像在iPad浏览器内存中暂存，停止后必须点击“保存/分享”。

## 3. MaixCAM端操作

固定热点参数：

```text
SSID：MaixBall-AP
密码：在未提交的`local_config.py`中配置（至少8位）
频段：2.4GHz
信道：6
MaixCAM地址：192.168.66.1
```

首次从其他Wi-Fi切换到Maix热点时，无线MaixVision连接必然中断。建议第一次通过
Type-C连接并运行；程序会继续在MaixCAM上启动热点、WebRTC和AI。

1. 停止旧JPEG、RTSP或其他占用相机的程序。
2. 用MaixVision打开本目录。
3. 确认同目录至少有：

```text
main.py
ball_position.py
calibration_config.py
model_310817.mud
model_310817.cvimodel
```

4. 确认同目录还有`webrtc_record_server.py`，然后运行`main.py`。
5. 正常启动日志应包含：

```text
Program: webrtc_ball_hdemo_uart_310817_roi_p24_maix_ap_v1
WiFi AP: ssid=MaixBall-AP, channel=6, ip=192.168.66.1
WebRTC: 1280x720 NV21 H264 @ 30 fps
AI channel: 640x360 RGB888 @ 30 fps
Camera channels supported: 2
WebRTC URLs (open wlan0/LAN URL on iPad):
  http://192.168.66.1:8000
Browser recorder URLs (open wlan0/LAN URL on iPad):
  wlan0: http://192.168.66.1:8080/
```

若热点已经正确运行，日志会显示`WiFi AP reused ... no restart`，不会重启无线接口。
程序退出时故意保留热点，停止任务后仍能搜索到`MaixBall-AP`属于正常行为。

启动时若只显示一个相机通道，程序会主动报错退出，不会以错误通道结构继续运行。

## 4. iPad打开录像网页

1. iPad打开Wi-Fi并连接`MaixBall-AP`，使用`local_config.py`中配置的密码。
2. 建议先用Safari；也可用最新版Chrome。
3. 打开固定录像地址`http://192.168.66.1:8080/`。
4. 等待状态变为“WebRTC已连接，可开始录像”。
5. 横屏观看，确认左下角显示`1280 × 720`，且画面覆盖完整水管/摆杆。
6. 页面顶部零点区会显示当前零点和最近一次操作状态。

注意：

- 不要使用`usb0`地址；
- 录像页必须使用`8080`，官方纯播放页才是`8000`；
- WebRTC还使用信令端口8001，网络不能拦截设备之间的局域网通信；
- iPad显示“无互联网连接”是正常的，不要切回其他Wi-Fi。

## 5. iPad录像和回放

### 正式录像

1. 画面稳定后点击绿色“开始录像”。
2. 状态栏会显示录像时长。
3. 测评结束点击红色“停止”。
4. 页面下方出现本次录像播放器。
5. 点击“回放”，立即检查视频。

### 回放和导出

1. 点击“保存/分享”。
2. 若弹出系统分享面板，选择“存储到文件”或AirDrop。
3. 若浏览器不支持文件分享，页面会自动尝试下载。
4. 文件优先保存为MP4；当前浏览器不支持MP4录制时自动使用WebM。

不要在保存前刷新或关闭页面，否则内存中的录像会丢失。单段录像上限暂设为
10分钟，比赛建议一轮一段，避免iPad浏览器内存压力。iPad系统屏幕录制可作为
应急备份，但正常测评优先使用网页内录像。

比赛前先做30秒短测，再做一次3～5分钟完整录像，确认：

- 不是黑屏；
- 方向正确；
- 视频没有中途冻结；
- 停止后能立即回放；
- 保存后重新打开仍可播放。

## 6. AI帧率与验收

WebRTC主通道由硬件H.264处理。录像发生在iPad端，不增加MaixCAM第二路相机
读取；AI循环也不执行JPEG压缩。P23关闭逐帧详情，每秒打印一次识别和UART摘要：

```text
STATS inference_fps=24.1 detected=24/24 tracking=1 position_cm=+1.00 zero_cm=+0.00 error_cm=+1.00 dx_uart=8 uart=7B 01 00 08 7D
STATS inference_fps=24.0 detected=0/24 tracking=0 dx_uart=0 uart=7B 00 00 00 7D
```

`detected=24/24`表示这一秒24次有效结果全部识别到钢球；`tracking`和`uart`
是打印时最后一帧的状态。这里只将终端摘要限制为1Hz，硬件UART仍按每次AI结果
发送，约等于`inference_fps`。

真正发生UART短写、UART异常或相机读取异常时，仍会单独打印`[WARN]`。

建议验收：

```text
inference_fps >= 20
uart_failure_total = 0
read_failure_total = 0
WebRTC连续播放10分钟
```

分别在“不打开iPad网页”和“iPad已播放”两种状态记录10行`STATS`。两组
`inference_fps`应接近；若差异超过3fps，需要进一步检查Wi-Fi、温度和编码负载。

## 7. 常见问题

### 页面打不开

1. 确认`main.py`仍在运行；
2. 确认iPad仍连接`MaixBall-AP`；
3. 确认地址是`http://192.168.66.1:8080/`；
4. 改用最新版Chrome；
5. 关闭移动数据或VPN后重试；
6. 忘记该网络后重新输入`local_config.py`中配置的密码；
7. 确认日志出现`WebRTC URLs`和`Browser recorder URLs`。

### 页面能打开但没有视频

1. 点击页面中的播放区域；
2. 检查浏览器是否禁用了自动播放；
3. 切换Chrome/Safari；
4. 确认程序没有`WebRTC bind_camera failed`或`WebRTC start failed`；
5. 用同一网络的电脑浏览器做对照；
6. 在无互联网热点下单独测试，确认当前固件的ICE连接行为。

### AI帧率低

WebRTC版没有逐帧JPEG压缩。如果仍低于20fps，优先检查：

- 是否误读了WebRTC主通道；
- 模型推理耗时；
- `dual_buff`是否仍启用；
- MaixCAM温度；
- 本机调试显示开销；
- 是否存在其他占用硬件编码器的程序。

### 网页没有检测框

这是设计行为。主通道交给WebRTC硬件编码后不能读取或回写，因此网页显示完整
原始画面。识别坐标、过滤状态和串口数据仍正常输出。

## 8. 标定、可选零点与UART

正式控制前必须先运行`calibrate.py`。位置厘米值依赖真实安装状态下的
`AXIS_START_PX/AXIS_END_PX`和物理端点厘米值；未标定时示例坐标不能用于控制
电机。未标定状态下AI、热点、WebRTC和录像仍运行，但UART只能发送：

```text
7B 00 00 00 7D
```

当前实测标定为左端`-11.5 cm`、右端`+11.5 cm`，因此正方向朝物理右侧，并与原有电机控制方向一致。
默认零点为物理`0.0 cm`，程序每次重启都会恢复该默认值，不做掉电保存。

在8080页面修改零点：

1. 把钢球放到希望作为零点的位置；
2. 保持静止，等待至少5个有效检测帧；
3. 点击“将当前钢球位置设为零点”；
4. 等待状态由`pending`变为`success`；
5. 若要恢复物理中心，点击“恢复中心零点”。

以下情况会返回`failed`并保留旧零点：

- 尚未完成标定；
- 当前未检测到钢球；
- 最近有效样本不足5帧；
- 最近5帧位置极差超过0.30 cm；
- 当前结果为`stale`、`jump`、`miss`或`axis_filter`错误。

网页线程只提交请求，真正的钢球位置读取和零点修改由AI主循环执行。网页断开或
Wi-Fi断开后，已选零点仍保存在当前MaixCAM进程内，视觉和UART闭环继续运行；
网页连接状态不是`tracking`条件。

UART物理接口和固定5字节帧格式完全没有变化：

```text
UART1
A19=TX
A18=RX
115200 8N1
7B tracking dx_hi dx_lo 7D
```

只改变了偏差的参考零点和换算含义：

```text
error_cm = position_cm - selected_zero_cm
dx_uart = clamp(round(error_cm × 8.0), -112, 111)
```

即`+1 cm → +8`、`-1 cm → -8`、`+5 cm → +40`、`-5 cm → -40`。
`tracking=1`仅表示标定有效且当前钢球检测有效；丢球或未标定仍发送
`7B 00 00 00 7D`。MaixCAM不执行第3问的`+5/-5`任务时序。

比赛任务建议：

- 第3问：使用“恢复中心零点”，由MSPM0执行`+5/-5 cm`任务；
- 第4、5问：使用“恢复中心零点”，控制偏差趋近0；
- 第6问：测试开始前把球放到裁判指定位置，稳定后点击“将当前位置设为零点”；
- 第2问视觉失效时只输出`tracking=0`，不尝试替代滚球评分动作。

所有设零操作必须在测试开始前完成。比赛运行中不得再次点击设零按钮或进行其他
人为干预。
