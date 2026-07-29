# MaixCAM Pro H 题球位检测交接包

本目录是一版可直接交给视觉队友继续填实物参数的最小实现：已有 YOLO 负责找球，局部背景差/阈值、连通域和圆度负责细化质心，最后沿摆杆轴换算为毫米并发送固定 20 字节 UART 帧。

详细协议、主控接入、STOP/FIX/IMPROVE 和验收表见：

`../../docs/2026-07-29-H题MaixCAM-Pro球位协议与主控接入.md`

## 已确认与未确认

- 实物：MaixCAM Pro，系统 `maixcam-pro-2026-01-24-maixpy-v4.12.5`，MaixPy `4.12.5`，摄像头 ID `ov_os04a10`。
- 官方 MaixPy API：`camera.Camera`、`nn.YOLOv5/YOLOv8/YOLO11`、`uart.UART.write`、`time.ticks_ms/ticks_diff`。
- UNKNOWN：镜头焦距/FOV、模型种类和 `.mud` 路径、球类别号、模型输入分辨率、UART 实际电平、曝光/增益、杆端标定点。

## 队友只需做的事

1. 把适配 MaixCAM Pro/MaixPy 4.12.5 的模型放到板上，填写 `config.py` 的 `MODEL_KIND`、`MODEL_PATH`、`BALL_CLASS_ID`。
2. 用万用表或逻辑分析仪确认 MaixCAM Pro UART 和 MSPM0 都是兼容的 3.3 V 逻辑，再把 `UART_LEVEL_CONFIRMED=True`。不要把 5 V TTL 直接接入。
3. 固定最终相机和分辨率，拍空槽图；在同一张图上量负端与官方 `+5 cm` 方向端的归一化像素坐标，填写 `ROD_NEG_END_NORM/ROD_POS_END_NORM`，再把 `CALIBRATION_CONFIRMED=True`。
4. 从 5000 us、gain 100 起步，一次只改一个量；确认球在全杆范围不过曝、不拖影后冻结参数。
5. 运行 `main.py`。预览左上角显示 `v/r/x`，其中 `x` 的单位是 0.1 mm；无效时固定为 `-32768`。

UART1 默认使用 MaixCAM Pro `A19(TX)`、`A18(RX)` 和 `/dev/ttyS1`，`115200 8N1`。单向发球位时必须连接 `A19 -> MSPM0 RX` 和共地，A18 可不接。

## 文件

- `config.py`：所有需要队友填写的现场参数。
- `algorithm.py`：可在 PC 上测试的一维标定和局部质心修正。
- `ball_protocol.py`：无 MaixPy 依赖的冻结协议及黄金帧。
- `main.py`：MaixPy 4.12.5 入口。
- `../tests/test_maixcam_ball.py`：协议、CRC、标定和合成圆斑测试。

运行 PC 侧测试：

```powershell
cd vision
.\.venv\Scripts\python.exe -m pytest tests\test_maixcam_ball.py -q -p no:cacheprovider
```

