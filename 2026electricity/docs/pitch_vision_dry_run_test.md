# PITCH 视觉干运行测试

## 本次目标

验证 MaixCAM 到 F407 的 `BallObservation v2` 帧、位置符号和毫米到 X42S
脉冲的换算。此版本固定为 `PITCH_VISION_MOTION=DRY_RUN`：不会向 USART3
发送任何由视觉产生的运动命令，按钮手动控制逻辑保持原样。

## 已冻结的接口

| 项目 | 当前值 | 依据 |
| --- | --- | --- |
| 视觉串口 | USART6, PC6 TX / PC7 RX, 115200 8N1 | 现有 `.ioc` |
| 帧格式 | BallObservation v2, 20 byte, CRC16-Modbus | `Modules/ball_observation_protocol.c` |
| 控制周期 | 50 ms | 初始软件预算，待实测 |
| 最大观测年龄 | 150 ms | 初始安全门限，待端到端延迟测试 |
| 目标球位置 | `0.0 mm` | 当前坐标零点约定 |
| 死区 | `+-0.5 mm` | 初始防抖门限 |
| 标定 | `960 pulse/mm` | 单次粗测：KEY2 30 次、每次 32 pulse，约上升 1 mm |
| 最大单次候选 | `0.20 mm = 192 pulse` | 初始低能量限幅 |
| PID | `Kp=0.10, Ki=0, Kd=0` | 未标定，仅用于干运行换算观察 |

`960 pulse/mm` 不是丝杆导程的最终结论。完成至少 5 组往返尺量标定后，只修改
`Core/Src/main.c` 中 `pitch_vision_config.pulses_per_mm`。

## 手动测试

1. 手动烧录本版本 ELF；不要按 KEY4，等待 `PITCH_READ1000_RESULT=COMM_PASS`。
2. MaixCAM 的 TX 接 F407 `PC7/USART6_RX`，两端共地，双方为 3.3 V TTL。
3. 用蓝牙观察启动行：
   ```text
   VISION_UART_DMA_READY
   PITCH_VISION_READY
   PITCH_VISION_MOTION=DRY_RUN
   PITCH_VISION_CAL_PULSES_PER_MM=960
   ```
4. 在相机中让球稳定停在坐标零点附近。预期 `state=TRACKING`，
   `fresh=1`，`age_ms<=150`，`pulses=0`。
5. 将球移到视觉坐标负侧约 `-10.0 mm`。按当前约定，预期
   `err_0p1mm=100`、`u_0p001mm=200`、`pulses=192`、`dir=1`。
   这只证明坐标到脉冲的计算正确，不证明电机方向正确。
6. 遮住球或停止视觉发送。预期最新状态变成 `INVALID` 或在 150 ms 后变成
   `STALE`，且不会产生新的 `candidates`。

## 蓝牙字段

```text
PITCH_VISION,state=TRACKING,motion=DRY_RUN,valid=1,fresh=1,
seq=...,x_0p1mm=...,conf=...,age_ms=...,err_0p1mm=...,
u_0p001mm=...,dir=...,pulses=...,frames=...,invalid=...,
stale=...,duplicate=...,candidates=...,crc=...,format=...,
semantic=...,rx_overflow=...,uart_error=...
```

- `x_0p1mm`、`err_0p1mm`: 单位为 `0.1 mm`。
- `u_0p001mm`: 本周期 PID 建议的管道位移，单位为 `0.001 mm`。
- `pulses`: 由 `abs(u_mm) * 960` 四舍五入所得，并被 `192 pulse` 限幅。
- `fresh=0`、`state=INVALID/STALE/LOW_CONF` 时，不允许把该帧接入电机命令。
- `crc/format/semantic/rx_overflow/uart_error` 必须保持 0，才能进入下一步。

## 下一阶段门槛

在满足以下条件前，不接通视觉到电机的自动命令出口：

1. 连续 60 s，`crc/format/semantic/rx_overflow/uart_error=0`。
2. `age_ms` 的最大值不超过 150 ms，视觉帧率和 `seq` 单调递增的证据齐全。
3. 已确认视觉坐标正负与实际球的移动方向一致。
4. 已做独立低能量方向试验，确认 `dir=1/0` 分别对应的实际管道运动方向。
5. 完成至少 5 组正反向机械标定，建立平均值和回程误差。
