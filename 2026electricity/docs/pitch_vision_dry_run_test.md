# PITCH 视觉干运行测试

> 此文档保留的是历史干运行步骤。当前固件已进入受限自动验证阶段，实物测试以
> `docs/pitch_vision_limited_auto_test.md` 为准。

## 本次目标

验证 MaixCAM 到 F407 的 `BallObservation v2` 帧、位置符号、小球速度估计和
速度模式候选。此版本固定为 `PITCH_VISION_MOTION=DRY_RUN`：不会向 USART3
发送任何由视觉产生的运动命令，按钮手动控制逻辑保持原样。

## 已冻结的接口

| 项目 | 当前值 | 依据 |
| --- | --- | --- |
| 视觉串口 | USART6, PC6 TX / PC7 RX, 115200 8N1 | 现有 `.ioc` |
| 帧格式 | BallObservation v2, 20 byte, CRC16-Modbus | `Modules/ball_observation_protocol.c` |
| 控制周期 | 50 ms | 初始软件预算，待实测 |
| 最大观测年龄 | 150 ms | 初始安全门限，待端到端延迟测试 |
| 目标球位置 | `-5.0 mm` | 锁定水管几何后，实测机械中心对应视觉坐标约 `-5.1 mm` |
| 位置死区 | `+-2.0 mm` | 初始防抖门限 |
| 速度死区 | `+-10 mm/s` | 初始静止判定门限 |
| 速度范围 | `1~5 RPM` | 初始低能量限幅 |
| PD | `Kp=0.03 RPM/mm, Kd=0.01 RPM/(mm/s)` | 未标定，仅用于干运行观察 |
| 速度滤波 | `alpha=0.25` | 一阶低通初值 |

实测 KEY2/KEY3 在 `10 RPM`、`300 ms` 下单次升降约 `0.1 mm`，该结果尚未
直接用于本阶段的候选 RPM 计算。

## 手动测试

1. 手动烧录本版本 ELF；不要按 KEY4，等待 `PITCH_READ1000_RESULT=COMM_PASS`。
2. MaixCAM 的 TX 接 F407 `PC7/USART6_RX`，两端共地，双方为 3.3 V TTL。
3. 用蓝牙观察启动行：
   ```text
   VISION_UART_DMA_READY
   PITCH_VISION_READY
   PITCH_VISION_MOTION=DRY_RUN
   PITCH_VISION_CONTROL=BALL_PD_TO_RPM
   PITCH_VISION_SPEED_LIMIT_RPM=5
   ```
4. 在相机中让球稳定停在坐标零点附近。预期 `state=TRACKING`，
   `fresh=1`，球静止后 `speed_rpm=0`。
5. 将球稳定放在视觉坐标负侧约 `-100.0 mm`。预期 `err_0p1mm` 为正、
   `dir=0`、`speed_rpm` 约为 `3`。
6. 将球稳定放在视觉坐标正侧约 `+100.0 mm`。预期 `err_0p1mm` 为负、
   `dir=1`、`speed_rpm` 约为 `3`。
7. 球快速接近中心时，`ball_v_0p1mm_s` 和速度阻尼可能让候选方向短暂反转，
   用于提前制动。
8. 遮住球或停止视觉发送。预期最新状态变成 `INVALID` 或在 150 ms 后变成
   `STALE`，且 `speed_rpm=0`、不会产生新的 `candidates`。

## 蓝牙字段

```text
PITCH_VISION,state=TRACKING,motion=DRY_RUN,valid=1,fresh=1,
seq=...,x_0p1mm=...,conf=...,age_ms=...,max_age_ms=...,
err_0p1mm=...,ball_v_0p1mm_s=...,rpm_0p01=...,dir=...,
speed_rpm=...,frames=...,invalid=...,
stale=...,duplicate=...,candidates=...,crc=...,format=...,
semantic=...,rx_overflow=...,uart_error=...
```

- `x_0p1mm`、`err_0p1mm`: 单位为 `0.1 mm`。
- `ball_v_0p1mm_s`: 滤波后的小球速度，单位为 `0.1 mm/s`。
- `rpm_0p01`: 带符号的 PD 输出，单位为 `0.01 RPM`。
- `speed_rpm`: 按电机整数 RPM 接口量化后的候选速度，范围 `0~5 RPM`。
- `dir=0` 对应 KEY3 逆时针，`dir=1` 对应 KEY2 顺时针。
- `fresh=0`、`state=INVALID/STALE/LOW_CONF` 时，不允许把该帧接入电机命令。
- `crc/format/semantic/rx_overflow/uart_error` 必须保持 0，才能进入下一步。

## 下一阶段门槛

在满足以下条件前，不接通视觉到电机的自动命令出口：

1. 连续 60 s，`crc/format/semantic/rx_overflow/uart_error=0`。
2. `age_ms` 的最大值不超过 150 ms，视觉帧率和 `seq` 单调递增的证据齐全。
3. 已确认视觉坐标正负与实际球的移动方向一致。
4. 已做独立低能量方向试验，确认 `dir=1/0` 分别对应的实际管道运动方向。
5. 完成至少 5 组正反向机械标定，建立平均值和回程误差。
