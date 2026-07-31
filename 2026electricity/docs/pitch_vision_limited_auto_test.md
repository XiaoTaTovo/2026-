# 视觉受限自动模式首轮实物测试

## 当前安全边界

- 上电默认不自动运动。
- `KEY1` 只负责电机使能/失能。
- 蓝牙串口发送 `A` 才会进入自动模式；发送 `D` 退出自动模式。
- 自动速度在电机执行层硬限幅为 `1 RPM`，视觉候选即使更大也不会透传。
- 单次 ARM 的累计运动预算为 `3000 ms`。达到预算后立即 STOP 并退出自动模式。
- 连续 `200 ms` 没有新视觉决策，立即 STOP 并退出自动模式。
- `INVALID`、`LOW_CONF`、`STALE` 均立即 STOP 并退出自动模式。
- 换向必须先 STOP、收到 ACK，再发送相反方向速度。
- `KEY4` 保留为停止并故障锁存，解除锁存需要复位。

`3000 ms` 预算来自现有实测：`10 RPM` 运行 `300 ms` 约升降 `0.1 mm`，
因此 `1 RPM` 运行 `3000 ms` 约对应同一电机转角。它只是首轮方向验证预算，
不是机械软限位。当前没有可靠的允许升降范围，不能据此长期闭环。

## 启动确认

等待以下状态出现：

```text
PITCH_VELOCITY,state=DISABLED_READY,comm=1,enabled=0,...
```

同时确认视觉行持续满足：

```text
PITCH_VISION,state=TRACKING,motion=GATED_AUTO,valid=1,fresh=1,...
```

若 `crc`、`format`、`semantic`、`rx_overflow` 或 `uart_error` 增长，不进入自动模式。

## 第一次测试顺序

1. 取下钢球，人工把水管放在机械中性位置，手放在断电位置附近。
2. 按 `KEY1`，确认 `enabled=1`、`state=ENABLED_STOPPED`，电机不转。
3. 不发送 `A`，放入钢球并轻推，确认仍没有自动运动。
4. 把球放在中心附近，蓝牙串口发送大写 `A`。
5. 确认出现 `PITCH_VEL_EVENT=AUTO_ARMED`，中心附近电机应保持停止。
6. 只把球移到中心左侧约 `10~20 mm`，不要让球自由高速滚动。
7. 预期出现 `AUTO_VELOCITY_SENT`，方向位为 `0`，速度为 `1 RPM`。
8. 电机应逆时针，使相机画面右端降低；最多累计运动约 `3 s` 后自动停止。
9. 发送 `D`，再按 `KEY1` 失能。

任何方向相反、连续运动超过约 `3 s`、水管接近机械极限或出现异响时，立即按
`KEY4`，必要时直接切断电机电源。

## 需要回传的最小日志

只截取一次测试前后各一条：

```text
PITCH_VELOCITY,...
PITCH_VISION,...
PITCH_VEL_EVENT=AUTO_ARMED,...
PITCH_VEL_EVENT=AUTO_VELOCITY_SENT,...
PITCH_VEL_EVENT=AUTO_STOP_SENT,...
```

并说明实际电机转向以及水管右端是升高还是降低。首轮只验证控制方向和停机边界，
不评价 PID 平衡效果。
