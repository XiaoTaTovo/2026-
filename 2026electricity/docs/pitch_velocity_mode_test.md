# PITCH 速度模式首轮实机测试

## 本次目标

仅验证 X42S 的 Emm 速度模式命令能可靠地启动、停止，并确认两条逻辑方向对应的真实丝杆运动方向。

本固件不做以下事情：

- 不根据 MaixCAM 数据驱动电机。
- 不运行球位 PID。
- 不使用外置角度传感器。
- 不假定 `dir=0/1` 分别对应平台上升或下降。
- 不把电机原始位置直接换算成丝杆毫米数。

## 当前固件行为

上电后先完成 1000 轮 `状态 -> 实时位置` 只读自检。只有出现
`PITCH_READ1000_RESULT=COMM_PASS` 后，按键才可以发送运动相关命令。

| 按键 | 当前动作 |
| --- | --- |
| KEY1, PE11 | 已失能时使能；已使能且停止时失能 |
| KEY2, PE12 | 逻辑正方向：读取位置 -> 10 RPM 运行 300 ms -> 立即停止 -> 再读位置 |
| KEY3, PE13 | 逻辑负方向：读取位置 -> 10 RPM 运行 300 ms -> 立即停止 -> 再读位置 |
| KEY4, PE14 | 立即停止并锁故障，重新上电后才恢复 |

按键均为上拉输入、按下为低电平，按下沿触发，消抖 30 ms。KEY2 与 KEY3 同时按下会被拒绝。

当前固定参数在 `Core/Src/main.c`：

- 地址：`1`
- 逻辑正方向协议值：`0`
- 逻辑负方向协议值：`1`
- 转速：`10 RPM`
- 加速度参数：`100`
- 定时运行：`300 ms`
- 同步标志：`0`

因此，KEY2 的速度帧为 `01 F6 00 00 0A 64 00 6B`，随后自动发送停止帧
`01 FE 98 00 6B`。这是代码与厂商 `Emm_V5_Vel_Control()` 帧格式的一致性说明，不是实机行为证明。

## 测试 0：机械与供电准备

1. 将升降平台置于行程中部，确认正反两个方向都留有足够机械余量。
2. 本轮不要带球做平衡，不以硬限位作为停止手段。
3. 供电限流，手边保留能直接断开 X42S 动力电源的物理方式。
4. 保持 F407、X42S 通讯板和动力电源共地；UART3 为 PB10 TX、PB11 RX。
5. KEY4 的串口停止不是断电急停。出现撞限位、异常加速、异响、发热、掉线或方向判断错误时，先断开电机动力电源。

## 测试 1：上电门禁

烧录后不要按键。蓝牙日志应依次包含以下关键项，日志可交错：

```text
PITCH_VELOCITY_TEST_READY
PITCH_TEST_MODE=TIMED_VELOCITY
PITCH_READ1000_START=1000
PITCH_READ1000_RESULT=COMM_PASS
PITCH_VEL_EVENT=READY,...
PITCH_VELOCITY,state=DISABLED_READY,comm=1,enabled=0,...
```

通过条件：启动和 1000 轮只读自检期间，电机不得自行转动。未看到 `COMM_PASS` 与 `READY` 时，不继续。

## 测试 2：仅使能

短按 KEY1 一次。预期：

```text
PITCH_VEL_EVENT=KEY_PRESS,key=1,...
PITCH_VEL_EVENT=ENABLE_SENT,...cmd=ENABLE,...
PITCH_VEL_EVENT=COMMAND_ACK,...cmd=ENABLE,ack=0x02,...
PITCH_VELOCITY,state=ENABLED_STOPPED,...enabled=1,velocity_cmd=0,...
```

通过条件：不得发生转动。允许出现抱轴或保持力。`enabled=1` 只表示收到了协议应答，不能代替转矩实测。

## 测试 3：KEY2 定时正向脉冲

短按 KEY2 一次，之后不要再次按键，等待状态回到 `ENABLED_STOPPED`。预期：

```text
PITCH_VEL_EVENT=POSITION_BEFORE,...cmd=RUN_POS,value=<原始位置>
PITCH_VEL_EVENT=VELOCITY_SENT,...cmd=RUN_POS,value=10
PITCH_VEL_EVENT=COMMAND_ACK,...cmd=RUN_POS,ack=0x02,...
PITCH_VEL_EVENT=AUTO_STOP_SENT,...cmd=STOP,value=300
PITCH_VEL_EVENT=COMMAND_ACK,...cmd=STOP,ack=0x02,...
PITCH_VEL_EVENT=POSITION_AFTER,...cmd=RUN_POS,value=<原始位置>
PITCH_VEL_EVENT=POSITION_DELTA,...cmd=RUN_POS,value=<原始位置差>
```

记录：平台真实移动方向、是否只运动约 300 ms 后停止、`before/after/delta`、以及
`timeouts/errors/drops`。本步骤把协议 `dir=0` 映射为实际“上升”或“下降”，不能从代码猜测。

## 测试 4：KEY3 定时反向脉冲

仅当测试 3 完全正常时，短按 KEY3 一次并等待结束。记录同样字段。正常情况下，电机原始位置差的符号应与 KEY2 相反；实际数值由电机编码器单位、丝杆导程和运行时间共同决定，当前未定义验收数值。

## 测试 5：KEY4 停止链路

在 `ENABLED_STOPPED` 状态下短按 KEY4，先验证停止帧、应答和故障闭锁链路。当前
300 ms 的自动脉冲不适合人工可靠地在运动中按下 KEY4；它是首轮低能量测试的有意限制。
预期：

```text
PITCH_VEL_EVENT=KEY_PRESS,key=4,...
PITCH_VEL_EVENT=STOP_SENT,...cmd=STOP,...
PITCH_VEL_EVENT=COMMAND_ACK,...cmd=STOP,ack=0x02,...
PITCH_VEL_EVENT=FAULT_LATCHED,...
PITCH_VELOCITY,state=FAULT_LATCHED,...failure=STOP_BUTTON,...
```

KEY1/KEY2/KEY3 在锁故障后均不可用，重新上电才能恢复。若 KEY4 后电机仍持续转动，立刻物理断电；该结果禁止进入后续测试。

在测试 3、4 完全通过且已按实测位移确认机械余量后，才可把 `run_ms` 暂时改为
`1000U`，重新编译并烧录，用于验证“运动中 KEY4”的响应。该修改不是进入 PID 的许可；
测试完成后恢复 `300U`，记录 KEY4 到可见停止的实际时间。

## 本轮通过条件与下一步

首轮通过只代表以下事实：

1. 两个方向均能收到 `0x02` 应答。
2. 每次速度命令都能在定时停止或 KEY4 后停止。
3. 原始位置差与可见运动方向一致，且没有 `timeout/error/drop`。

通过后再做丝杆位移标定：每个方向重复多次定时脉冲，用尺测量实际毫米位移，建立
`raw position/mm` 和实际速度的均值与回程差。PID、视觉自动控制和软行程边界在该标定之前保持禁用。
