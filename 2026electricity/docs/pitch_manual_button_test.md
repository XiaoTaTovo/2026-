# PITCH 四按钮首轮上板测试

## 本固件做什么

- 上电先完成 1000 轮 `状态 -> 实时位置` 只读通信自检。
- 自检通过前不接受使能或运动按钮。
- 蓝牙只输出遥测，收到的字符会被丢弃，不会变成电机命令。
- AS5600 和视觉 PID 本轮不参与控制。

## 四个按钮

| 按钮 | 引脚 | 短按动作 | 限制 |
| --- | --- | --- | --- |
| KEY1 | PE11 | 已失能时请求使能；已使能且空闲时请求失能 | 忙时拒绝 |
| KEY2 | PE12 | 逻辑正方向相对移动 32 脉冲 | 必须已使能 |
| KEY3 | PE13 | 逻辑负方向相对移动 32 脉冲 | 必须已使能 |
| KEY4 | PE14 | 请求立即停止并锁止 | 最高优先级；必须重新上电才恢复 |

四键均为上拉输入、按下为低电平，消抖时间 30 ms。动作只识别按下边沿，长按不会连发。KEY2 与 KEY3 同时按下会被拒绝。

## 固定测试参数

- 地址：1
- 速度：60 RPM
- 加速度参数：100
- 单步：32 脉冲
- 模式：0，相对上一次输入目标位置（厂商例程定义）
- 命令应答超时：100 ms
- 运动后等待：1200 ms，再读取一次实时位置

若电机当前确为 16 细分且一圈为 3200 脉冲，32 脉冲约等于电机轴 3.6 度。模式 0 会把连续按键命令累加到上一次输入目标；这个换算尚未由电机参数和机械传动比确认，不能当作摆杆角度。

## 测试 0：安全准备

1. 先让电机与水管机构脱开或保持完全卸载，不要直接带球、丝杆或硬限位测试。
2. 电机电源使用限流供电，手边保留可直接断开电机电源的物理手段。
3. 确认 F407、通讯板和电机共地；USART3 仍为 PB10 TX、PB11 RX。
4. KEY4 的串口 STOP 不是断电急停，不能替代物理切断电机电源。

出现异常运动、撞限位、明显发热、异响、电源压降、通信丢失或方向不确定时，立即切断电机电源并保存完整日志。

## 测试 1：只验证上电门禁

烧录后先不要按键，也不要给机构加载。开机日志中应看到以下字段；自检摘要与手动模块日志可能交错，不要求它们严格按此顺序出现：

```text
BT_DMA_READY
PITCH_MANUAL_READY
BT_CONTROL=TELEMETRY_ONLY
PITCH_READ1000_START=1000
...
PITCH_READ1000_RESULT=COMM_PASS
PITCH_COMMUNICATION_GATE=PASS
PITCH_EVENT=READY,...
PITCH_MANUAL,state=DISABLED_READY,comm=1,enabled=0,latched=0,...
```

验收：整个开机和 1000 轮自检过程中电机不得自行转动。若没有 `COMM_PASS` 和 `READY`，不要继续。

## 测试 2：只验证 KEY1 使能

短按 KEY1 一次，预期关键日志：

```text
PITCH_EVENT=KEY_PRESS,key=1,...
PITCH_EVENT=ENABLE_SENT,...
PITCH_EVENT=COMMAND_ACK,...cmd=ENABLE,ack=0x02,...
PITCH_MANUAL,state=ENABLED_IDLE,...enabled=1,latched=0,...
```

验收：不应产生转动；允许出现正常的抱轴/保持力。这里的 `enabled=1` 只表示固件收到了使能命令的 `0x02` 应答，不等同于独立测量了真实转矩。

## 测试 3：只验证 KEY2 一个小步

确认测试 2 通过后，短按 KEY2 一次，不要长按。预期关键日志：

```text
PITCH_EVENT=POSITION_BEFORE,...value=<运动前位置>
PITCH_EVENT=MOVE_SENT,...cmd=MOVE_POS,...value=32
PITCH_EVENT=COMMAND_ACK,...cmd=MOVE_POS,ack=0x02,...
PITCH_EVENT=POSITION_AFTER,...value=<运动后位置>
PITCH_EVENT=POSITION_DELTA,...cmd=MOVE_POS,...value=<位置差>
```

记录电机从操作员视角的真实转向、`before/after/delta` 和是否只动作一次。逻辑正方向暂不等同于摆杆上升方向。

## 测试 4：再验证 KEY3

只有测试 3 正常时才短按 KEY3 一次。预期与 KEY2 相同，但命令为 `MOVE_NEG`。正常情况下位置差符号应与 KEY2 相反；具体数值由电机位置单位和配置决定。

## 测试 5：KEY1 失能

空闲时再短按 KEY1，预期 `DISABLE_SENT`、`ack=0x02`，随后状态为 `DISABLED_READY,enabled=0`。此时再按 KEY2/KEY3 应出现 `REJECT_DISABLED`，不得发送运动命令。

## 测试 6：最后验证 KEY4

重新使能后短按 KEY4，预期：

```text
PITCH_EVENT=KEY_PRESS,key=4,...
PITCH_EVENT=STOP_SENT,...
PITCH_EVENT=FAULT_LATCHED,...value=2
PITCH_EVENT=COMMAND_ACK,...cmd=STOP,ack=0x02,...
PITCH_MANUAL,state=FAULT_LATCHED,...failure=STOP_BUTTON,...
```

锁止后 KEY1/KEY2/KEY3 都不再生效，必须断开电机电源并让 F407 重新上电才能恢复。

## 测试 7：卸载往返阶跃重复性

本测试不改固件、不改速度、不连接摆杆、水管或钢球；只验证已通过安全门禁的 X42S 空载动态重复性。

1. 因 KEY4 已锁止，先让 F407 和 X42S 完整断电再上电，等待 `PITCH_READ1000_RESULT=COMM_PASS`。
2. 保持电机卸载、限流供电，并准备物理断电；短按 KEY1，确认 `ENABLED_IDLE`。
3. 连续完成 10 组：每组严格等待上一条 `PITCH_MANUAL,state=ENABLED_IDLE` 后，短按一次 KEY2，再等待空闲，短按一次 KEY3，再等待空闲。
4. 每次只记录 `POSITION_DELTA`；不要在 `WAIT_SETTLE` 时按下一个键，也不要调速度、加速度、脉冲数或模式。

通过条件：20 次运动都收到 `ack=0x02`；正向 `delta` 在 `+600..+710 raw`，反向 `delta` 在 `-710..-600 raw`；每组正反两步的代数和绝对值不超过 `80 raw`；全程 `timeouts=0`、`errors=0`、`drops=0`。出现异常运动、异响、发热、掉电、通信错误或任何超界位置变化，立刻按 KEY4 并切断 X42S 电源。

本测试通过只接受“空载位置阶跃重复性”。它不接受摆杆角度、丝杆位移、球位视觉或 PID 闭环；下一阶段才接 USART6 的视觉数据做只观测验证。
