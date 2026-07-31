# 俯仰轴 PID 实时调试流程

## 1. 当前控制门禁

上电后，F407 会先对 X42S 执行 1000 轮只读通信自检。只有同时看到：

```text
PITCH_READ1000_RESULT=COMM_PASS
PITCH_VELOCITY,state=DISABLED_READY,comm=1,enabled=0
PID_DEBUG_BUILD=pid-live-r1
```

才说明通信门禁已通过并且板上运行的是本调试固件。此时电机仍未使能，也不会自动运动。

随后短按 `KEY1`，应看到：

```text
PITCH_VELOCITY,state=ENABLED_STOPPED,comm=1,enabled=1
```

`KEY1` 只负责电机使能/失能。自动闭环还需要蓝牙发送 `A`，两道许可缺一不可。

## 2. 四个按键

| 按键 | PID 调试模式中的作用 |
| --- | --- |
| `KEY1` | 电机使能/失能 |
| `KEY2` | 球脱离/到边缘，立即退出自动模式、发送停止并进入 `hold=1` |
| `KEY3` | 清除 `hold`，只恢复再次 ARM 的许可，不直接启动电机 |
| `KEY4` | 停止并锁存故障，必须重新上电恢复 |

串口停止不能代替物理断电。发生机械撞限位、异常加速、异响或电机未按命令停止时，应直接断开电机动力电源。

## 3. 蓝牙信号测试

在发送任何自动运动命令前，依次发送以下文本，每条以回车或换行结束：

```text
PING
PID?
PID ON
```

预期分别收到：

```text
PONG
PID_CONFIG,...
PID_ON_OK
```

`PID ON` 只打开 PID 波形输出并启用 KEY2/KEY3 的调试语义，不会 ARM 自动控制。

## 4. 首轮扰动测试

1. 水管置于机械行程中部，球放在目标点附近。
2. 等待 1000 轮自检通过，按 `KEY1`，确认 `ENABLED_STOPPED`。
3. 发送 `PID ON`，确认持续收到 `PID_SAMPLE`。
4. 发送 `A`，确认 `AUTO_ARM_OK`、`auto=1`。不要在视觉无效时 ARM。
5. 用手给球一个小扰动，先只做单方向、约 10 mm 的位移。
6. 球掉落、卡住或到达水管边缘时立即按 `KEY2`，确认 `hold=1,motion=0,auto=0`。
7. 在保持状态修改参数，然后按 `KEY3` 或发送 `RESUME`。确认 `hold=0` 后，重新发送 `A`。
8. 测试结束发送 `D`，确认 `AUTO_DISARM_OK`，再按 `KEY1` 失能。

自动速度的执行层硬上限为 `5 RPM`，默认 `AUTOMAX=1`。该上限是已完成手动测试的 `10 RPM` 的一半。一次 ARM 的默认累计运动预算为 `3000 ms`，到时自动停止并退出自动模式。换向必须先 STOP 并收到应答。

## 5. PID_SAMPLE 字段

```text
PID_SAMPLE,t=...,vstate=...,valid=...,reason=...,conf=...,motor=...,x=...,target=...,err=...,ballv=...,p=...,i=...,d=...,u=...,out=...,sat=...,cmd=...,dir=...,fresh=...,age=...,auto=...,motion=...,hold=...,auto_reason=...,fault=...,seq=...
```

| 字段 | 单位和含义 |
| --- | --- |
| `t` | F407 毫秒时基 |
| `vstate` | 视觉状态：`TRACKING/INVALID/STALE/LOW_CONF` |
| `valid/reason/conf/fresh/age` | MaixCAM 检测有效性、无效原因、千分制置信度、新鲜度和总延迟毫秒 |
| `motor` | 电机控制状态，如 `ENABLED_STOPPED/RUNNING_AUTO` |
| `x` | 球位置，`0.1 mm`；负值在目标左侧方向，正值在右侧方向 |
| `target` | 目标位置，`0.1 mm` |
| `err` | `target - x`，`0.1 mm` |
| `ballv` | 球速度，`0.1 mm/s`；正值表示 `x` 增大 |
| `p/i/d` | 比例、积分、微分分量，`0.01 RPM` |
| `u` | 限幅前 PID 输出，`0.01 RPM` |
| `out` | 视觉控制器限幅后的输出，`0.01 RPM` |
| `sat` | `out` 是否被 `MAXRPM` 限幅 |
| `cmd/dir` | 量化后的候选整数 RPM 与 X42S 方向位 |
| `auto/motion/hold/fault` | 自动许可、实际运动、脱离保持和故障锁存 |
| `auto_reason` | 最近一次退出自动模式的原因 |

## 6. 可实时修改的参数

运行中允许修改以下视觉/PID参数，修改后控制器会清空积分和速度历史：

```text
SET KP=0.030
SET KI=0.000
SET KD=0.010
SET ILIM=1.000
SET TARGET=-50
SET DB=20
SET VDB=100
SET CONF=500
SET AGE=150
SET PERIOD=50
SET MINRPM=1
SET MAXRPM=5
SET ALPHA=0.250
SET SIGN=0
SET SAMPLE=100
```

参数含义：

- `KP`：位置误差每增加 1 mm 带来的 RPM。过小响应慢，过大容易往复振荡。
- `KD`：误差变化率的阻尼。增大可抑制快速越过目标，但对速度噪声更敏感。
- `KI`：消除长期静差。首轮保持为 0，PD 稳定后再小幅增加。
- `ILIM`：积分项最大 RPM，防止长时间偏差导致积分累积。
- `DB`：位置死区，单位 `0.1 mm`。
- `VDB`：速度死区，单位 `0.1 mm/s`。位置和速度同时进入死区才停止。
- `ALPHA`：速度低通系数。减小会更平滑但延迟更大，增大会更灵敏但噪声更大。
- `MAXRPM`：视觉控制器输出限幅；电机层还会再次受 `AUTOMAX` 和硬上限约束。
- `MINRPM`：非零输出量化后的最小转速。过大会导致中心附近来回动作。
- `CONF/AGE`：视觉质量门限。任一不满足会停止并退出自动模式。
- `PERIOD`：控制决策周期，毫秒。
- `SAMPLE`：蓝牙波形输出周期，毫秒，不改变控制周期。

以下电机通信和运动安全参数只能在停止且未 ARM 时修改：

```text
SET AUTOMAX=1
SET TIMEOUT=200
SET BUDGET=3000
SET ACCEL=0
SET POSDIR=0
SET NEGDIR=1
SET SYNC=0
```

这些修改只保存在 RAM 中，重新上电会恢复 `main.c` 中的默认值。

## 7. 初始调参顺序

1. 保持 `KI=0`，先只调 `KP` 和 `KD`。
2. 每次只改变一个参数，幅度控制在当前值的约 10% 到 20%。
3. 响应明显过慢且没有振荡时，小幅增加 `KP`。
4. 越过目标后持续振荡时，优先小幅增加 `KD`；若噪声导致方向频繁切换，则降低 `KD` 或减小 `ALPHA`。
5. PD 能稳定回到中心但存在固定偏差时，再从很小的 `KI` 开始，并保持较小 `ILIM`。
6. 每轮至少保留扰动前 1 秒、扰动过程和恢复后 2 秒的完整 `PID_SAMPLE` 原始日志。

没有完整日志时，不能仅凭一两行状态判断 PID 应如何修改。

## 8. PC 端持续监听

`tools/pid_live_monitor.ps1` 独占打开蓝牙串口并持续保存原始数据。调试期间不要再用其他串口软件打开同一个 COM 口。

```powershell
powershell -ExecutionPolicy Bypass -File tools/pid_live_monitor.ps1 `
  -PortName COM7 -SessionDirectory "$env:TEMP\pitch_pid_session"
```

监视器运行期间通过命令队列发送单行命令：

```powershell
powershell -ExecutionPolicy Bypass -File tools/pid_live_send.ps1 `
  -SessionDirectory "$env:TEMP\pitch_pid_session" -Command "PID?"
```

停止监听：

```powershell
powershell -ExecutionPolicy Bypass -File tools/pid_live_stop.ps1 `
  -SessionDirectory "$env:TEMP\pitch_pid_session"
```

`serial.log` 是板端原始数据，`host.log` 记录监视器启动和下发命令的时间。每轮调参必须保留这两个文件。
