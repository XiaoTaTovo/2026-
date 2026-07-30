# 2026 H 题主控 CCS 工程

这是 MSPM0G3507 的 2026 H 题纯主控工程。默认且当前唯一开放的正式模式是要求 2
（B2）：从 A 点出发，依次走 1.5 m 直线、R500 右半圆、1.5 m 直线、R500 右半圆，
识别 A 点横向标志后把车上测试位置停回 A 点。

## 验证边界

| 层级 | 状态 | 证据 |
| --- | --- | --- |
| SOURCE | 通过 | 只有 2026 H 模式；电机只经 TB6612 执行 |
| HOST | 通过 | 严格 C11 与 GCC `-fanalyzer`；路线、安全、IMU、TB6612、VOFA 测试 |
| SYSCONFIG | 通过 | SysConfig 1.28.0 严格生成；WWDT 4 s；无电机串口外设 |
| BUILD | 通过 | CCS 21.0.0 + TI Arm Clang 5.1.1，0 warning / 0 error |
| PROGRAM | 未执行 | 当前正式 B2 哈希尚未烧录；旧诊断镜像的板级结果不能替代本镜像 |
| BOARD | 未执行 | 当前正式 B2 的方向、编码器符号、急停、断线保护和传感器均待实测 |
| END_TO_END | 未执行 | 尚未证明 B2 小于等于 20 s、停车偏差小于等于 2 cm |

编译成功只说明程序可生成 `.out`，不能替代实车证据。

## 唯一电机链

```text
路线给左右轮 mm/s
  -> CarDrivePort
  -> TB6612Drive 把 mm/s 换成目标 RPM
  -> 每 50 ms 用编码器算实际 RPM
  -> 前馈 + PI 得到 PWM 百分比
  -> PA14/15、PA16/17 控方向，PB12/PB4 输出 PWM，PA28 控 STBY
```

工程的执行器对象、反馈、保护和调参全部归属本地 TB6612/编码器链，没有后端切换开关。
PA8/PA9 仅作为无上下拉的普通输入保留，不初始化 UART1。工程中不存在串口电机协议、
串口电机中断或串口电机驱动对象。

## B2 控制

每个轨迹段都先用几何给出基础轮速，再叠加红外误差：

```text
v_left  = v * (1 - kappa * b_eff / 2) + correction
v_right = v * (1 + kappa * b_eff / 2) - correction
e        = track_position
d_raw    = (e - e_prev) / dt
alpha    = dt / (tau_d + dt)
d_f      = d_f_prev + alpha * (d_raw - d_f_prev)
correction = clamp(Kp * e + Ki * integral(e) + Kd * d_f, limit)
```

- 直线：`kappa=0`，中心速度初值 `350 mm/s`。
- 右半圆：`kappa=-1/500 mm^-1`，中心速度初值 `300 mm/s`。
- 弧线等效轮距初值 `160 mm`，所以理想弧线轮速约为左 `348 mm/s`、右
  `252 mm/s`。
- 直线巡线最大修正 `87.5 mm/s`；弧线最大修正 `20 mm/s`，避免红外外环把
  R500 几何前馈完全覆盖。
- 增速限制 `1500 mm/s^2`；减速和故障停车不等待斜坡。
- 巡线 D 项使用一阶低通，初值 `tau_d=60 ms`；20 ms 外环下 `alpha=0.25`。
  首次锁线和丢线重捕都会清零 D 状态，避免重新看到线时产生导数突跳。
- 连续看不到窄线时中心速度降为 `60 mm/s`，搜索距离达到约 `80 mm` 即锁存
  `CAR_FAULT_LINE_MISSED`。

末弧标志搜索窗口为：

```text
expected = pi * 500 - finish_sensor_to_test_point_mm
earliest = expected - required_line_search_mm
latest   = expected + required_line_search_mm
```

默认 `150 mm` 偏置和 `80 mm` 搜索量对应 `1340.8~1500.8 mm`。标志必须连续两帧；
若首帧恰好位于上边界，允许下一帧完成确认。确认后继续保持 R500 曲率前进
`150 mm`，使车上测试位置而不是前置红外阵列停到 A 点。`150 mm` 是待测机械量。

## 软件安全边界

- TB6612 的零目标会把 PWM、四个方向脚和 STBY 全部清零；即使非零 `mm/s` 换算后
  两轮都四舍五入成 `0 rpm`，也会立即停车，不保留上一拍 PWM。
- 轮端运动看门狗只接受与目标方向一致的编码器计数。达到超时边界后才来的脉冲无效；
  任一轮在目标大于等于 `10 rpm` 时连续 `600 ms` 无正确方向运动，就关闭整个 TB6612。
- 红外 ADC 每次只做一次软件触发转换，ISR 与 raw interrupt status 轮询均可完成采样；
  成功和超时路径都会停止转换并清中断状态。ICM42688 任一 SPI 字节超时都会让整帧
  无效并累计错误，不再把 `0xFF` 当作新鲜姿态。
- MCU WWDT 周期为 `4 s`，只在主循环完整执行到末尾后重启。主循环卡在 SPI、ADC、I2C
  或控制代码时不能喂狗；调试器暂停 CPU 时 WWDT 同步暂停。
- A 点备用宽线判据同时要求足够的白底背景，均匀灰面不会触发；VOFA 所有帧和回复都做
  写入边界检查，空间不足时丢弃整帧并增加 drop 计数。

这些是源码和构建层证据。WWDT 实际复位时间、编码器断线停车时间和 TB6612 引脚电平仍需
上板测量，不能由编译结果代替。

## 参数初值

| 参数 | 当前值 | 来源 | 作用 |
| --- | ---: | --- | --- |
| 轮径 | 65 mm | 实车记录，待复测 | 编码器计数换算距离和 RPM |
| 物理轮距 | 115 mm | 实车记录，待复测 | 编码器航向估计 |
| 每轮计数 | 724 count/rev | 实车记录，待复测 | RPM 与里程换算 |
| 弧线等效轮距 | 160 mm | 经验值 | R500 左右轮基础速度 |
| 速度环周期 | 50 ms | 代码配置 | 编码器速度更新周期 |
| 速度 PI | Kp 250 / Ki 600 milli | 经验初值 | 目标 RPM 跟踪 |
| PWM 上限 | 60% | 安全初值 | 限制电机输出 |
| 巡线 Kp | 0.025 mm/s/position | 经验初值 | 红外偏差转差速修正 |
| 巡线 Kd | 0.0005 mm/position | 经验初值 | 阻尼快速横向误差变化 |
| D 低通时间常数 | 60 ms | 计算初值 | 20 ms 外环下滤波系数 0.25 |
| 丢线搜索 | 80 mm | 安全初值 | 禁止无传感器盲跑 |
| 终点偏置 | 150 mm | 未验证 | 红外到车上测试位置的纵向距离 |

参数定义、调用链、物理含义和第一轮单变量实验见
[`docs/H2026_从驱动到整车控制.md`](docs/H2026_从驱动到整车控制.md)。

## 上电操作

1. 电机电源断开，仅给逻辑侧供电，确认无异常发热。
2. OLED P1 第一行显示当前 `RED/GRAY` 后端；第二行首先显示 `CAL:WHITE -> K2`。
3. 全部探头保持在白底同一高度，按右侧 KEY2；显示 `CAL:BLACK -> K2` 后换成黑面，
   再按 KEY2。两次采样均为 16 帧非阻塞累计。
4. OLED 显示 `CAL:OK K1 START`、`IMU:OK`、`ENC:1` 且故障 `F:00000000` 后，
   中间 KEY1 才会启动正式 B2。运行中再次按 KEY1 是锁存急停，不是可恢复暂停；复位后
   才能重新标定和启动。
5. 左侧 KEY3 只在 OLED P1/P2 间切换。P1 显示标定、IMU、编码器、路线段、线位置、
   PWM 和故障；P2 在未标定时显示 8 路 RAW，标定后显示 8 路 NRM，并显示线位置、
   巡线修正量、左右目标/实测 RPM。

OLED 主循环每 250 ms 重画一次内存缓冲区，每 5 ms 只发送一个 128 字节硬件页，
避免整屏 I2C 传输长期占用按键、串口和安全停机轮询。

## VOFA UART3

PB2/PB3，115200 8N1，FireWater 文本协议，100 ms 周期，固定 48 个数字字段。完整字段
表见教程。可用命令：

```text
STOP
PARAMS
SPDKP <0..10000>
SPDKI <0..10000>
SPDKD <0..10000>
SPDLIMIT <1..80>
LINEKP <0..0.5>
SEARCHMM <10..500>
HELP
```

除 `STOP`、`PARAMS`、`HELP` 外，命令必须在车已停车时执行。修改只保存在 RAM；
`SEARCHMM` 在下一次启动重新建路线后生效。

## 构建

在 PowerShell 中进入本目录：

```powershell
.\scripts\host_test.ps1
.\scripts\host_test.ps1 -ExtraCompilerArgs '-fanalyzer'
.\scripts\sysconfig_check.ps1
.\scripts\ccs_build.ps1
```

默认版本：CCS 21.0.0、TI Arm Clang 5.1.1 LTS、MSPM0 SDK 2.09.00.01、
SysConfig 1.28.0。产物为：

```text
Debug/empty_mspm0g3507_nortos_ticlang.out
Debug/empty_mspm0g3507_nortos_ticlang.map
```

当前正式 B2 构建的 `.out` SHA-256 为
`BF199F97F7DB70A3BDFC20C737F8C7B720A81FF90881ACE8ECD5929017FB39F1`；
Flash 使用 `39936/131072 Byte`，SRAM 使用 `5286/32768 Byte`（含 512 Byte 栈）。

按用户 2026-07-30 的永久边界，本仓库和后续 Codex 会话只做源码、测试和构建，
不得烧录、加载程序、启动调试、复位或继续运行目标板；板载固件版本由用户自行确认。

## 首次上板唯一顺序

架空车轮并限制电机电源，确认正命令下左右轮方向和编码器符号，再测试 KEY1/`STOP`
急停，最后分别断开左右编码器验证约 `600 ms` 内 TB6612 关闭。任一项失败都先修接线、
方向宏或编码器符号，禁止直接落地调 PID。
