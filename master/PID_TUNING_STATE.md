# PID 调参持续工作状态

最后更新：2026-07-30

## 恢复入口

每次继续调参时依次读取：

1. `PID_TUNING_WORKDOC.md`：长期职责边界、协议目标、数据闸门和每轮格式。
2. 本文件：当前源码事实、目标环、遥测映射和待用户提供项。
3. `tuning_log.jsonl`：已有有效试跑记录；当前尚未开始试跑，因此文件尚未创建。

## 当前目标

- MCU：TI MSPM0G3507。
- 固件目录：`master/`。
- 准备调试：内速度环，以及巡线相关的直线段和弧线/转向段。
- 调参顺序：先验证并调好内速度环，再动巡线外环。
- 当前源码没有独立的“转向 PID”：直线和弧线共用同一个 `line_kp/line_ki/line_kd`，区别是基础曲率/速度和修正限幅。

## 源码身份

- 仓库提交：`608ccd3`。
- `vofa_telemetry.c` SHA-256：`C56194B33935076726516BD950550044E3D696AA068A0647255E583636CA42C1`。
- 工作树已有用户未提交改动；不能用提交号代表车上固件。
- 必须由用户确认实际烧录版本，调参时以车上 `PARAMS` 和串口 banner 为准。

## 当前控制事实

### 内速度环

- 更新周期：`50 ms`。
- 当前编译初值：`Kp=250 milli`、`Ki=600 milli`、`Kd=0`、输出限幅 `60%`。
- 结构：目标 RPM 与实测 RPM 做误差；含静摩擦/转速前馈、PID、积分抗饱和和输出限幅。
- 运行期可写：`SPDKP`、`SPDKI`、`SPDKD`、`SPDLIMIT`，仅停车时生效。

### 巡线外环

- 控制任务周期：`20 ms`。
- 当前编译初值：`Kp=0.025`、`Ki=0`、`Kd=0.0005`、D 低通 `tau=60 ms`。
- PID 输入：`track_position`；输出：左右轮差速修正 `line_correction_mm_s`。
- 直线：中心速度 `350 mm/s`，修正限幅 `87.5 mm/s`。
- 弧线/转向：中心速度 `300 mm/s`，等效轮距 `160 mm`，修正限幅 `20 mm/s`。
- 运行期当前只可写 `LINEKP`；`LINEKI`、`LINEKD`、D 低通、直线/弧线限幅仍是编译期常量。

## 当前命令表

当前固件仅支持：

- `STOP`
- `PARAMS`
- `HELP`
- `SPDKP <0..10000>`
- `SPDKI <0..10000>`
- `SPDKD <0..10000>`
- `SPDLIMIT <1..80>`
- `LINEKP <0.0..0.5>`
- `SEARCHMM <10..500>`

长期工作文档要求的 `LIST/GET/SET/LOGMS/LOGSET/LOGMODE/LOGDEC/LOGDUR`、ARM/DONE 事件、`trial_id` 和高速缓冲尚未实现。

## 当前遥测契约

- 协议：VOFA+ FireWater 文本 CSV。
- 串口：UART3 / `UART_VOFA_INST`，`115200 8N1`。
- banner：`#H2026_TB6612_FIREWATER_V1 fields=48 period_ms=100 baud=115200`。
- 当前周期：`100 ms`，48 个数字字段。
- 映射版本：以上 `vofa_telemetry.c` SHA-256；源码变化后必须重新生成，禁止沿用。

| 索引 | 含义 | 缩放/单位 |
| ---: | --- | --- |
| I0 | uptime | ms |
| I1 | mode | enum |
| I2 | executor.running | 0/1 |
| I3 | executor.finished | 0/1 |
| I4 | segment index | integer |
| I5 | track phase | enum |
| I6 | segment progress | ×10 mm |
| I7 | center distance | ×10 mm |
| I8 | line valid | 0/1 |
| I9 | track position | estimator position |
| I10 | track confidence | integer |
| I11 | track active count | channels |
| I12 | track active mask | bitmask |
| I13 | line pattern | enum |
| I14 | line correction | ×10 mm/s |
| I15 | left wheel command | ×10 mm/s |
| I16 | right wheel command | ×10 mm/s |
| I17 | left target RPM | rpm |
| I18 | right target RPM | rpm |
| I19 | left measured RPM | rpm |
| I20 | right measured RPM | rpm |
| I21 | left output | percent |
| I22 | right output | percent |
| I23 | left encoder delta | count |
| I24 | right encoder delta | count |
| I25 | speed sample elapsed | ms |
| I26 | speed update count | integer |
| I27 | yaw | ×10 degree |
| I28 | combined faults | bitmask |
| I29 | last exit reason | enum |
| I30 | line gap distance | ×10 mm |
| I31 | marker streak | frames |
| I32 | last marker confidence | integer |
| I33 | last marker active count | channels |
| I34 | finish offset progress | ×10 mm |
| I35 | result time | ms |
| I36 | result valid | 0/1 |
| I37 | encoder valid | 0/1 |
| I38 | drive active | 0/1 |
| I39-I46 | 8 路归一化灰度 | 0..1000 |
| I47 | TX drop count | integer |

## 开工阻塞项

在有效调参前，需要用户提供或确认：

1. 是否授权先在允许区 `vofa_telemetry.c/.h` 实现参数注册表、ARM/DONE 事件和高速日志缓冲。
2. 车上当前实际烧录的固件身份；至少提供启动 banner 和 `PARAMS` 原始回复。
3. 架空轮安全检查是否通过：左右轮方向、左右编码器符号、KEY1/`STOP` 急停、两路编码器断线约 600 ms 停车。
4. 第一阶段先调速度环时，给出目标运行点或典型目标 RPM、期望上升时间/稳态误差/超调，以及可接受的最大 PWM 和机械抖动。
5. 人工试跑或架空轮实验由用户触发；Codex 只给命令和步骤，并分析用户导入的完整串口/VOFA 数据。

## 当前判断

- `100 ms` 固定遥测不能可靠观察 `50 ms` 速度环和 `20 ms` 巡线环的动态细节，只能做低频趋势和安全检查。
- 若要按长期工作文档进行可证伪的 run-to-run 调参，应先完成允许区内的遥测协议升级，再开始正式记录试跑。

## 2026-07-30 速度环预检

- 当前活动的 `empty_mspm0g3507.c` 在 `#if 1` 诊断分支中运行独立的红外/电机测试程序；`main()` 只调用 `TB6612_Init()`、`TB6612_SetMotors()`、`handle_serial()` 和两秒自动停车。
- 该分支的 `w/s/a/d/x` 字符会直接给电机百分比，不会初始化 `CarFirmware`、不会运行 `TB6612Drive` 的 50 ms 速度闭环，也不会调用 `VofaTelemetry_*`。
- 当前 `Debug/empty_mspm0g3507_nortos_ticlang.out` 为 312812 Byte，SHA-256 为 `A68DE6B015020F7CB5712E8889497108D0A9B8231702E522A5A15183034C3F28`；它不是可用于内速度环调参的已确认镜像。
- `Debug/dslite_flash_red_oled_20260730.log` 记录 XDS110 连接失败（Error -260），不构成任何成功烧录证据。
- 结论：在保留或替换当前诊断入口的决策完成前，不得把蓝牙数据或架空轮现象当作内速度闭环数据，也不得改 PID 参数。

## 2026-07-30 速度环专用镜像

- 用户确认保留既有开环诊断代码，并授权新增独立速度环调试入口；`speed_tuning.h` 的 `H2026_SPEED_TUNING_BUILD=1` 选择新入口。将该值改为 `0` 可恢复原诊断分支，不删除原代码。
- `speed_tuning.c` 初始化既有 `TiMspm0Platform` 和 `TB6612Drive`，不依赖 IMU、灰度标定或路线状态；KEY1 触发双轮 `350 mm/s` 阶跃，自动限时 `5 s`，再次按 KEY1 提前停止。
- UART3/蓝牙仍使用既有运行期调参命令：`PARAMS`、`SPDKP`、`SPDKI`、`SPDKD`、`SPDLIMIT`。`STOP` 仍为故障锁存急停，不可作为常规轮次前置命令。
- 新协议 banner：`#H2026_SPEED_TUNING_FIREWATER_V1 fields=9 period_ms=25 baud=115200`。
- 数字帧列：`t_ms,target_l_rpm,measured_l_rpm,output_l_pct,target_r_rpm,measured_r_rpm,output_r_pct,faults,tx_drop`；ARM/DONE 为 `#EVT` 文本行。
- 带宽预算：9 字段最坏约 118 Byte/frame，`25 ms` 为 40 Hz，约 4.72 kB/s，占 11520 Byte/s 的 41%；低于 50% 余量线。该采样周期满足速度环 `50 ms` 更新周期的一半，能可靠辨识低于约 `8 Hz` 的动态。
- SOURCE/HOST：通过。CCS clean/full BUILD：0 error、0 warning。镜像：`Debug/empty_mspm0g3507_nortos_ticlang.out`，386156 Byte，SHA-256 `0F1DBB182F36A43B54A8E1810B8914C467340747E3FDB45FBB57F14232F4A06D`。
- MANUAL_PROGRAM/BOARD/END_TO_END：未验证；必须由用户手动烧录、选择蓝牙 SPP 端口、架空轮试验并导入完整原始日志。

## 2026-07-30 COM7 只读观察

- 已在 `COM7` 以 115200 8N1、RTS/DTR 均关闭的方式被动监听约 30 s；收到 9 字段、严格 25 ms 间隔的速度环专用帧，证明该端口和新遥测协议实际连通。
- 观察窗口内所有 `target_l_rpm/measured_l_rpm/output_l_pct/target_r_rpm/measured_r_rpm/output_r_pct` 均为 `0`，没有 `#EVT ARM` 或 `#EVT DONE`，因此不是有效试跑，严禁据此调整 PID。
- 窗口开始收到两条 `#ERR UNKNOWN_COMMAND`，说明另一个客户端曾向 COM7 写入非当前协议命令。后续试跑前关闭所有其他 VOFA/串口终端，确保 COM7 只有一个观察者。

## 2026-07-31 速度环架空轮调参完成

- 已确认专用镜像通过 COM7 输出 9 字段 H2026_SPEED_TUNING_FIREWATER_V1 协议，并支持有界 RUN / IDLE。
- 共完成 6 个完整 ARM 到 DONE 的 350 mm/s、5 s 双轮阶跃；每轮故障位与 TX 丢弃均为 0，完整记录见 tuning_log.jsonl。
- 当前已写入 RAM 且选定的参数：SPDKP=180、SPDKI=200、SPDKD=0、SPDLIMIT=60；前馈保持 static=6000、rpm=225、死区 2 rpm。
- 确认轮：左右峰值均为 115 rpm（目标 103 rpm，约 11.7% 超调），5% 收敛约 400 ms，末段均值左 104.1 rpm、右 104.65 rpm，无故障、无丢帧、无饱和。
- 编译期默认 H2026_SPEED_KP_MILLI/H2026_SPEED_KI_MILLI 已更新为 180/200；重新构建并手动烧录后，参数在复位后保持。
