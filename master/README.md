# 2026 H 题主控 CCS 工程

这是由 2024-H 实车工程迁移出的 MSPM0G3507 主控。当前唯一可烧录的正式题模式是
要求 2（B2）：小车从 A 点出发，沿 1.5 m 直线、R500 右半圆、1.5 m 直线、
R500 右半圆顺时针行驶一圈，识别 A 点横向启停线后停车并保留总时间。

## 当前状态

| 层级 | 状态 | 结论 |
| --- | --- | --- |
| SOURCE | 已完成 | 默认 `PROJECT_MODE_H2026_B2`；旧 2024-H 与历史预测路线保留 |
| HOST | 已完成 | 严格 C11 路线、横线、丢线、时间回绕与故障测试 |
| SYSCONFIG | 已完成 | SysConfig 1.28 严格生成通过，0 warnings / 0 errors |
| BUILD | 已完成 | CCS 21.0.0 + TI Arm Clang 5.1.1，0 warnings / 0 errors |
| PROGRAM | 未执行 | 本次没有连接、复位或烧录开发板 |
| BOARD | 待执行 | 必须先架空车轮完成方向、急停和传感器检查 |
| END_TO_END | 待执行 | 尚未证明 B2 的 20 s 和停车偏差不超过 2 cm |

`B3`、`B4`、`B5`、`B6` 分别有独立编译门，且路线构建器会拒绝所有占位实现。
MaixCAM、步进电机、摆杆角度反馈、限位和滚球控制尚未接入；不能通过改一个总开关
把未完成模式烧进去。

## 正式 B2 控制结构

每段使用中心速度、固定曲率前馈和红外横向误差反馈：

```text
v_left  = v * (1 - kappa * b_eff / 2) + correction
v_right = v * (1 + kappa * b_eff / 2) - correction
```

- 直线 `kappa=0`，中心速度初值 `350 mm/s`。
- 右半圆 `kappa=-0.002 1/mm`，中心速度初值 `300 mm/s`。
- 等效转向轮距初值 `160 mm`，来自旧车实测，不等同尺量轮距。
- 轮目标变化率限制为 `1500 mm/s^2`。
- TB6612 速度闭环对左右编码器分别做运动看门狗：目标绝对值达到 `10 rpm` 后布防，
  任一车轮连续 `600 ms` 没有编码器计数变化时，驱动层先立即停机，再由现有
  `CAR_FAULT_ENCODER_STALE` 路径锁存故障。阈值为架空轮首测值，必须验证起步静摩擦、
  低速转弯与拔掉单路编码器三种工况后再冻结。
- 连续轨迹暂时看不到窄线时降到 `60 mm/s`；累计前进 `80 mm` 仍未恢复即锁存
  `CAR_FAULT_LINE_MISSED` 并停车，不允许按里程盲跑整圈。
- 最终 A 横线窗口以 `末弧长度 - 传感器到测试点偏置` 为名义触发位置，向前、向后
  各开放 `required_line_search_mm`；下界换算为末弧 arm ratio，上界仍未识别则故障。
  标志要求连续 2 帧、宽面或连续宽足迹，并且此前必须锁定过主线；若第一帧恰在
  上界出现，仍保留一个控制周期完成第二帧确认。`SEARCHMM` 只允许停车时修改，
  并在下一次启动重建路线窗口，禁止行驶中出现上下界参数版本不一致。
- 横线触发后按“红外阵列到车上唯一测试位置”的纵向距离继续前进；当前初值
  `150 mm`，接近速度 `180 mm/s`。必须按本车永久标记重新测量。
- 该补偿段保持最后一个 R500 右弧的曲率，不在传感器刚碰到横线时提前切成直线：
  前置阵列会先扫到 5 cm 横线外侧，此时后方测试位置仍未到圆弧终点。以
  `R=500 mm`、前置约 `150 mm` 估算，提前走切线会产生约 `22 mm` 径向偏差。
  `150 mm` 只作首轮值，最终必须用实际停车偏差单变量标定。

按理想目标速度估算，主路线约 `19.04 s`；若横线在测试点前约 150 mm 触发，
最后 150 mm 从 300 降到 180 mm/s，再计入起步斜率，理论约 `19.5 s`。
这只是计算值，不是成绩证据。若实车横线触发位置、速度闭环或偏置关系不同，必须以
日志和秒表重新计算。

## 引脚保持

没有修改 `empty_mspm0g3507.syscfg` 的任何引脚、上下拉、时钟或外设实例。

| 功能 | 既有引脚 |
| --- | --- |
| TB6612-1 左/右 PWM | PB12 / PB4，TIMA0，20 kHz |
| 左/右方向与 STBY | PA14/PA15，PA16/PA17，PA28 |
| 左编码器 A/B | PA7 / PA22 |
| 右编码器 A/B | PA30 / PA31 |
| 8 路红外地址、ADC、EN、ERR | PA24/PA25/PA26，PA27，PB24，PB25 |
| ICM42688 SPI/CS | PB9/PB8/PB7，PB0 |
| OLED I2C0 | PA0 / PA1，400 kHz |
| KEY1 / KEY2 / KEY3 | PB23 / PB26 / PB27，低有效、内部上拉 |
| 蜂鸣器 | PB5 |
| 路线遥测与调参 UART3 | PB2 / PB3，115200 8N1 |
| 板载 CH340 调试 UART0 | PA10 / PA11，115200 8N1 |
| 保留电机 UART1 | PA8 / PA9，115200 8N1 |

完整资源表见 [`docs/MSPM0G3507_拓展板_引脚汇总.md`](docs/MSPM0G3507_拓展板_引脚汇总.md)。
如果新购器件实际是单点 ToF/红外测距模块，而不是 8 路地面反射强度阵列，本工程的
巡线采集层不能直接复用；在拿到准确型号、接口、数量、供电和安装几何前不要改
SysConfig 猜引脚。

## 按键与上电操作

1. 上电后保持小车静止，等待 OLED 显示 `IMU:OK`。
2. 将全部红外探头置于白底，按一次 KEY3；OLED 显示 `GC:CAPW` 并采 16 帧。
3. OLED 进入 `GC:BLACK` 后，将全部探头置于黑线，第二次按 KEY3；成功后显示
   `GC:OK`。每次上电都要完成白/黑标定。
4. KEY1 在停车状态启动 B2；运行中再次按 KEY1 是紧急停车并锁存故障。
5. KEY2 当前不切换模式，防止现场误触进入未完成任务。

OLED 第一行显示 `B2`、`RUN/STOP/DONE` 和运行或结果时间。故障、急停和漏线后，
运行时间保持在故障发生时刻，不再清零。PB2/PB3 的 FireWater 遥测包含模式、运行
时间、结果时间、曲率、中心速度参考和连续丢线距离；当前启动横幅为
`H2026 OFFICIAL B2 DEFAULT V53`。

## 可复现验证

在 PowerShell 中进入本目录后运行：

```powershell
.\scripts\host_test.ps1
.\scripts\sysconfig_check.ps1
.\scripts\ccs_build.ps1
```

默认工具路径：

- CCS CLI：`D:\software\Code\CCS_21.0.0\ccs\eclipse\ccs-server-cli.bat`
- SysConfig：`D:\software\Code\CCS_21.0.0\ccs\utils\sysconfig_1.28.0`
- MSPM0 SDK：`D:\software\Code\TI\SDK\mspm0_sdk_2_09_00_01`
- 编译器：`TICLANG_5.1.1.LTS`

如果工具安装位置不同，使用脚本的 `-CcsCli`、`-SysConfigCli` 和 `-Product`
参数覆盖默认值，不需要改工程元数据。

脚本先单独执行 CCS `clean`，再执行 `full`；`clean` 本身不会编译。成功产物为：

```text
Debug/empty_mspm0g3507_nortos_ticlang.out
Debug/empty_mspm0g3507_nortos_ticlang.map
```

`.out` 和整个 `Debug/` 是可再生构建产物，已由 Git 忽略。烧录前必须重新核对
MSPM0G3507、LQFP-64、XDS110、板卡供电和电机悬空状态；本仓库不提供自动烧录
脚本，也不执行 mass erase、unlock 或自动复位。

2026-07-29 V53 当前 clean/full 构建证据：

```text
OUT bytes:   737164
OUT SHA-256: 70834B2C18A72A068C42BBCB37B9D715AC2CE677BB1EA8B563D058F110F18DC1
FLASH:       78992 / 131072 bytes (60.3%)
SRAM:         7179 /  32768 bytes (21.9%, includes 512-byte stack section)
```

## 首次上板顺序

1. 电机 VM 断开，只给主控和逻辑侧供电，确认无异常发热和 OLED 状态。
2. 经人工确认后烧录本次 `.out`，完成 IMU 静止校准和白/黑标定。
3. 架空车轮后按 KEY1，确认左右轮前进方向、编码器符号和再次按 KEY1 的急停。
4. 低速、短距离落地确认“线在右侧时左轮加速”，再测直线和右半圆。
5. 最后测量红外阵列中心到唯一测试位置的纵向距离，更新
   `H2026_FINISH_SENSOR_TO_TEST_POINT_MM`，每个值至少重复 10 次。
6. 保存 PB2/PB3 遥测、秒表、停车偏差和电池电压。连续 10 次同时满足
   `<=20 s`、`<=2 cm` 后，才能把该构建标为 B2 实测通过。
