# 2026 全国大学生电子设计竞赛 H 题工程

本仓库集中保存 2026 H 题的底盘、摆杆控制器、球位视觉和独立图传代码。各控制器保持
物理与软件解耦，通过明确协议组合；“可以编译”不等于已经通过实车或整机验收。

## 目录说明

| 目录 | 平台 | 用途 | 当前边界 |
| --- | --- | --- | --- |
| `master/` | TI MSPM0G3507 + CCS | 小车主控、TB6612、编码器、红外巡线和 B2 基线 | 正式主开发工程，详细状态见目录内 README |
| `final/` | TI MSPM0G3507 + CCS | 从主工程独立出的六任务赛用快照 | 与 `master` 分开维护，烧录前必须重新执行测试与实车验收 |
| `2026electricity/` | STM32F407VET6 + CubeMX/HAL/CMake | 天空星摆杆控制器基础工程 | 当前是外设与构建基线，电机运动能力不得仅凭编译结果开放 |
| `vision/maxicam/` | MaixCAM Pro / MaixPy | 钢球检测、杆轴标定和定长 UART 球位帧 | 模型、镜头、曝光、电平与实机标定仍需现场确认 |
| `vision/picture/` | ESP32-CAM + Raspberry Pi | 独立图传、显示、录像与回放 | 与控制视觉物理分离，现场需做干扰和断线恢复测试 |

## 控制架构

```text
MSPM0 小车控制器
  -> 红外巡线 / 编码器速度环
  -> TB6612 双轮底盘

STM32F407 摆杆控制器
  <- MaixCAM 球位
  <- 摆杆角度传感器
  -> X42S 电机

ESP32-CAM -> Wi-Fi -> Raspberry Pi 显示与录像
```

小车执行链只使用 `TB6612 + 双轮编码器速度环`。STM32F407 与 X42S 属于独立摆杆
控制链，不得把串口电机协议重新混入小车主控。

## 构建与测试

### MSPM0 主工程

```powershell
cd master
.\scripts\host_test.ps1
.\scripts\sysconfig_check.ps1
.\scripts\ccs_build.ps1
```

完整接线、安全门、参数和上板顺序见 [`master/README.md`](master/README.md)。六任务快照
的入口与限制见 [`final/README.md`](final/README.md)。

### STM32F407 工程

需要 Arm GNU Toolchain、CMake 和 Ninja：

```powershell
cd 2026electricity
cmake --preset Debug
cmake --build --preset Debug
```

CubeMX 的唯一配置源是 `2026electricity/2026electricity.ioc`。重新生成代码前应检查
`USER CODE` 区、时钟、引脚和生成差异，不提交 `build/` 中的本机构建缓存。

### 视觉代码基础检查

```powershell
python -m compileall -q vision
```

这一步只验证 Python 文件能够被解析，不替代 MaixCAM 模型加载、摄像头采集、UART 电平、
球位标定和图传实机测试。

图传的安装、网络和验收步骤见 [`vision/picture/README.md`](vision/picture/README.md)。

## 硬件安全与验收

- 未完成对应 README 中的 `PROGRAM / BOARD / END_TO_END` 验收前，不宣称任务通过。
- 电机首次上电必须架空或脱开负载，准备独立断电开关，并从只读状态查询开始。
- X42S 的型号、协议版本、逻辑电平、ID、波特率、校验和使能有效电平必须以实物为准。
- 摄像头、蓝牙和电机串口必须共地并核对 TX/RX 方向；禁止把 5 V UART 直接接入 3.3 V MCU。
- 仓库不保存 Wi-Fi 私密口令、个人密钥、录像、数据集和本机编译缓存；示例配置只提供占位值。

## 分支与提交

`main` 保存当前可复现基线。提交前至少运行与改动目录对应的主机测试或真实工具链构建，
并在提交信息中注明尚未完成的板级验证，不用编译成功替代硬件证据。
