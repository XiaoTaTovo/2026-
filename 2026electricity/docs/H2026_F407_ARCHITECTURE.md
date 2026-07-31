# H 题 STM32F407 控制工程架构与 bring-up 顺序

## 1. 当前证据

- MCU：STM32F407VET6，LQFP100。
- 工程：STM32CubeMX 6.17.0，STM32CubeF4 1.28.3，CMake + Arm GNU Toolchain。
- 时钟：当前生成代码使用 HSI 16 MHz，`16 / 8 * 168 / 2 = 168 MHz`。
- 原理图依据：用户提供的 H1/H2 引脚网络图。
- 电机协议依据：商家筛选后的 STM32F407 UART 例程，以及 X/Emm 固件协议代码；当前 PITCH 按 Emm 验证。
- 角度传感器机械基准：铰轴；传感器静止部分固定在机架，旋转部分跟随铰轴。

## 2. 外设所有权

| 功能 | MCU 外设 | 引脚 | 原理图网络 | 当前 `.ioc` |
|---|---|---|---|---|
| 蓝牙调参 | USART1 | PB6 TX / PB7 RX | BT_TX_STM / BT_RX_STM | 已配置，115200 8N1 |
| 主控通信预留 | USART2 | PA2 TX / PA3 RX | YAW_TX_STM / YAW_RX_STM | 已配置，115200 8N1，当前不使用 |
| PITCH EMM | USART3 | PB10 TX / PB11 RX | PITCH_TX_STM / PITCH_RX_STM | 已配置，115200 8N1 |
| MaixCAM | USART6 | PC6 TX / PC7 RX | Mai_TX_STM / Mai_RX_STM | 已配置，115200 8N1 |
| OLED | I2C1 | PB8 SCL / PB9 SDA | OLED_SCL_STM / OLED_SDA_STM | 已配置，100 kHz；当前 AS5600 台架测试暂时共用此总线 |
| 铰轴角度传感器 | I2C3 | PA8 SCL / PC9 SDA | I2C3_SCL / I2C3_SDA | 已配置，100 kHz；正式 PCB 路线，当前未接入 |
| 按键 1~4 | GPIO | PE11~PE14 | KEY1~KEY4 | 上拉输入 |

## 3. 备用 STEP/DIR/EN 网络

串口是正式主路线，下面六根线只保留为硬件兼容/救援接口：

| 轴 | STEP | DIR | EN |
|---|---|---|---|
| YAW | PA0 | PC1 | PC2 |
| PITCH | PA1 | PC3 | PC0 |

### FIX：当前 CubeMX 冲突

- 当前 `TIM3_CH1/PA6` 被命名为 STEP，但原理图的 STEP 是 PA0/PA1。
- 当前只把 PC2/PC3 配成 EN/DIR，且没有体现两个轴。
- 当前 TIM3 `Period=99`、`Pulse=500`，比较值大于周期，不能作为有效步进 PWM 配置。
- 由于正式路线使用 UART，本阶段禁止启动 TIM3 PWM，也不把这些 GPIO 接入应用层。

### STOP：上电前待确认

- 备用 STEP/DIR/EN 接口的有效电平和板载上下拉尚未从用户手册核对；当前不使用。
- 当前 PC2 上电被写为低电平；在确认 EN 有效电平前，不能把“低”解释为安全失能。
- 首次 UART 测试时，电机动力电源应断开，只给 MCU、蓝牙和调试接口供电。

## 4. 软件分层

```text
Core/                 CubeMX 生成层：时钟、GPIO、UART、I2C、TIM
Bsp/                  硬件适配层：有界 HAL 调用，不解析业务协议
Modules/              设备/协议层：OLED、X42S、角度传感器、蓝牙命令解析
App/                  H 题状态机、模式选择、控制器和故障策略
docs/                 引脚、协议、验收记录和参数来源
```

CubeMX 生成文件只在 `USER CODE` 区域内修改。新增源文件统一加入顶层 `CMakeLists.txt`，不写入可能被重新生成的 `cmake/stm32cubemx/CMakeLists.txt`。

## 5. 安全状态机

```text
BOOT
  -> SELF_TEST
  -> READY_READ_ONLY
  -> EMM_STATUS_QUERY_PASS
  -> EMM_READ_1000_PASS
  -> ARMED
  -> MANUAL_MOTION_LIMITED
  -> ANGLE_LOOP
  -> BALL_CONTROL

任意阶段 -> FAULT_LATCHED -> SAFE
```

- `READY_READ_ONLY` 之前不发送任何电机命令。
- 当前 EMM 只读固件每轮依次查询 `状态 -> 实时位置`，目标为完整 1000 轮；两类返回帧都校验地址 `0x01`、功能码、长度和校验字节，任一坏帧或超时立即失败闭锁。X42S/Y42 专用的广播读 ID 不用于 EMM。
- `COMM_PASS` 只证明 1000 轮通信完成，不会自动进入自动控制；当前工程仅开放受限的手动定时速度测试：10 RPM、300 ms、自动 `0xFE` 停止，视觉 PID 仍无电机命令出口。
- 蓝牙只能修改白名单参数，不能绕过状态机直接使能电机。
- OLED 只显示状态，不拥有控制权。
- MaixCAM 数据必须带有效性和超时；视觉失联不得保持旧球位继续控制。

## 6. 逐项验收顺序

1. 当前工程 clean build。
2. USART1 有线 USB-TTL 回环，随后再接蓝牙模块。
3. OLED 单独点亮、清屏、固定字符；确认控制器型号和 I2C 地址后再写设备初始化序列。
4. 铰轴角度传感器：先运行 `PitchAxisAngleSelfTest`，探测 AS5600 `0x36` 并以 20 ms 周期读取 500 次；当前台架走 I2C1 `PB8=SCL`/`PB9=SDA`，正式 PCB 改回 I2C3 `PA8=SCL`/`PC9=SDA`。只验收身份、磁铁状态、原始计数和静态噪声，不自动置零、不定义正方向、不进入控制。
5. USART2 当前仅保留为主控通信预留，不接入本次 PITCH 电机测试。
6. USART3 PITCH 的 1000 轮 `状态 -> 实时位置` 只读自检已完成实物验收；当前固件在自检通过后开放受限的 KEY1~KEY4 定时速度测试，具体操作见 `docs/pitch_velocity_mode_test.md`。自动控制仍未开放。
7. MaixCAM 串口离线帧解析：`ball_observation_protocol` 已冻结图像端的 20 Byte v2 帧、CRC16-Modbus 和 `valid/reason/x_0.1mm/confidence/capture_age_ms` 语义；本阶段只提供字节解析与故障计数，不接控制输出。USART6 DMA 接入、视觉年龄门控和控制周期待相机链路实测后再单独验收。
8. 人工确认机械限位、方向、电流和急停后，开放单轴低能量运动。
9. 单轴角度闭环通过后，再进入球位置外环。

### 1000 轮只读验收口径

- EMM 不使用手册第 92 页仅适用于 `X42S/Y42` 的 `00 15 6B` 广播读 ID；每轮固定顺序为 `按已确认地址读状态 -> 按已确认地址读实时位置`，完整轮数必须为 1000。
- 状态帧和位置帧各校验一次地址回显，因此 1000 轮必须得到 `PITCH_ADDRESS_ECHO_OK=2000`；请求错误、协议错误、任一阶段超时都必须为 0。
- 记录状态首值、末值和变化次数；状态位语义和安全许可在开放运动前单独验收，不能只凭 `COMM_PASS` 推断。
- 记录电机位置首值、最小值、最大值、末值和相邻帧最大跳变；本轮不预设机械噪声阈值，先由静止实测建立基线。
- 最终日志必须同时出现 `PITCH_READ1000_RESULT=COMM_PASS` 和 `PITCH_MOTION_GATE=LOCKED`。

### 铰轴角度静态验收口径

- 该步骤使用 `PitchAxisAngleSelfTest`，独立拥有 AS5600 读取状态机；它不读取 EMM 位置作为管道角度，也不向 EMM 写任何命令。
- 固定硬件假设为 AS5600、7-bit I2C 地址 `0x36`、3.3 V 逻辑和外部上拉。当前台架接线为 I2C1 `PB8=SCL`/`PB9=SDA`；正式 PCB 接线为 I2C3 `PA8=SCL`/`PC9=SDA`。应用层只通过 `App/Inc/pitch_axis_board_config.h` 的 `PITCH_AXIS_ANGLE_I2C_HANDLE` 选择其一，避免修改驱动或自检状态机。实际模块型号、磁铁同轴度、距离和供电仍须实物回填。
- 启动后先探测设备，再每 20 ms 读取一次，目标为 500 个有效样本。正常结束应输出 `PITCH_ANGLE_RESULT=PASS`、`PITCH_ANGLE_VALID=500`、`PITCH_ANGLE_I2C_ERRORS=0`、`PITCH_ANGLE_MAGNET_ERRORS=0` 和 `PITCH_ANGLE_CONTROL=DISABLED`。
- `I2C_NACK` 表示器件未应答或接线/供电错误；`MAGNET_MISSING`、`MAGNET_WEAK`、`MAGNET_STRONG` 表示 AS5600 状态寄存器认为磁场不合格。任一失败都不会尝试重试、置零、使能或运动。
- 当前 5 ms I2C HAL 超时只用于低频冒烟验收。进入 100 Hz 以上的角度内环前，必须实测事务耗时和主循环抖动，再决定 I2C DMA/中断改造与最终采样周期。

### H 题联动数据边界

| 数据 | 来源 | 物理含义 | 后续用途 | 失效动作 |
|---|---|---|---|---|
| 球位置 | MaixCAM / USART6 | 小球沿管道的一维位置 | 球位置外环生成目标管道角度 | 超时后目标角度回到安全值，不沿用旧球位 |
| 铰轴角度 | 铰轴角度传感器 / 当前 I2C1，正式 I2C3 | 管道相对车架的真实角度 | 管道角度反馈和机械限位判断 | 无效时禁止运动并闭锁 |
| EMM 实时位置 | PITCH EMM / USART3 | 电机编码器侧原始位置 | 通信健康、方向、传动和堵转诊断 | 超时或跳变时禁止继续下发运动 |
| 小车状态 | 主控预留 / USART2 | 启动边沿、运行阶段和可选加速度信息 | 任务同步、前馈和失联降级 | 失联时保持球控安全状态，不猜测小车阶段 |

在传动比、零位和机械同轴性实测前，禁止把 EMM 原始位置直接当作管道角度。后续联动状态必须至少携带数值、单位、采样时间、有效位和超时判定。

## 7. UART DMA 传输契约

USART1 蓝牙首先验证，USART3 的 PITCH EMM 和 USART6 的 MaixCAM 复用同一传输层；USART2 保留给主控通信。

| 项目 | 当前值 | 说明 |
|---|---:|---|
| RX 硬件 DMA 缓冲 | 128 Byte | DMA Circular，放在普通 SRAM，不放 CCMRAM |
| RX 软件环形缓冲 | 512 Byte | ISR 生产、主循环消费，满时丢弃新字节并计数 |
| TX 软件环形缓冲 | 512 Byte | 主循环入队、DMA 完成中断消费 |
| TX DMA | Normal | 每次发送环形缓冲中的一段连续区域 |
| RX 事件 | HT / TC / IDLE | 全部使用 DMA 写位置差量处理，重复位置不重复入队 |

关键规则：

- `HAL_UARTEx_RxEventCallback(..., Size)` 中的 `Size` 是 DMA 当前写入位置，不是本次事件长度。
- TC 时允许 `Size == DMA_BUFFER_SIZE`；不能先对它取模再判断重复，否则可能丢掉整块数据。
- TC 与 IDLE 连续报告同一位置时，第二个事件必须忽略。
- ISR 只搬运字节和更新计数，不解析蓝牙命令或 X42S 协议。
- TX DMA 未完成时禁止再次直接调用 `HAL_UART_Transmit_DMA`；新数据只能进入 TX 环形队列。
- RX 软件环溢出、TX 队列满、UART 错误和 DMA 启动失败都有独立累计计数。

USART1 DMA 冒烟验收：

1. 上电收到一次 `BT_DMA_READY`。
2. 发送 1、63、64、65、127、128、129、511 Byte 已知序列并逐字节回显一致。
3. 连续发送超过 128 Byte 且中间无空闲，验证 HT/TC 跨环不重不漏。
4. 每轮记录 `rx_bytes/tx_bytes`、HT/TC/IDLE、duplicate、overflow、uart_error。
5. 冷启动与蓝牙断开重连各重复 10 次；任何 overflow/error 非零都不进入调参协议。

## 8. 当前未冻结信息

- 蓝牙模块准确型号、供电和逻辑电平。
- OLED 控制器型号（SSD1306/SH1106 等）、分辨率和 7 位 I2C 地址。
- 铰轴角度传感器准确型号、安装同轴度和允许机械角度。
- EMM/X 当前固件类型、设备 ID、TTL/RS485 电气接口、EN 有效电平。
- USART2 主控通信和单个 PITCH 轴在 H 题机械结构中的最终职责。
