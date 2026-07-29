# 本次上传改动说明

> 2026-07-23 实物 PCB 校正：KEY1=PB23、KEY2=PB26、KEY3=PB27。
> OLED 已验证中键、右键、左键按下分别显示 `KEYH:011`、`101`、`110`。

1. 新增灰度 ADC 调试模式 `PROJECT_MODE_GRAY_ADC_DEBUG`，并设为当前默认模式。
2. 新增 SSD1306/SH1106 OLED 驱动，OLED 使用 I2C0：PA0=SDA、PA1=SCL。
3. 在 OLED 上实时显示 8 路灰度传感器的 ADC 原始值。
4. 在 OLED 上显示灰度模块 EN、ERR 引脚电平、ADC 读取状态和递增帧号。
5. 增加灰度模块引脚配置：AD0=PA24、AD1=PA25、AD2=PA26、OUT=PA27、EN=PB24、ERR=PB25。
6. 将 ADC 配置为重复单通道模式；每路切换后等待 10 us，并采样 4 次取平均。
7. 修复平台层灰度 GPIO 端口宏名称错误。
8. 将工程使用的 TI Arm Clang 版本调整为本机可用的 4.0.4 LTS。
9. 新增 `PROJECT_MEMORY.md`，用于持续记录项目状态和调试结果。
10. 使用 KEY1/KEY2 按键完成白底和黑线校准：KEY1=PB23（中键）采集白底，KEY2=PB26（右键）采集黑线，各采样 16 帧取平均；KEY3=PB27（左键）用于失败重试。
11. 校准完成后在 OLED 显示 8 路 `0～1000` 归一化值，并按阈值显示每路是黑色 `B` 还是白色 `W`。
12. 在 `gray_array.c` 中新增 `GrayArray_ClassifyLatest()`，统一输出 8 路黑、白和不确定分类掩码。
13. 将灰度 ADC 调试状态机、按键处理和 OLED 显示逻辑抽到 `app/gray_adc_debug.c/.h`，简化主入口文件。
14. 同步远程 `main` 的速度环调试改进：独立 KP/KI/KD、前馈、速度滤波、误差死区、抗积分饱和和可调失败保护。
15. 保留灰度调试模式和当前灰度、OLED、按键引脚配置，更新蓝牙调试入口以使用 main 的完整速度环流程。
