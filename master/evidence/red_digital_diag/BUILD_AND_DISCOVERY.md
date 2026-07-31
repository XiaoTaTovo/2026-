# RED digital diagnostic build and discovery

## Source and SysConfig

- Target: MSPM0G3507, LQFP-64
- SysConfig source: `empty_mspm0g3507.syscfg`
- SysConfig tool: 1.28.0+4712
- Project metadata product: MSPM0 SDK 2.09.00.01
- Installed `product.json` version: 2.09.00.02
- SysConfig result: generated successfully; no pin or configuration errors
- Informational diagnostics: ADC auto-power-down wake time and STOP/STANDBY
  register-retention notices for PWM, SPI, and UART

Generated configuration evidence:

- PB17 / module D5: GPIOB input, PINCM43, no internal resistor
- PB18 / module D6: GPIOB input, PINCM44, no internal resistor
- PB19 / module D7: GPIOB input, PINCM45, no internal resistor
- ADC1 sequence contains only legacy PA18 and PA21 inputs
- PB17/PB18/PB19 are absent from ADC1 configuration

## Build

- Command: `gmake -C C:\Users\taowz\Desktop\2026\code\master\Debug clean all`
- Compiler/linker: TI Arm Clang 5.1.1 LTS
- Result: success, zero compiler errors and zero compiler/linker warnings
- FLASH used: 0x2698 = 9880 bytes of 131072 bytes
- SRAM used: 0x746 = 1862 bytes of 32768 bytes, including 512-byte stack
- Artifact: `Debug\empty_mspm0g3507_nortos_ticlang.out`
- Artifact size: 234604 bytes
- Artifact SHA-256: `73645171B3F40028F4E581867F2184057F8C9FFF3A645E09ABE857E202E6A110`
- SysConfig source SHA-256: `695EA3DDF6978C374D5AD17ADBF5B13FAF4CD78FA8EA428BFD79CE4221D3A065`
- Generated C SHA-256: `9CE90ED8F919BAD58F15BC20E8D44FF138CB3BAE600640105C142F497729215D`
- Generated H SHA-256: `396C1D8E18BAC0BB65C21B34E880C677B6DD04DB49490CE138B6EEF7905E1872`

The artifact contains the `#RED_DIGITAL_DIAG_V1` banner. The selected entry
keeps TB6612 stopped and STBY low, emits one CSV frame every 50 ms, and updates
the OLED heartbeat every 250 ms.

## Read-only discovery

Present devices:

- XDS110 Class Debug Probe: `USB\VID_0451&PID_BEF3&MI_02\6&B69B450&0&0002`
- XDS110 composite identity: `USB\VID_0451&PID_BEF3\NOSERIAL`
- XDS110 auxiliary data port: COM10
- XDS110 application/user UART: COM11
- Bluetooth SPP ports: COM7 and COM8
- Prior project evidence identifies COM7 as UART3 telemetry at 115200 8N1

Resource-ownership failure at discovery time:

- CCS/CCS server processes were running.
- DSLite was running.
- VOFA+ was running.
- The packaged probe detector timed out after 15 seconds.
- No target connection, reset, program, halt, run, breakpoint, memory read, or
  register write was attempted.

## Gate status

- `SOURCE`: PASS
- `SYSCONFIG`: PASS with the recorded SDK metadata mismatch warning
- `BUILD`: PASS
- `G4 identity discovery`: PASS
- `G4 exclusive ownership`: FAIL until CCS debug, DSLite, and VOFA+ release the
  probe and COM7
- `MANUAL_PROGRAM`: UNKNOWN
- `BOARD`: UNKNOWN
- `END_TO_END`: UNKNOWN
