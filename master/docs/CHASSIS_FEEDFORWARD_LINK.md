# Chassis Feedforward Link

This document describes the current measurement-only link. It does not enable
the F407 pitch motor.

## Roles

| Board | Interface | Responsibility |
|---|---|---|
| MSPM0 master | UART0, PA10/PA11, 115200 8N1 | Read ICM42688, calibrate acceleration bias, derive command acceleration, enqueue state frames |
| STM32F407 | USART2, PA2/PA3, 115200 8N1 | Circular-DMA receive, frame validation, freshness check, estimate/log only |
| STM32F407 | USART3, PB10/PB11 | Existing X42S link; unchanged and locked out of feedforward |
| STM32F407 | USART1, PB6/PB7 | Existing Bluetooth telemetry; unchanged |

Connect `master PA10 (TX) -> F407 PA3 (USART2 RX)`,
`master PA11 (RX) <- F407 PA2 (USART2 TX)`, and a common ground. Both sides
must use 3.3 V TTL. Do not connect this link to RS-232 levels or to the X42S
USART3 pins.

## Frame

Each frame is 22 bytes, little-endian, beginning with `A5 5A`:

| Offset | Size | Field | Unit |
|---:|---:|---|---|
| 0 | 2 | SOF | fixed `A5 5A` |
| 2 | 1 | protocol version | `1` |
| 3 | 1 | message type | `0x31` state |
| 4 | 2 | sequence | modulo 65536 |
| 6 | 4 | source timestamp | master milliseconds |
| 10 | 2 | flags | validity, route, fault |
| 12 | 2 | center speed | `0.1 mm/s`, signed |
| 14 | 2 | command acceleration | `mm/s^2`, signed |
| 16 | 2 | measured acceleration | `mm/s^2`, signed |
| 18 | 2 | yaw rate | `0.1 deg/s`, signed |
| 20 | 2 | CRC | CRC16-Modbus over offsets 2..19 |

The master emits one frame every 20 ms from the line-tuning application. The
F407 parser accepts a frame only after SOF, version/type and CRC pass. It
resynchronizes after a damaged frame and treats a sample older than 150 ms as
stale.

## IMU calibration contract

The existing `-0.45 dps` value is a yaw gyro bias only. It is not reused for
longitudinal acceleration. On startup the master collects 400 valid samples
while stationary, averages the configured acceleration axis as its bias, then
applies the configured scale (`2048 LSB/g` for the current +/-16 g register
setting) and a first-order low-pass (`alpha=0.15`). Axis and sign remain
explicit configuration values until the forward-push test is completed.

## Acceptance sequence

1. Program both artifacts manually and power both boards with motors disabled.
2. Confirm F407 Bluetooth output contains `CHASSIS_UART_DMA_READY` and then
   repeated `CHASSIS_FF,state=LOCKED` lines.
3. Leave the car still for at least 3 seconds. `frames` must increase, `crc`
   must remain zero, and `imu` should settle near zero after calibration.
4. Push the car forward by hand and release it. Confirm `imu` changes with a
   repeatable sign and returns toward zero. Repeat backward and check the sign
   reverses.
5. Run the chassis at low speed and compare `cmd` against `imu`; record the
   measured sign, scale and delay before changing configuration.

Do not use `tilt_mrad` to command X42S until the axis, sign, frame age, pitch
mechanics and a bounded manual stop test are all accepted. Until then
`PITCH_FF=LOCKED` is the intended state.

## Failure behavior

CRC/format errors are counted and discarded. DMA overflow and stale frames are
diagnostic faults; they do not cause a motor command. A missing or invalid IMU
sample makes the master mark the measured acceleration invalid. The F407 keeps
the previous estimate for observation only and never sends an automatic pitch
motion command.
