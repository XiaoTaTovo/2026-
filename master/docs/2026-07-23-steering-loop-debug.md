# Steering loop debug record

## Implemented path

- The IMU task runs every 5 ms and integrates the calibrated yaw rate.
- The control task runs every 20 ms and calls `CarRouteExecutor_Update`.
- A `CAR_SEGMENT_TURN` uses a proportional yaw loop. The remaining angle
  selects the requested wheel speed. In the current TB6612 task backend this
  request is mapped directly from mm/s units to PWM; there is no task speed PID.
- Positive route angles keep the existing convention: left wheel is stopped and
  right wheel moves forward. Negative angles use the opposite wheel.

The initial parameters are `turn_heading_kp = 2.0` and
`turn_min_speed_mm_s = 30.0`. They are starting values, not measured results.

## Required bench sequence

1. Keep the car still and level after power-up. Wait for 400 IMU samples
   (about 2 seconds at the current 5 ms period).
2. Confirm the estimator is calibrated and yaw is stable. Rotate the chassis by
   hand in both directions and confirm the sign of `yaw_deg`.
3. In CCS select `PROJECT_MODE_TURN_DEBUG`. Lift the wheels and run the
   built-in 30 degree positive turn only long enough to confirm the commanded
   wheel and PWM. The chassis cannot produce yaw while supported, so stop or
   reset before the six-second segment timeout.
4. Put the car on the floor and run the same low-speed 30 degree turn. Record the
   start yaw, target yaw, final yaw, overshoot, elapsed time, and any timeout.
5. Repeat the same turn ten times. Only after this passes should the turn
   segments be used in the H2024 route.

Change one parameter per test: first `turn_heading_kp`, then
`turn_min_speed_mm_s`, then `angle_tolerance_deg`. Do not retune the motor-board
speed PID during this step.

## Blocking conditions

- If the yaw sign is wrong, change the platform `H2024_IMU_YAW_SIGN` after
  rechecking the physical axis; do not compensate with the turn gain.
- If yaw is not calibrated or becomes stale, the safety supervisor must stop
  the route. A successful host build does not prove the IMU wiring or motor
  polarity on the real car.
- The PCB-verified mapping is `KEY1=PB23`, `KEY2=PB26`, `KEY3=PB27`.
  Regenerate `ti_msp_dl_config.h/.c` in CCS and verify these generated macros
  before burning the board.

## Telemetry naming

The VOFA CSV column order is unchanged. Only the two labels at positions 13 and
14 changed from `ppr_l,ppr_r` to `cpr_l,cpr_r`; the Bluetooth command accepts
`CPR` and keeps `PPR` as a compatibility alias.

## Buttons and OLED

The button driver currently implements a debounced press event only. A short
press and a long press both produce one event after debounce; holding a button
does not auto-repeat and does not change parameters.

| Mode | KEY1 | KEY2 | KEY3 |
| --- | --- | --- | --- |
| Gray debug | capture white | capture black | retry after calibration error |
| H2024/turn debug | start route; press again for emergency stop | switch OLED P1/P2 | reserved |

H2024 and turn-debug modes render status at 200 ms intervals and transfer one
OLED hardware page per main-loop pass. The display no longer alternates pages
automatically. P1 is the route/output page and P2 is the arm/backend diagnostic
page. KEY2 selects the page. `K:123` is the raw level of KEY1/KEY2/KEY3: an
unpressed key is `1`, and a held key is `0`. The current firmware uses OLED
`SDA=PA0` and `SCL=PA1`; PB2/PB3 are reserved for `UART_BLUETOOTH`.

`H2024_MODE_TURN_DEBUG` does not require gray calibration because its route
contains no line-following segment. Gray calibration remains mandatory for
H2024 items 2 through 4.

## Verified hardware result (2026-07-24)

- The positive debug target is 30 degrees.
- The real chassis turns left and stops at about +27 degrees.
- Repeated turn behavior has been accepted by the user as normal.
- Measured odometry inputs are wheel diameter 65 mm and encoder CPR 724.
- The first ITEM1 floor baseline completed successfully with good straight-line
  behavior; parameter optimization is intentionally deferred to the next day.
- This is the intended result: `angle_tolerance_deg` is 3 degrees, so the turn
  segment advances when the remaining error is less than or equal to 3 degrees.
- At the start of the turn the command is approximately `L=0, R=60 mm/s` and
  the TB6612 adapter displays approximately `PWM=0/14`.
- The successful run proves the complete path: KEY1 press -> arm -> safety
  gate -> route executor -> yaw P control -> TB6612 -> IMU feedback -> stop.

## Confirmed incident log

### I01: Correct source changed but wrong firmware behavior remained

- Symptom: only the Bluetooth `I/E/M` page appeared and task PWM stayed zero.
- Cause: the C-drive CCS project and the D-drive repository copy had diverged;
  the actual C-drive build still selected Bluetooth mode.
- Evidence: the map contained `BluetoothControl_Update` but not
  `CarFirmware_Tick` or `CarRouteExecutor_Update`.
- Prevention: after every mode change, clean-build the C-drive project and
  verify the task symbols in the generated map before flashing.

### I02: Button pin assumptions did not match the PCB

- Symptom: only one physical key affected the display and calibration actions
  appeared attached to the wrong key.
- Cause: an earlier pin list used the wrong GPIO mapping.
- Verified mapping: KEY1=PB23 (middle), KEY2=PB26 (right), KEY3=PB27 (left),
  all active-low. Raw pressed states are 011, 101, and 110 respectively.

### I03: OLED refresh starved button and control polling

- Symptom: P1/P2 alternated continuously, short KEY1 presses were missed, and
  the control loop was difficult to observe.
- Cause: at 100 kHz I2C the old code cleared and sent all 1024 framebuffer bytes
  every 50 ms. The blocking transfer consumed most of the main loop.
- Fix: KEY2 now selects a stable page; rendering is every 200 ms and transfer
  uses `OLED_UpdatePages(tx_page, 1)` so each pass sends only one hardware page.

### I04: False `F:00000002` immediately after a successful arm

- Symptom: `BTN:1 ARM`, `STAT:0`, then `APP:N RUN:N`, `M:N`, `PWM:0/0`, while
  the current encoder field still displayed `E:Y`.
- Cause: the main loop captured `now_ms`, then the direct TB6612 encoder adapter
  captured a timestamp one SysTick later. Unsigned `now_ms - timestamp_ms`
  underflowed to a huge age and falsely raised `CAR_FAULT_ENCODER_STALE`.
- Fix: the stale comparison now uses signed wrap-safe elapsed-time arithmetic.
- Regression coverage: normal age, timestamp one tick in the future, timer
  wraparound, and genuinely stale samples are covered by
  `tests/test_safety_timestamp.c.reference`.

### I05: Task stopped early but OLED returned to `SEG:00 STOP`, `F=0`

- Symptom: ITEM1 travelled less than one metre, stopped with no fault, then the
  display showed `SEG:00 STOP`, so line exit and distance exit were impossible
  to distinguish.
- Cause 1: `CarApp_Update` clears each output snapshot after the app disarms;
  the OLED was reading that transient output instead of the executor's latched
  index/finished state.
- Cause 2: a line-enabled 1000 mm straight arms gray exit after 500 mm. A white
  floor falsely classified as a line can therefore end the segment as soon as
  the half-distance guard opens.
- Fix: OLED now reads the executor latch and displays the last motion exit as
  `LINE`, `DIST`, or `ANGLE`; CUE and STOP segments do not overwrite it.
- Fix: KEY3 is a resident two-surface runtime calibration control in H2026
  modes. Every power-up starts at `G:WHITE`; white and black captures each use
  a 16-frame average, and KEY1 is rejected until `G:OK`.
- Safety: KEY3 is ignored while a route is running. A stopped/finished vehicle
  can press KEY3 again at any time to start a new white/black calibration.

### I06: `SEG:04 DONE LINE` before the physical 100 cm endpoint

- Evidence: `DONE LINE` with a zero fault mask proves the route intentionally
  accepted a gray endpoint; it is not an encoder-distance exit or a fault.
- First fix rejected: requiring 6/8 active channels and 4000 confidence would
  reject the real marker because the measured line is only about 1 cm wide and
  normally reaches two probes.
- Final rule: endpoint detection arms after 80% of the segment distance, then
  requires two adjacent active probes, confidence at least 1200, and two
  consecutive 20 ms control frames. Arc tracking keeps its original sensitive
  `line.valid` rule.
- Timing basis: at the current 180 mm/s straight speed, a 10 mm marker remains
  under the array for about 55 ms, so two 20 ms frames fit inside the marker.
- Diagnostics: P2 shows the live active mask as `M:xx`. On a line exit it
  latches `END C/A/M` and replaces `BIAS` with `TRIG`, the segment progress in
  millimetres at the exact accepted frame.

## OLED field reference

### P1 route/output page

| Field | Meaning | Expected or possible values |
| --- | --- | --- |
| `MODE` | Selected route mode | `TURN`, `ITEM1` to `ITEM4`, or `?????` |
| `TB` | Static intended-backend label | Confirm the real callback with P2 `TB:Y` |
| `Y/G` | Yaw and runtime gray calibration | Yaw `WAIT/OK`; gray `WHITE/CAPW/BLACK/CAPB/OK/ERR` |
| `YAW` | Integrated yaw in degrees | Positive is left; reset to zero at arm |
| `SEG` | Latched route state and last motion exit | State plus `LINE`, `DIST`, `ANGLE`, `REACQ`, or `-` |
| `L/R` | Requested wheel speeds | Signed mm/s commands, not measured speeds |
| `PWM` | Last TB6612 commands | Signed duty percent, limited to +/-80 |
| `F` | Combined fault mask | Eight hexadecimal digits; zero is normal |
| `K:123` | Raw keys and actions | KEY1 run/stop, KEY2 page, KEY3 gray calibration |

TURN normally progresses from `SEG:00 STOP` to `SEG:01 RUN` and finally
`SEG:03 DONE`. ITEM1 normally runs at segment 01 and finishes at index 04.
ITEM2 motion segments are 01 straight, 03 half-circle, 05 straight,
07/09/11 line-follow, and 08/10 right turn; it finishes at index 13.

### P2 arm/backend page

| Field | Meaning | Expected or possible values |
| --- | --- | --- |
| `BTN` | Debounced KEY1 event count and result | Action is `NONE`, `ARM`, `REJ`, or `STOP` |
| `I/E/M/T` | IMU, encoder, motor-ready, TB direct backend | Normally all `Y` |
| `STAT` | Last arm return value | 0 OK, -1 input/argument, -2 state, -3 capacity |
| `GC` | Runtime gray calibration state | `WHITE K3`, capture count, `BLACK K3`, `OK`, or channel/span error |
| `LN/END` | Live or latched line estimate | valid flag, confidence, active count and `M` channel bit mask |
| `APP` | High-level application armed | `Y` while a valid route owns motion |
| `RUN` | Route executor running | `Y` only during active route execution |
| `PWM` | Same physical command as P1 | 0/0 stopped; TURN starts near 0/14 |
| `BIAS/TRIG` | Gyro bias or latched line-exit progress | Bias is about -0.5 to -0.4 dps; `TRIG` is mm |

The normal pre-start state is `I:Y E:Y M:Y T:Y`, `STAT:0`, `GC:OK`,
`APP:N RUN:N`, and `PWM:0/0`. A normal start gives `BTN:n ARM`, `APP:Y RUN:Y`.

### Fault values

| Hex | Meaning |
| --- | --- |
| `00000000` | No fault |
| `00000001` | Emergency stop |
| `00000002` | Encoder invalid/stale |
| `00000004` | IMU invalid/stale |
| `00000008` | Gray sample invalid/stale while required |
| `00000010` | Segment timeout |
| `00000020` | Invalid route/geometry |
| `00000040` | Motor preparation/output failure |
| `00000080` | IMU initialization failure |
| `00000100` | Gray calibration missing for a gray-dependent task |

Faults are bit masks and can combine; for example, `00000006` means encoder
and IMU stale together.

## Next acceptance sequence

1. Repeat +30 degrees ten times. Require 10/10 no-fault completion and record
   final yaw and elapsed time; the initial acceptance band is 27 to 33 degrees.
2. Change only `H2024_DEBUG_TURN_DEG` to -30. Expect `L:+60 R:+0`, approximately
   `PWM:14/0`, decreasing yaw, and a final yaw near -27 degrees.
3. In H2026 mode, put all eight sensors on white and press KEY3; wait for
   `GC:BLACK K3`, move all sensors to black, press KEY3 again, and require
   `GC:OK` before KEY1 can arm the route.
4. Select `PROJECT_MODE_H2026_ITEM_1` and lift the wheels first. Expect
   `SEG:01 RUN`, approximately `L/R=180/180`, and `PWM=41/41` before correction.
5. Before judging the one-metre stop, replace placeholder wheel diameter and
   CPR with measured values. Record distance, lateral error, final yaw, time,
   battery voltage, and fault mask for every floor run.
6. Start ITEM2 only after ITEM1 is repeatable. Its line PID is active on arc
   segment 03 and rectangle line-follow segments 07/09/11, not white-ground
   straight segments 01 and 05.

## H2026 item 2 line handoff and guarded rectangle

- Segment 05, C to D, reuses the item 1 thin-line stop condition. It can exit
  with `LINE` after 80% distance when two adjacent sensors reach confidence
  1200 for two frames; 1000 mm remains the `DIST` fallback.
- Segment 07 begins line following immediately after the D checkpoint. The
  rectangle is 400 mm line-follow, right 90 degrees, 800 mm line-follow,
  right 90 degrees, then 400 mm line-follow to A.
- The first two rectangle edges arm corner detection only after 75% distance.
  Two consecutive frames trigger on either position >= 2000 or the broader
  right-corner shape: right-side energy >= 650 permille, at least three active
  probes, and an active span of at least three channels. Three consecutive
  lost-line frames are also accepted near the end.
- A confirmed corner enters `AXLE` instead of turning immediately. The car
  holds the trigger yaw and advances at 80 mm/s for the initial measured
  gray-to-axle distance of 150 mm. The yaw turn begins only after the axle is
  approximately at the corner pivot. With no gray event, the hard fallbacks
  are map distance + 150 mm (550/950 mm for the first two rectangle edges).
- Rectangle straight tracking is gray-primary. A limited yaw term (`Kp=1.0`,
  maximum 15 mm/s correction) damps slow heading drift without overriding the
  line controller.
- H2026 right turns allow line reacquisition after 80 degrees. A line within
  +/-900 position for two consecutive frames exits with `REACQ`; the original
  90-degree yaw exit remains the `ANGLE` fallback.
- Segment 11 uses three consecutive lost-line frames near A as its line exit.
- The H2026 arc stores its theoretical tangent
  `arc_start_yaw + direction * arc_angle`. Segment 05 follows that target after
  the cue instead of locking the residual yaw at the instant the arc exits.
  This addresses the intermittent straight/diagonal C-to-D departure.
- The rectangle line-follow controller reuses `H2024_LINE_PID_KP/KI/KD`.
  Current values are P-only, `Kp=0.025`, `Ki=Kd=0`.
- Relevant code is in `app/h2024_task.c` for the map,
  `core/route_executor.c` for event/controller behavior, and
  `platform/ti_mspm0_platform_config.h` for thresholds.

## Route Bluetooth CSV

Task mode now transmits read-only route telemetry automatically on
`UART_BLUETOOTH` (`PB2=TX`, `PB3=RX`, 115200 8N1). It does not start the
independent Bluetooth motor controller and therefore cannot steal TB6612
ownership from the route. A row is sent every 100 ms and immediately when the
segment, phase, corner candidate/streak, reacquire streak, or exit reason
changes.

The banner starts with `#RCSV`; FireWater-compatible numeric data rows start
with the constant stream identifier `2026,`. Important columns:

| Column | Meaning |
| --- | --- |
| `seg/type` | Route index and segment type; H2026 corners are 07/08 and 09/10 |
| `phase` | `0=TRACK`, `1=AXLE` sensor-to-axle compensation |
| `progress_mm` | Encoder distance from the current segment start |
| `yaw_x10` | Yaw in 0.1 degree units |
| `line_valid/position` | Current line validity and signed centroid |
| `confidence/active/mask` | Total black strength, active probe count and bit mask |
| `right_pm/span` | Right-side energy ratio in permille and active-channel span |
| `candidate/event_streak` | Current corner evidence and consecutive confirmation frames |
| `reacq_streak` | Consecutive centered-line frames during the yaw turn |
| `axle_mm` | Distance advanced after the gray corner trigger |
| `cmd_l/cmd_r`, `pwm_l/pwm_r` | Requested mm/s and physical TB6612 duty commands |
| `exit` | `1=LINE`, `2=DIST`, `3=ANGLE`, `4=REACQ` |
| `g0..g7` | Eight normalized gray values, 0 white to 1000 black |

Normal first-corner evidence is `seg=7,phase=0,candidate=1`, then
`phase=1` while `axle_mm` rises toward about 147 mm (150 minus the 3 mm
tolerance), followed by `seg=8,exit=1`. A missed gray corner stays in phase 0
and reaches `seg=8,exit=2` near 550 mm. During turn, two centered frames after
80 degrees produce `seg=9,exit=4`; otherwise yaw reaches 90 degrees and exits
with `3`.

## Deferred improvements

- The TB6612 task backend now has a 50 ms encoder wheel-speed inner loop while
  retaining the current direct/open-loop fallback and resident Bluetooth
  tuning path.
- If the 1 cm endpoint still misses or false-triggers, tune one parameter at a
  time using latched `TRIG/C/A/M`: arming ratio first, confidence second, then
  frame count. Keep encoder distance as the unchanged fallback.

## 2026-07-27 TASK2-2 arc-to-straight diagnosis and fix

The latest recording is `C:\Users\taowz\Desktop\任务2-2.csv`. It contains old
and new firmware rows; only rows with `I0=2026` and `I32=1` belong to the new
task speed loop. Three completed half-circles reached the angle exit at
`1659/1736/1756 mm`, while the theoretical center-path length is only
`pi*400=1257 mm`. This identifies an oversized physical radius, not a route
table or encoder-distance exit.

The ARC exit is `abs(yaw - segment_start_yaw) + tolerance >= arc_angle`, so the
current one-degree tolerance exits near 179 degrees. Three runs explicitly
changed `SEG03 -> SEG04 -> SEG05`; only one disturbed run reversed near 176
degrees and ended with fault `0x10` (segment timeout). Moving the car between
white and black gray calibration does not preserve a wrong absolute launch
yaw: KEY1 resets yaw, and each ARC stores its own start yaw. The chassis must,
however, remain still until IMU `CAL:OK`, otherwise the gyro bias itself can be
contaminated.

The last run exposed a second failure after the software had entered SEG05.
It initially requested `53/53 RPM`, then a wheel-speed transient moved yaw away
from the tangent. The old straight outer loop used `Kp=4` and a 140 mm/s limit,
eventually requesting about `94/12 RPM`; that command makes the straight segment
look like a continuing circle even though the state machine is correct.

The H2026 item-2-only fixes are:

- Keep physical `track_width_mm=115` for encoder odometry.
- Use `arc_effective_track_width_mm=160` only for ARC wheel-ratio geometry. The
  no-line target changes from approximately `41/31` to `41/27 RPM`.
- Use straight heading `Kp=1.0` with a separate `25 mm/s` correction limit.
- Limit ARC gray correction to `20 mm/s`, so one edge-centroid spike cannot
  reverse the required inner/outer-wheel relationship.
- Keep H2024 routes and defaults unchanged; non-positive new fields select the
  previous limits and physical track width.
- Keep the existing 45-column route CSV order unchanged. `I42` now reports the
  same limited straight correction that the executor actually applies.

Host C11 `-Wall -Wextra -Werror` tests and the TI ARM Clang target build both
pass. The generated image is
`Debug/empty_mspm0g3507_nortos_ticlang.out`, SHA-256
`3966C9FDE267FBF61CA5C2E3891EA1B3161B9286220813C3C685FB19FD7D905B`.

For the next floor run, ARC should finish near `1250-1400 mm`, with targets
near `41/27 RPM`. SEG05 must show `53/53 RPM` initially; `abs(I42)` must never
exceed 25 and a disturbance must decay without reaching the former `94/12 RPM`
split. Stop after one run and inspect these fields before tuning anything else.

## 2026-07-27 yaw timing root cause and fix

An A/B test isolated the yaw scale from the task controller: with the OLED
disconnected, one physical 360-degree turn produced approximately
`yaw_x10=3600`. The gyro scale, sign, and measured stationary bias are therefore
not the primary cause of the earlier under-reported turns.

The old main loop could spend about 11 ms in blocking 115200-baud telemetry and
another long interval sending an OLED page at 100 kHz. When their combined
delay exceeded `imu_max_step_ms=20`, `CarYawEstimator_Update()` rejected the
whole interval and erased real rotation. That error directly delayed TURN,
ARC, and line-reacquisition angle conditions.

The current implementation makes the output path bounded:

- VOFA output uses a 1024-byte TX interrupt ring. A line is queued completely
  or dropped completely; the control loop never waits for bytes on the wire.
- UART hardware FIFO is enabled. RX still interrupts on one byte, while TX
  refills at the half-empty threshold.
- OLED is one fixed diagnostic page, uses 400 kHz I2C, redraws every 250 ms,
  and sends at most one display page per 5 ms service call.
- A sample interval above 20 ms is now counted as delayed but still integrated.
  Only an interval above 200 ms is rejected as a true stale-rate outage.

Host tests cover a 25 ms sample that must integrate and a 201 ms outage that
must be rejected. The TI ARM Clang build passes. SRAM usage is `0x1AAB` of
`0x8000`, leaving `0x6555` bytes. The current image is
`Debug/empty_mspm0g3507_nortos_ticlang.out`, SHA-256
`A52780647A4D94502920DE458E1E49AF021AF740BB4D762273EB2D96C6139E03`.

Hardware acceptance must keep the OLED connected and motors stopped. After
stationary IMU calibration, rotate left 360 degrees three times and right 360
degrees three times, power-cycling or recording the before/after delta for each
trial. Each absolute delta must be 350 to 370 degrees, with left positive and
right negative. Do not retune TURN or line parameters until this passes.
