# LINE8 GPIO Integration

The seller's MSPM0 GPIO sample reads `X1` through `X8` independently and its
reference tracking logic treats low as black-line detected. `RedArray` now
supports this as `RED_ARRAY_SIGNAL_DIGITAL`: it performs a majority vote over
five GPIO frames and publishes `1000` for a detected line and `0` otherwise.

`drivers/red_array.h/.c` is the portable driver boundary. It does not include
MSPM0 DriverLib: another board only needs to implement `RedArrayReadFrameFn`
and return eight values where zero is electrical low and nonzero is electrical
high. `platform/ti_mspm0_platform.c` is the MSPM0G3507 adapter.

## H9 Wiring

Use separate jumpers. Do not linearly plug the module's connector into H9.

| LINE8 pin | H9 net | MSPM0 pin |
| --- | --- | --- |
| GND | GND | GND |
| X1 | D0 | PA18 |
| X2 | D1 | PA21 |
| X3 | D2 | PB24 |
| X4 | D3 | PB25 |
| X5 | D4 | PB17 |
| X6 | D5 | PB18 |
| X7 | D6 | PB19 |
| X8 | D7 | PB20 |

Power the module only according to its vendor documentation. Before attaching
any GPIO, verify its high output level is at most 3.3 V.

## Required SysConfig Change

Create a GPIO instance named `GPIO_LINE8` with these eight digital inputs:

`X1=PA18`, `X2=PA21`, `X3=PB24`, `X4=PB25`, `X5=PB17`, `X6=PB18`,
`X7=PB19`, `X8=PB20`.

Remove those eight pins from ADC instances. Keep the existing GRAY ADC0/PA27
configuration unchanged. The platform intentionally fails closed until the
generated `GPIO_LINE8_X1_PORT` through `GPIO_LINE8_X8_PORT` macros exist.

The current checked-in SysConfig has only the temporary `GPIO_RED_DIAG`
instance for PB17/PB18/PB19, so the normal LINE8 adapter is deliberately not
live yet. Preserve that diagnostic setup until all eight GPIO assignments are
created in SysConfig. The current standalone diagnostic entry is selected by
`H2026_RED_DIGITAL_DIAGNOSTIC_BUILD` in `red_digital_diagnostic.h` and keeps
the motor disabled.

## Bring-Up Order

1. Leave `H2026_RED_DIGITAL_DIAGNOSTIC_BUILD` at `1`, with the motor power
   physically disconnected. Prove the temporary D5/D6/D7 inputs change
   independently when their corresponding sensor heads cover/uncover a black
   strip.
2. In the SysConfig GUI, replace the temporary ADC/diagnostic allocation with
   the eight `GPIO_LINE8` inputs above. Generate code and confirm the generated
   `ti_msp_dl_config.h` contains all eight `GPIO_LINE8_X*_PORT` and
   `GPIO_LINE8_X*_PIN` macros.
3. Set `H2026_RED_DIGITAL_DIAGNOSTIC_BUILD` to `0` and
   `H2026_LINE8_GPIO_DIAGNOSTIC_BUILD` to `1` in
   `red_digital_diagnostic.h` and `line8_gpio_diagnostic.h`, respectively.
   Build the complete eight-channel diagnostic. It refuses to compile when any
   GPIO macro is missing and calls `TB6612_Stop()` continuously.
4. With wheels raised or motor power still removed, verify white gives
   `X1..X8=1`, then cover one head at a time with black and verify only the
   corresponding `Xn` changes to `0`. `BLACK` is the inverted bit mask, so a
   covered X1 reads `BLACK=01`, X8 reads `BLACK=80`.
5. Set `H2026_LINE8_GPIO_DIAGNOSTIC_BUILD` back to `0` only after all eight
   tests pass. The platform reader already provides this frame to the formal
   `CAR_TRACK_SENSOR_RED_ARRAY` path, with `normalized[i] = 1000` for black.
   Set `H2026_LINE8_REVERSE_ORDER` to `1` only if left and right are reversed.

The formal `CarFirmware` main in this checkout is currently commented out;
the new eight-channel diagnostic is therefore the supported immediate test
build. The GPIO driver and platform adapter are ready for that formal entry
point once it is restored and the SysConfig step has generated the eight
macros.

## Polarity And Orientation

The seller sample treats low as black line, so the default mask is `0xFF`.
If the sensor is installed mirrored, set `H2026_LINE8_REVERSE_ORDER` to `1`.
If one channel has inverted logic, clear that bit in
`H2026_LINE8_ACTIVE_LOW_MASK`.
