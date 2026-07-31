# RED D5-D7 parallel digital screening result

- Artifact SHA-256: `73645171B3F40028F4E581867F2184057F8C9FFF3A645E09ABE857E202E6A110`
- Target: MSPM0G3507 LQFP-64; manual programming was reported by the operator.
- Connection reported by the operator: module D5 -> PB17, D6 -> PB18, D7 -> PB19; module VCC=3.3 V and GND common; IR and D1-D4/D8 disconnected.
- Deviation from the original isolated-channel plan: the operator instructed a simultaneous D5-D7 screening. It is valid for a three-channel black/white response observation, but it cannot localize a failed channel, wire, or connector.
- Capture port: COM7 at 115200 8N1, RTS/DTR disabled, no serial writes.

## Raw evidence

| Paper state | Raw file | SHA-256 | Frames | D5/PB17 | D6/PB18 | D7/PB19 | Timestamp gaps |
| --- | --- | --- | ---: | --- | --- | --- | --- |
| White (label supplied by operator immediately after capture) | `raw/D5_D6_D7_current_unlabeled.csv` | `10EE979950142F2F6E5BFDA9050A2D7FE754FA9E51079610046045C77F6310ED` | 144 | 1 in 144/144 | 1 in 144/144 | 1 in 144/144 | 50 ms in 143/143 |
| Black | `raw/D5_D6_D7_black.csv` | `663FC763ACAB2E42AFD2D446269014A6AC50104AFD2B1CD0E1559CD15E6B48BB` | 191 | 1 in 191/191 | 1 in 191/191 | 1 in 191/191 | 50 ms in 190/190 |

For every frame in both captures:

- D5/D6/D7 API reads matched `DIN_B` bits 17/18/19 respectively.
- `DOE_B` bits 17-19 were zero (inputs, not MCU outputs).
- PINCM43/44/45 were `0x00040081` (GPIO function and input enabled with no PIPU/PIPD).
- `errors=0` and `faults=00`.

## Conclusion

The MSPM0 configuration and firmware observation path are correct for PB17/PB18/PB19. All three observed inputs remained high for both white and black paper, so no route is verified as a working digital line-sensor output and the eight-channel digital driver must not be delivered.

This does **not** prove that the module is defective. The earliest remaining failure domain is physical: pin-order/continuity, module supply or emitter operation, fixed sensing height, the actual definition of the `IR` pin, or module/channel hardware. The required multimeter readings of D5/D6/D7-to-GND for white and black were not provided, so the requested GPIO-register/serial/meter three-way proof is incomplete.

## Minimum next physical test

With the existing 3.3 V/GND supply and IR still disconnected, measure VCC-to-GND and then each D5/D6/D7-to-GND on white and black paper at the same height. Do not drive the IR pin. If all three D pins remain near the same rail in both states while the MCU logs stay high, the MCU configuration is cleared and investigation stays on the module/power/emitter/height/IR-definition side.
