# RED D5 digital diagnostic result

- Artifact SHA-256: `73645171B3F40028F4E581867F2184057F8C9FFF3A645E09ABE857E202E6A110`
- Target: MSPM0G3507 LQFP-64, manually programmed by the operator.
- Connection label: module `D5` to MCU `PB17`; 3.3 V module supply and common ground; IR disconnected; no other module D output connected.
- Capture: COM7, 115200 8N1, RTS/DTR disabled, no serial writes.
- Safe state: diagnostic reports PB17/PB18/PB19 as GPIO inputs with no pull; TB6612 STBY remains safe; every captured record reports `errors=0,faults=00`.

## Raw evidence

| State label | Raw file | SHA-256 | Complete CSV frames | D5/PB17 level | Timestamp delta |
| --- | --- | --- | ---: | --- | --- |
| White paper | `raw/D5_white_retry1.csv` | `28C36ABED6BE361679A0C280D628597F0DD96B013C26E43940A6AA01957F7147` | 190 | 1 in 190/190 | 50 ms in 189/189 gaps |
| Black paper | `raw/D5_black.csv` | `E315D5AE7A6913600855E83794E580DC0D0AFE9A28E359A6A6A459FD2EFE1CFB` | 204 | 1 in 204/204 | 50 ms in 203/203 gaps |

In both captures, the D5 GPIO API value agreed with `DIN_B` bit 17 in every frame; PB17-PB19 output-enable bits were clear, and PINCM43/44/45 were `0x00040081` in every frame.

## Result

`D5` did not change level between the operator-labelled white and black states. This is not enough to classify D5 as a working digital output, and it is not evidence that the full module is unusable. D6 and D7 must be tested with the same controlled sequence before a module-level conclusion.

The requested multimeter D5-to-GND voltages for the two states were not supplied, so the required three-way comparison (GPIO register, serial log, meter) is incomplete.
