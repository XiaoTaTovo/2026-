# RED digital diagnostic smoke result

- Artifact SHA-256: `73645171B3F40028F4E581867F2184057F8C9FFF3A645E09ABE857E202E6A110`
- Capture port: COM7, 115200 8N1, RTS/DTR disabled, no serial writes
- Capture duration: 10024 ms
- Raw capture: `raw/smoke_com7_manual_program.log`
- Raw capture SHA-256: `E3A3FD386E6865047BB733A70F2E8E2CA58B1FF5FAFECB080CE2D9D1927CE736`

## Parsed result

| Check | Result |
| --- | --- |
| Complete diagnostic CSV frames | 202 |
| Timestamp range | 133000 ms to 143050 ms |
| Timestamp delta | exactly 50 ms for every adjacent frame |
| GPIO API value versus DIN bit | 0 mismatches |
| PB17/PB18/PB19 output-enable bits | 0 invalid frames |
| PINCM43/44/45 | `0x00040081` in all frames |
| Internal pull configuration | none (`0x00040081` has neither PIPU nor PIPD) |
| Diagnostic errors | 0 in all frames |
| Diagnostic faults | 0 in all frames |

At this smoke stage no module D output was connected. The observed D5/D6/D7
logic values were all high and are evidence only for a stable MCU-side input
configuration. They are not evidence about the optical module.
