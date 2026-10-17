# Speed sensor drift (one of three channels)

**Result: PASS**

Unit carries 80 % load. Speed channel 1 starts to drift upward at 10 RPM/s (up to +300 RPM). With three channels and median voting the controller must keep the real speed on target, flag the bad channel, and not trip.

## Criteria

| | Criterion | Measured | Limit |
| --- | --- | --- | --- |
| PASS | Real speed never falls below 3450 RPM | minimum 3499.4 | > 3450 |
| PASS | Real speed never exceeds 3550 RPM | peak 3500.5 | < 3550 |
| PASS | Channel 1 is flagged bad within 15 s of the drift starting | set at 10.2 s | between 5 and 20 s |
| PASS | Channel 2 is never flagged | never set | never |
| PASS | Controller never trips | never set | never |

## Key figures

- Peak speed: 3500 RPM
- Final speed: 3500 RPM
- Peak exhaust gas temperature: 488 degC
- Peak fuel flow: 1.26 kg/s
- Peak electrical output: 18.2 MW
- Tripped: no
- Peak exhaust spread: 5 degC

## Events

- t = 5.0 s: speed_fault (ch=0, mode=drift, a=10.0, b=300.0)

## Plot

![scenario plot](plot.png)

## Run record

- mode: real-time HIL (software rehearsal: virtual serial, controller on Linux)
- scenario file: scenarios/speed_sensor_drift.json
- scenario sha256 (first 12): 8d0ff1babdf4
- run at (UTC): 2026-10-12 17:16:21
- seed: 31
- sensor noise: True
- rated electrical power (MW): 22.72
- python: 3.13.16
- host: Linux-6.18.44-fc-v114-x86_64-with-glibc2.39
- negative tests: not run in real time (see the software tier)
