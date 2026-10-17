# Load rejection from 50 percent load

**Result: PASS**

Unit is started, brought to rated speed and loaded to 50 %. The generator breaker then opens and the whole load is lost at once. The governor must catch the speed surge.

## Criteria

| | Criterion | Measured | Limit |
| --- | --- | --- | --- |
| PASS | Speed is within 3450-3550 RPM just before the load is lost | 3500.1 at 9.5 s | 3450 to 3550 |
| PASS | Peak speed stays below the 3850 RPM overspeed trip level | peak 3647.4 | < 3850 |
| PASS | Speed returns to 3450-3550 RPM within 30 s of the load loss and holds for 10 s | settled 7.3 s after event | <= 30 s |
| PASS | Exhaust gas temperature stays below the 620 degC trip level | peak 378.0 | < 620 |
| PASS | Controller never trips | never set | never |

## Key figures

- Peak speed: 3647 RPM
- Final speed: 3500 RPM
- Peak exhaust gas temperature: 378 degC
- Peak fuel flow: 0.88 kg/s
- Peak electrical output: 11.4 MW
- Tripped: no
- Peak exhaust spread: 5 degC

## Negative tests (the rule is removed; the scenario must FAIL)

| Outcome | Rule removed | Criteria that failed | Why this proves the rule |
| --- | --- | --- | --- |
| FAILS AS INTENDED | governor, accel_limiter, breaker_preset | stable_before, peak_speed, recovers, no_trip | With the governor removed the rotor is left with the fuel it had at 50 % load and runs away until the overspeed protection trips it. |

## Events

- t = 10.0 s: open_breaker

## Plot

![scenario plot](plot.png)

## Run record

- scenario file: scenarios/load_rejection.json
- scenario sha256 (first 12): 2c71792718e9
- run at (UTC): 2026-10-17 15:53:16
- git commit: f5faab9
- seed: 11
- sensor noise: True
- rated electrical power (MW): 22.72
- python: 3.14.4
