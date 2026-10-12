# Startup to rated speed

**Result: PASS**

Cold start from rest: crank, purge, light-off, accelerate, starter cut-out, governor takes over at rated speed (breaker open, no load).

## Criteria

| | Criterion | Measured | Limit |
| --- | --- | --- | --- |
| PASS | Speed reaches 3500 +/- 50 RPM within 60 s and holds for 10 s | reached at 42.6 s | <= 60 s |
| PASS | Speed never exceeds the 3850 RPM overspeed trip level | peak 3500.4 | < 3850 |
| PASS | Exhaust gas temperature stays below the 620 degC trip level | peak 255.2 | < 620 |
| PASS | Controller never trips | never set | never |

## Key figures

- Peak speed: 3500 RPM
- Final speed: 3500 RPM
- Peak exhaust gas temperature: 255 degC
- Peak fuel flow: 0.38 kg/s
- Peak electrical output: 0.0 MW
- Tripped: no

## Events

- t = 0.0 s: start

## Plot

![scenario plot](plot.png)

## Run record

- scenario file: scenarios/startup_to_rated.json
- scenario sha256 (first 12): 80dc497ad128
- run at (UTC): 2026-10-10 08:48:40
- git commit: not a git repository
- seed: 7
- sensor noise: True
- rated electrical power (MW): 22.72
- python: 3.14.4
