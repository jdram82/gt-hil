# Gas Turbine HIL - Tier 1 (standard Linux)

Software model of a single-shaft gas-turbine generator set (the "plant"), the controller that runs it, a scenario runner that
proves the controller against faults, and a frozen Modbus register map. Week 3 moves the controller to an STM32 and the plant to
a BeagleBone Black; the files below are written so they compile unchanged on both.

## Quick start
    make test          # builds, runs 83 unit checks, then all 7 scenarios and their negative tests
    make unit          # unit checks only
    make scenarios     # scenarios only
Needs: gcc, make, python3 with numpy and matplotlib. A fresh clone should pass `make test` with no other setup.

## Layout
- `include/hil_constants.h`, `include/hil_regs.h` limits, and the Modbus register map **v1.0** (frozen; see `docs/register_map.md`)
- `plant/` physics model (`plant.c`) and the simulation facade used by Python (`sim.c`: link model, plant watchdog, breaker gating)
- `controller/` the controller: `standin_governor.c` (sequencing, min-select, latches), `sensor_vote.c`, `protection.c`
- `scenarios/` one JSON file per scenario: events, pass criteria, negative tests
- `tools/scenario_runner.py` runs a scenario, judges it, then re-runs it with one rule removed and checks it now fails
- `tests/test_plant.c` (28 checks, physics), `tests/test_controller.c` (55 checks, control philosophy)
- `results/<scenario>/` report.md, report.json, data.csv, plot.png, plus `negative_*.png`
- `docs/physics.md` the physics and the controller explained, with the week-2 findings; `docs/register_map.md`

## Status (end of week 2)
| Scenario | Result | What it proves |
|---|---|---|
| Startup to rated speed | PASS (rated at 42.6 s) | start sequence and governor hand-over |
| Load rejection from 50 % | PASS (peak 3647 RPM, settles in 7 s) | governor catches a 50 % load loss |
| Exhaust temperature runaway | PASS (alarm, trip, first-out, gas < 700 °C) | over-temperature protection |
| Speed sensor drift | PASS (speed held within 1 RPM of 3500) | 2-of-3 voting and diagnostics |
| Exhaust spread | PASS (alarm 40 °C, trip 80 °C) | combustor-fault detection |
| Multiple sensor failures | PASS (1 fault alarms, 2 trip) | loss-of-voting trip |
| Communication loss | PASS | two-way watchdog, latched trip, explicit reset, start lock-out |

Every scenario also has a **negative test**: one rule is switched off and the scenario must then fail, otherwise the scenario
does not really prove the rule. All 7 negative tests fail as intended.

## Not done yet
- Plant and controller still run in one process. Week 3: STM32 port + Modbus master; BeagleBone Modbus slave.
- Droop mode is covered by unit tests, not by a scenario.
- No vibration channel (fault code 3 is reserved).
