# Gas Turbine HIL - Tier 1 (standard Linux)

Software model of a single-shaft gas-turbine generator set (the "plant"), the controller that runs it, a scenario runner that
proves the controller against faults, and a frozen Modbus register map. Week 3 moves the controller to an STM32 and the plant to
a BeagleBone Black; the files below are written so they compile unchanged on both.

## Quick start
    make test          # builds, runs 100 unit checks, then all 7 scenarios and their negative tests
    make unit          # unit checks only
    make scenarios     # scenarios only
Needs: gcc, make, python3 with numpy and matplotlib. A fresh clone should pass `make test` with no other setup.

## Layout
- `include/hil_constants.h`, `include/hil_regs.h` limits, and the Modbus register map **v1.0** (frozen; see `docs/register_map.md`)
- `plant/` physics model (`plant.c`) and the simulation facade used by Python (`sim.c`: link model, plant watchdog, breaker gating)
- `controller/` the controller: `standin_governor.c` (sequencing, min-select, latches), `sensor_vote.c`, `protection.c`
- `scenarios/` one JSON file per scenario: events, pass criteria, negative tests
- `tools/scenario_runner.py` runs a scenario, judges it, then re-runs it with one rule removed and checks it now fails
- `tests/test_plant.c` (28 checks, physics), `tests/test_controller.c` (58 checks, control philosophy), `tests/test_modbus.c` (14 checks, framing)
- `results/<scenario>/` report.md, report.json, data.csv, plot.png, plus `negative_*.png`
- `common/` Modbus RTU (CRC, master and slave framing) and the master cycle (`hil_master.c`) shared by the STM32 and the Linux test node
- `bbb/plant_node.c` BeagleBone program: 10 ms real-time plant loop, Modbus slave, timing statistics, TCP test-command port
- `stm32/` HAL port (`hil_port_stm32.c`) and app loop (`hil_app.c`) for the Nucleo-F446RE
- `tools/hil_runner.py` runs a scenario in real time against plant_node + the controller over a serial link; `tools/vserial.py` is a virtual null-modem for rehearsal
- `results_hil/<scenario>/` real-time HIL rehearsal reports; `results_hil/timing_single_run/` timing from one uncontended run
- `docs/hil_bringup.md` wiring, BeagleBone and CubeIDE steps, run commands, troubleshooting
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

## Status (end of week 3)
All 7 scenarios pass in **real time over a Modbus RTU link** (115200 8E1, 50 ms poll, 10 ms plant step), with the same controller source
that passed in software. This rehearsal ran on a PC with a virtual serial pair (software link, no wire).

| Measurement (one uncontended run, 60 s load-rejection) | Value |
|---|---|
| Poll period | mean 50.0 ms, p99 51.7 ms, max 83 ms |
| Plant response latency | mean 11 us, p99 22 us |
| Plant loop wake lateness | mean 145 us, p99 358 us, max 43 ms (shared cloud VM; expect better with `--rt` on the BBB) |
| Modbus errors | 0 bad frames, 0 exceptions |

Parallel runs on a 2-core machine distort timing; use `results_hil/timing_single_run` for numbers.
Communication-loss has HIL-specific criteria (the plant cannot see controller state while the link is down); see `scenarios/communication_loss.json`.

## Honest limits
- The STM32 firmware is **syntax-checked against a stub HAL only**. It has not been compiled for ARM or run on the board. Bring-up: `docs/hil_bringup.md`.
- Real-wire timing (UART transfer time, USB-serial, BBB scheduling) is not measured until the boards are connected.
- Negative tests run in software mode only, not in real time.
- Droop mode is covered by unit tests, not by a scenario.
- Not covered: normal shutdown scenario, failed start / flame-out, vibration (fault code 3 reserved), IGV, surge, fuel gas system, grid model. Week 4 candidates.
