# Gas Turbine HIL - Week 1

Plant model, stand-in governor, scenario runner and the first two scenarios.

## Quick start
    make test          # builds, runs 28 unit checks, then both scenarios
    make scenarios     # scenarios only
Needs: gcc, make, python3 with numpy and matplotlib.

## Layout
- `include/` shared constants and the Modbus register map (v0.x, frozen in week 2)
- `plant/` physics model (`plant.c`) and the sim facade used by Python (`sim.c`)
- `controller/` stand-in min-select governor (replaced by the STM32 controller in week 3)
- `scenarios/` one JSON file per scenario: events, criteria
- `tools/scenario_runner.py` runs a scenario, evaluates criteria, writes reports
- `tests/test_plant.c` unit tests
- `results/<scenario>/` report.md, report.json, data.csv, plot.png
- `docs/physics.md` the physics explained

## Week 1 status
| Scenario | Result |
|---|---|
| Startup to rated speed | PASS (rated at 42.6 s) |
| Load rejection from 50 % | PASS (peak 3636 RPM, settles in 7 s) |
