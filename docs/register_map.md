# Modbus register map, version 1.0

Frozen at the end of week 2. Source of truth: `include/hil_regs.h` (both boards include it). Any change after this point must
bump `HIL_REGMAP_VERSION_MINOR` and be recorded here.

Link: Modbus RTU, 115200 baud, 8 data bits, even parity, 1 stop bit (8E1). BeagleBone Black is the **slave** (address 1) and runs the
plant; the STM32 Nucleo-F446RE is the **master** and runs the controller. Poll every 50 ms (the plant steps every 10 ms).
Analog values are unsigned 16-bit: `register = engineering value × scale`.

## Input registers (function 04), plant to controller

| Reg | Name | Scale | Meaning |
|---|---|---|---|
| 0 | `SPEED1_RPM` | 1 RPM | speed channel 1 |
| 1 | `SPEED2_RPM` | 1 RPM | speed channel 2 |
| 2 | `SPEED3_RPM` | 1 RPM | speed channel 3 |
| 3 | `TC1` | 0.1 °C | exhaust thermocouple 1 |
| 4 | `TC2` | 0.1 °C | exhaust thermocouple 2 |
| 5 | `TC3` | 0.1 °C | exhaust thermocouple 3 |
| 6 | `TC4` | 0.1 °C | exhaust thermocouple 4 |
| 7 | `PRESSURE` | 0.1 bar(g) | compressor discharge |
| 8 | `POWER` | 0.1 MW | electrical output |
| 9 | `FUEL_FLOW` | 0.01 kg/s | actual fuel flow |
| 10 | `DISCRETE` | bits | see below |
| 11 | `PLANT_HB` | count | plant heartbeat, +1 per answered poll (wraps) |

`DISCRETE` bits (one per condition, never a lumped "OK"):

| Bit | Name |
|---|---|
| 0 | `BREAKER_CLOSED` |
| 1 | `ESTOP_HEALTHY` |
| 2 | `FLAME` |
| 3 | `LUBE_OK` |
| 4 | `FUEL_OK` |
| 5 | `STARTER_OK` |
| 6 | `PLANT_WD` (plant-side watchdog has acted) |

## Holding registers (function 03 read, 16 write)

Written by the operator side (BeagleBone supervisor), read by the STM32:

| Reg | Name | Meaning |
|---|---|---|
| 0 | `START_CMD` | 1 run, 0 stop. A start needs a 0 to 1 edge after any trip. |
| 1 | `RESET_CMD` | rising edge resets a latched trip, if the cause has gone |
| 2 | `MODE` | 0 isochronous, 1 droop (4 %) |

Written by the STM32 master, read by the plant and the HMI:

| Reg | Name | Scale | Meaning |
|---|---|---|---|
| 3 | `FUEL_CMD` | 0.1 % | 0..1000 = 0..100 % valve |
| 4 | `DO_WORD` | bits | see below |
| 5 | `CTRL_HB` | count | controller heartbeat, +1 per cycle (wraps) |
| 6 | `CTRL_STATE` | enum | 0 IDLE, 1 CRANK, 2 IGNITE, 3 ACCEL, 4 WARMUP, 5 RUN, 6 COOLDOWN, 7 TRIP |
| 7 | `FIRST_OUT` | enum | latched first trip cause, 0 = none |
| 8 | `ALARM_WORD` | bits | see below |

`DO_WORD` bits: 0 `STARTER`, 1 `IGNITION`, 2 `SYNC_PERMIT` (breaker may close), 3 `TRIP_OK`
(de-energise-to-trip: 1 while healthy, 0 on trip; the plant opens the breaker when it falls).

`ALARM_WORD` bits: 0 overspeed (≥ 3780), 1 over-temperature (≥ 610 °C), 2 exhaust spread (≥ 40 °C), 3/4/5 speed channel 1/2/3 bad,
6 comms (≥ 3 failed polls), 7 start blocked.

`FIRST_OUT` codes: 1 overspeed, 2 over-temperature, 3 vibration (reserved), 4 low lube oil, 5 e-stop, 6 flame-out,
7 fail to light, 8 exhaust spread, 9 sensor fault, 10 comms loss, 11 fuel supply.

## Watchdog rules (two-way)

- Controller: 3 failed polls in a row raise `ALARM_COMMS`; 10 in a row (500 ms) trip with `FIRST_OUT = 10`.
- Plant: no write from the controller for 500 ms closes the fuel valve, stops starter and ignition and opens the breaker.
- Both sides also see a stopped heartbeat counter (`PLANT_HB`, `CTRL_HB`).
