# Gas turbine plant model: the physics, step by step

This explains what `plant/plant.c` computes and why. The machine is a notional single-shaft,
industrial gas turbine driving a generator, rated 3500 RPM and about 22.7 MW electrical.
It is **not** a model of any real OEM machine. It is a physically consistent teaching and
test model: the numbers are chosen to be believable, and every behaviour that matters for
control testing (inertia, fuel lag, temperature limit, load rejection) emerges from the equations
rather than being scripted.

## 1. What the model has to do

A controller tests itself against a plant. The plant needs only three things to be right:

1. **Direction and size of cause and effect**: more fuel gives more temperature, more power, more acceleration.
2. **Time constants**: how fast each of those happens.
3. **Limits**: the places where the machine can be damaged (overspeed, over-temperature, flame-out).

The model has five states, so it runs in microseconds at a 10 ms step:

| State | Symbol | Meaning |
|---|---|---|
| Rotor speed | ω | rad/s, the only fast mechanical state |
| Fuel valve position | v | 0 to 1 |
| Fuel flow | ẇf | kg/s, after the manifold lag |
| Thermocouple temperature | T_tc | the lagged sensor reading |
| Flame | flame | 0 or 1 |

Everything else (pressures, temperatures, powers) is calculated algebraically from speed and fuel each step.

## 2. The Brayton cycle in one paragraph

Air is compressed (needs work), fuel is burned in it (adds heat), the hot gas expands through
the turbine (gives work). The turbine produces more work than the compressor absorbs because hot gas
expands with more energy than cold air was compressed. The **difference** is the net shaft power.
It is a difference of two large numbers, which is why a gas turbine is so sensitive to fuel and why
at low speed it cannot sustain itself.

## 3. Compressor

Speed in per unit: n = ω / ω_rated.

- **Air flow** ṁa = ṁa,rated · n · (ambient density ratio). Flow scales with speed (a fixed-volume pump).
- **Pressure ratio** r = 1 + (r_rated − 1) · n². Pressure rise goes with the square of tip speed.
- **Discharge temperature** from the isentropic relation with efficiency η_c:

  T_cd = T_amb · [ 1 + ( r^κa − 1 ) / η_c ],   κa = (γ−1)/γ = 0.2857

- **Compressor power** W_c = ṁa · cp_air · (T_cd − T_amb).

At the design point: r = 12, T_cd = 639 K (365 °C), W_c = **28.2 MW**. More than half of the
turbine's gross power is spent just compressing the air.

## 4. Combustor

Fuel heat release raises the temperature of the mixed stream:

   TIT = T_cd + ẇf · LHV · η_comb / ( (ṁa + ẇf) · cp_gas )

LHV = 47 MJ/kg (natural gas), η_comb = 0.99. At design (ẇf = 1.5 kg/s, ṁa = 80 kg/s):
**TIT = 1383 K (1110 °C)**, fuel/air ratio 0.019. This is a plausible industrial figure.

Flame logic (a real start depends on it):
- **Light-off** needs ignition ON, speed above 0.10 pu, and fuel/air ratio above 0.003.
- **Lean blowout**: if fuel/air falls below 0.002 the flame goes out, and the turbine produces no heat.

## 5. Turbine

- Expansion ratio r_t = r · (1 − 0.06 n²): inlet, combustor and exhaust pressure losses.
- Exhaust temperature with turbine efficiency η_t = 0.88, κg = 0.2481 (γ = 1.33):

  T_exh = TIT · [ 1 − η_t · ( 1 − r_t^(−κg) ) ]

- Turbine power W_t = (ṁa + ẇf) · cp_gas · (TIT − T_exh).

At design: T_exh = 833 K = **560 °C**, W_t = **51.5 MW**.

## 6. Design-point summary (computed, and checked by `tests/test_plant.c`)

| Quantity | Value |
|---|---|
| Compressor power | 28.2 MW |
| Turbine power | 51.5 MW |
| Net gas power | 23.4 MW |
| Friction and windage (0.8 %) | 0.19 MW |
| Shaft power at design | 23.2 MW |
| Electrical (η_gen 0.98) | 22.7 MW |
| Thermal efficiency (electrical / fuel heat) | 32.2 % |
| Exhaust gas temperature | 560 °C |
| Torque at rated speed | 63.3 kN·m |

Efficiency of about 32 % and 560 °C exhaust are right for a small industrial machine. If you ever
see a number outside this neighbourhood after changing a parameter, something is wrong.

## 7. Rotor: the equation the controller is fighting

   J · dω/dt = (P_gas − P_friction − P_load) / ω + τ_starter

- **Inertia J** comes from the inertia constant H = 2.5 s: H = ½ J ω² / P_design, so J = 863 kg·m².
  H is the time the rotor could supply rated power from its stored kinetic energy alone. Small H means a fast, twitchy machine.
- **Friction** P_f = 0.8 % · n² of design.
- **Load** P_load = P_elec · (1 + D(n − 1)) / η_gen when the breaker is closed. D = 1 means 1 % speed change
  gives 1 % load change (a typical grid damping figure). Breaker open: zero.
- **Starter**: torque limited to 12 kN·m and power limited to 1 MW. It must beat the compressor drag until the turbine takes over.

### Why light-off speed matters
Below about 0.44 pu the starter alone is the only thing turning the rotor. Fuel is added at 0.18 pu, and the
turbine starts to contribute. Above roughly 0.6 pu the turbine alone can accelerate, so the starter drops out.
Between these, the **acceleration limiter** controls how hard the machine is pushed.

### Low-speed fade
Below n = 0.03 the flow scaling is floored (to avoid dividing by zero). Without correction, the model keeps a constant
residual drag power as the speed approaches zero, and the rotor hits exactly zero RPM in finite time. Real gas-path power falls
roughly as n³, so below the floor the powers are multiplied by (n/0.03)³. This is a numerical fix, not physics, and is covered by
the coast-down test.

## 8. Lags: why the controller cannot be instant

| Element | Time constant | Source |
|---|---|---|
| Fuel valve | 0.20 s | actuator and positioner |
| Fuel manifold | 0.30 s | volume between valve and burner |
| Exhaust thermocouple | 3.0 s | sheath thermal mass |

The thermocouple lag is the important one: during a rapid load change the real gas temperature moves
about 3 s before the controller sees it. This is why temperature limiters on real machines use a predictive
or lead-lag compensation. The thermocouple lag is also why the controller has a *separate* hard over-temperature trip.

Sensors (week 2): there are **three independent speed channels** and **four exhaust thermocouples**. Each reading is the
true value plus its own uniform noise (speed ±1 RPM, temperature ±0.25 °C) from a fixed-seed generator, so every run is
repeatable. The four thermocouples also carry a fixed healthy offset (+3, −2, +1, −2 °C), so a healthy spread is about 5 °C.
Faults (drift, stuck, noisy, offset) are injected on the *measurement path only*: the physics never sees them.

## 9. Integration

Explicit Euler at 10 ms. The fastest time constant is 0.2 s, so dt/τ = 0.05, accurate enough for control testing.
`test_plant` checks energy conservation (kinetic energy change equals the integral of net shaft power) to within tolerance.

## 10. The controller

`controller/standin_governor.c` (with `sensor_vote.c` and `protection.c`) is the reference controller. Week 3 compiles the same
files for the STM32; only the Modbus I/O is new.

   FSR = min( FSR_start, FSR_accel, FSR_temp, FSR_gov )

The lowest request wins. Therefore:
- At rated speed the **governor** has the lowest demand (a PI on speed error; isochronous, or 4 % droop when on the grid).
- During start the **acceleration limiter** wins, holding about 2 % speed per second to limit thermal stress.
- If exhaust temperature approaches 600 °C the **temperature limiter** wins and overrides the speed request.
- Anti-windup: each PI integrator tracks the selected output by back-calculation (τ = 0.5 s). The temperature limiter tracks
  `selected + 0.10` (a "headroom"), so while the exhaust is far below its limit the limiter is invisible, and it can still
  cut fuel quickly when the limit is reached. (Tracking the selected output exactly turned it into a fuel *rate* limiter: see section 14.)
- **Load rejection preset**: when the breaker opens, fuel steps to the no-load value (0.14 FSR) immediately.

Around the min-select sit the rules of the control philosophy:
- **Protection first**: over-speed (2-of-3 voted speed ≥ 3850 RPM), over-temperature (median of four ≥ 620 °C), exhaust spread
  (max − min ≥ 80 °C for 0.5 s), sensor fault, lube-oil loss, fuel-supply loss, e-stop, flame-out, fail-to-light, comms loss.
  Protection is evaluated before, and independently of, the fuel arbitration.
- **Latched trips** with a **first-out** cause. A trip clears only on an explicit reset edge, and only when the cause has gone.
  After a reset the start request must be released and re-issued (start lock-out).
- **Start permissives**, one bit each: e-stop, lube, fuel, starter, sensors, comms, standstill, no trip.
- **Warm-up and sync permit**: after "at speed" the unit holds 5 s (WARMUP), then RUN. The breaker can be closed only while the
  controller issues the SYNC permit (RUN and speed within ±0.5 %).
- **Speed voting**: median of three; a channel is flagged if it stays more than 40 RPM from the median for 1 s, or repeats the
  exact same value for 1 s while the machine is turning. With two good channels the mean is used; if they disagree, or fewer
  than two remain, the controller trips (sensor fault).
- **Two-way watchdog**: the controller alarms after 3 failed polls (150 ms) and trips after 10 (500 ms); the plant closes the
  fuel valve and opens the breaker by itself if no command arrives for 500 ms.

## 11. What the startup and load-rejection scenarios show

**Startup**: crank, purge for 5 s, ignition, light-off, accelerate, starter cut-out at 0.6 pu, governor takes over. Rated speed is
reached at **42.6 s**, with peak exhaust 254 °C. Exhaust is low because there is no load; the turbine is only making enough power
to spin itself (no-load fuel is about 18 % of design). The ripple during acceleration is the acceleration PI limit-cycling against
the 0.5 s filter. It does not breach any criterion but is a tuning item for week 3.

**Load rejection from 50 %** (11.4 MW lost at once): speed peaks at **3647 RPM** (+4.2 %), below the 3850 trip, undershoots to
about 3430 RPM, then settles within 7 s. Without the breaker preset (`FEAT_BREAKER_PRESET` off) the peak is 3717 RPM (+6.2 %): still a pass, but with less margin to the 3850 trip. The preset buys about 70 RPM.

## 12. Known limitations (be honest about these in interviews)

- Compressor map is a simple n and n² law, with no surge line, no stall, no variable inlet guide vanes.
- Constant specific heats; no humidity or altitude correction beyond density.
- Single lumped thermocouple lag; no metal temperature and no thermal stress model.
- The load is an *isolated, constant-power* load with linear damping. There is no grid: no frequency dynamics, no synchronising dynamics. Consequence: if the load exceeds what the machine can make at its temperature limit, speed collapses instead of the output simply saturating (section 14).
- Fuel valve is linear in flow, without gas pressure effects.
- Euler integrator, which is fine at this step size but not suitable if time constants fall below about 0.1 s.

## 13. How to change it

All parameters are in `plant_params_default()` (`plant/plant.c`). After any change run `make test`.
The design-point test will tell you immediately if the new combination gives an unreasonable machine.
Limits (trip levels, thresholds) live only in `include/hil_constants.h`.

## 14. Findings from week 2 (kept because they are the interesting engineering)

1. **The temperature limiter was a hidden fuel rate limiter.** With back-calculation anti-windup tracking the *selected*
   output, the limiter's integrator always sat just above the governor's request. A 50 % to 100 % load step (exhaust peak only
   560 °C at the thermocouple) then stalled the fuel ramp, speed fell and the unit tripped on over-temperature. Fixed by parking
   the integrator 0.10 FSR above the selected value and raising the gains (kp 0.003/K, ki 0.002/K/s).
2. **A temperature limiter cannot save an overloaded isolated machine.** When the load needs more heat than the limit allows, the
   limiter cuts fuel, speed falls, airflow falls, exhaust temperature rises again and the unit trips. On a real grid the speed would
   be held by the grid and the output would just saturate. That is why the "temperature runaway" scenario checks the *protection*
   (alarm, trip, first-out, fuel closed, gas below 700 °C) and not the limiter. The limiter is checked on its own in `test_controller`
   (it takes over fuel when its limit is moved below the running temperature) and by the start-up and load-step runs.
3. **Start-up never needed the temperature or acceleration limiter.** Removing them changes nothing in the start-up scenario
   (peak 305 °C); the start ramp is already gentle. The rule that start-up really proves is the speed governor: without it the
   fuel ramp carries the rotor through rated speed and the over-speed trip fires (peak 3864 RPM).
4. **A stuck sensor is invisible to a deviation check** when it freezes on the right value, so a flat-line check is needed. It
   only works with noise on the signal (the simulator turns it off when noise is off).
