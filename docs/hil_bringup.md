# Hardware bring-up: BeagleBone Black (plant) + STM32 Nucleo-F446RE (controller)

Status: the software side is proven (see `results_hil/`: all scenarios replayed in real time over a virtual serial cable,
with `ctrl_node` standing in for the STM32). The STM32 firmware files in `stm32/` pass a syntax check against a stub HAL
but have **not** been compiled for the F446RE or run on hardware yet. Bring-up below finds out.

## What runs where

| Device | Program | Job |
|---|---|---|
| BeagleBone Black | `build/plant_node` | plant model, 10 ms steps, Modbus RTU **slave** on UART1 (`/dev/ttyS1`), local command port 5001 |
| STM32 Nucleo-F446RE | `stm32/` + `common/` + `controller/` | controller, 50 ms cycle, Modbus RTU **master** on USART1 |
| BBB (or your PC via an SSH tunnel) | `tools/hil_runner.py` | operator and test supervisor: plays a scenario, judges the result |

## Wiring (3 wires, both sides are 3.3 V logic)

| STM32 Nucleo-F446RE | | BeagleBone Black |
|---|---|---|
| PA9  (USART1 TX, Arduino header D8)  | to | P9_26 (UART1 RXD) |
| PA10 (USART1 RX, Arduino header D2)  | to | P9_24 (UART1 TXD) |
| GND                                  | to | P9_1  (GND) |

TX goes to RX and RX to TX. Do not connect 5 V anywhere. Keep the wires short (under 30 cm).
Link settings: 115200 baud, 8 data bits, even parity, 1 stop bit (8E1), slave address 1.

## Step 1: BeagleBone

    ssh debian@192.168.7.2
    sudo apt update && sudo apt install -y build-essential git python3-numpy python3-matplotlib
    git clone https://github.com/jdram82/gt-hil.git && cd gt-hil
    make build/plant_node
    sudo config-pin P9_24 uart && sudo config-pin P9_26 uart
    ls -l /dev/ttyS1

If `config-pin` is missing, enable the UART1 overlay in `/boot/uEnv.txt` (`uboot_overlay_addr4=/lib/firmware/BB-UART1-00A0.dtbo`) and reboot.

Loopback check (jumper P9_24 to P9_26, no STM32 yet):

    sudo stty -F /dev/ttyS1 115200 raw -echo parenb -parodd cs8
    (sudo timeout 3 cat /dev/ttyS1 &) ; sleep 0.5 ; echo hello | sudo tee /dev/ttyS1 >/dev/null ; sleep 3

You should see `hello`. Remove the jumper afterwards.

## Step 2: STM32 firmware (STM32CubeIDE)

1. New STM32 project, board selector **NUCLEO-F446RE**, initialise peripherals to defaults (this gives USART2 on the ST-LINK
   virtual COM port and the LD2 LED).
2. Open the `.ioc`: **USART1** mode Asynchronous, baud 115200, **Word Length = 9 Bits (including Parity)**, **Parity = Even**,
   Stop Bits = 1 (this is 8E1 on the wire). Pins PA9 and PA10. Generate code.
3. Copy these folders from the repo into the project root: `controller/`, `common/`, `include/`, `stm32/`.
   Project Properties > C/C++ General > Paths and Symbols > Includes: add `${ProjDirPath}/stm32`.
4. Project Properties > C/C++ Build > Settings > MCU Settings: tick **Use float with printf from newlib-nano** (otherwise the
   status line shows no numbers).
5. In `Core/Src/main.c`, inside the generated USER CODE markers:

        /* USER CODE BEGIN Includes */
        #include "hil_app.h"
        /* USER CODE BEGIN 2 */
        hil_app_init();
        /* USER CODE BEGIN 3 (inside while (1)) */
        hil_app_loop();

6. Build and run. Open the ST-LINK COM port in a terminal at 115200 8N1. You should see `gt-hil controller up` and, every
   2 s, a status line: `cyc 40 fail 40 ...` until the plant is running (all polls fail with no plant), then `fail` stops growing.

Note: the F446RE FPU is single precision, so the `double` maths runs in software. The status line prints the cycle time
(`cycle ... us`); it has to stay far below 50 ms. That number is the first thing to report back.

## Step 3: run

On the BeagleBone (leave it running):

    cd ~/gt-hil && sudo ./build/plant_node --port /dev/ttyS1 --rt

`--rt` asks for real-time scheduling (`SCHED_FIFO`); it prints a warning and carries on if it is refused.
On the STM32 terminal the `fail` counter should stop increasing within a second.

Then run a scenario, on the BeagleBone:

    python3 tools/hil_runner.py --cmd-port 5001 scenarios/startup_to_rated.json

or from your PC (WSL) through an SSH tunnel:

    ssh -N -L 5001:127.0.0.1:5001 debian@192.168.7.2 &
    python3 tools/hil_runner.py --cmd-port 5001 scenarios/*.json

Each scenario runs in real time (2 to 4 minutes, including the pre-roll to bring the unit up). Results go to `results_hil/`,
each report with a `timing` block: wake-up lateness of the 10 ms plant loop, Modbus poll period, response latency.

## Software rehearsal (no hardware)

    make
    python3 tools/hil_runner.py --spawn --cmd-port 5201 scenarios/startup_to_rated.json

`--spawn` starts a virtual serial cable (`tools/vserial.py`), `plant_node` and `ctrl_node` (the same `hil_master.c` as the
STM32 runs, on Linux) and tears them down afterwards.

## What the real-time run can and cannot check

- The plant sees only what the controller wrote. During a link loss the controller cannot write, so its alarm and trip are
  invisible until the link returns. The communication-loss scenario therefore has HIL-specific criteria (see the `hil` block in
  `scenarios/communication_loss.json`).
- Negative tests (remove one rule, scenario must fail) are not run in real time; they need controller builds with a rule
  disabled and stay in the software tier (`make scenarios`).

## If something fails

| Symptom | Likely cause |
|---|---|
| STM32 `fail` counter keeps rising | TX/RX swapped, no common GND, wrong baud or parity (9-bit even), `plant_node` not running, wrong port |
| `timeouts` rising, `crc` zero | nothing coming back: wiring or pinmux (`config-pin`) |
| `crc` rising | parity or baud mismatch, noisy long wires |
| `plant_node: SCHED_FIFO` warning | run with `sudo` |
| Runner: "controller did not return to IDLE" | STM32 not running or not talking to the plant |
