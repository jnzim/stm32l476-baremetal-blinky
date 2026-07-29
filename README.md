# Motion Controller Architecture

[![Build Firmware](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml/badge.svg?branch=jz-dev)](https://github.com/jnzim/stm32-servo-drive/actions/workflows/build.yml)

## Status

**Currently in system identification / bring-up, on real hardware — not sim.**

- Cascaded current (20 kHz) and velocity (5 kHz) PI loops are closed and running on the
  actual STM32F411RE + DRV8353RS-EVM + AKM11E encoder stack — no plant simulation left in
  the active path.
- Loop gains were derived from real chirp/step-response system identification (Bode plots,
  curve-fit plant models — see `docs/PLANT_MODEL.md` and `docs/plots/`), not guessed.
- Actively debugging a velocity-loop ripple surfaced during step-response testing (see most
  recent commits).
- The position/trajectory-streaming loop and the full drive state machine (`drive.c`:
  IDLE/OPEN_LOOP/ALIGN/SERVO_ON/FAULT) are designed and scaffolded but **not yet wired into
  the running firmware** — see [State Machines](#state-machines) below. Current firmware
  runs directly from the system-ID harness (`sysid/foc_sysid.c`) for characterization; the
  Pi-streamed-trajectory mode (`RUN_MODE_CLOSED_LOOP`) is the next step once the current
  loops are validated.

## Overview

This is a single-axis BLDC servo drive running on an STM32F411RE Nucleo, being brought up
and characterized against a DRV8353RS-EVM gate driver and AKM11E encoder. The end goal is a
Raspberry Pi 5 streaming trajectories over SPI while the STM32 closes nested position/
velocity/current loops in real time — but right now the firmware runs a dedicated
system-identification harness that drives the motor directly (chirp injection, step
response) and streams telemetry back to the Pi at 20 kHz for offline analysis.

There is no traditional main loop. After startup, `main()` sleeps forever in `while(1)
{ __WFI(); }`. All work happens in independent interrupt handlers — see
[Interrupts](#interrupts) below.

---

## Hardware

### MCU
- **STM32F411RE** on Nucleo-64
- 100MHz, Cortex-M4 with FPU
- Nucleo-64 morpho pinout image (F411RE label) is the correct physical reference

### Parts to Order / BOM

| Part               | Description                                      | Qty |
|--------------------|--------------------------------------------------|-----|
| NUCLEO-F411RE      | STM32F411RE Nucleo-64 board                      | 1   |
| MAX3096CPE+        | 3.3V quad differential line receiver (encoder)  | 1   |
| 120Ω resistor      | Line termination                                 | 2   |
| 0.1µF ceramic cap  | Bypass cap                                       | 1   |
| 10µF cap           | Bulk bypass cap                                  | 1   |

---

## Pin Assignment — Complete Map

| STM32 Pin | Nucleo Header | Function          | Peripheral       |
|-----------|---------------|-------------------|------------------|
| PA0       | CN8 pin 1     | Encoder A         | TIM5 CH1         |
| PA1       | CN8 pin 2     | Encoder B         | TIM5 CH2         |
| PA4       | CN8 pin 5     | Current sense PhA | ADC1 CH4         |
| PA5       | CN10 pin 11   | SPI1 SCLK         | SPI1             |
| PA6       | CN10 pin 13   | SPI1 MISO         | SPI1             |
| PA7       | CN5 pin 4     | TIM1 CH1N         | PWM Phase A low  |
| PA8       | CN9 pin 8     | TIM1 CH1          | PWM Phase A high |
| PA9       | CN5 pin 1     | TIM1 CH2          | PWM Phase B high |
| PA10      | CN9 pin 3     | TIM1 CH3          | PWM Phase C high |
| PB0       | CN8 pin 3     | TIM1 CH2N         | PWM Phase B low  |
| PB1       | CN10 pin 24   | TIM1 CH3N         | PWM Phase C low  |
| PB5       | CN9 pin 5     | SPI1 MOSI         | SPI1             |
| PB6       | CN5 pin 3     | SPI1 CS           | SPI1             |
| PB8       | CN10 pin 3    | ENABLE            | GPIO output      |
| PB12      | CN7 pin 2     | SPI2 NSS          | SPI2             |
| PB13      | CN7 pin 4     | SPI2 SCK          | SPI2             |
| PB14      | CN7 pin 6     | SPI2 MISO         | SPI2             |
| PB15      | CN7 pin 8     | SPI2 MOSI         | SPI2             |
| PC1       | CN8 pin 6     | Current sense PhB | ADC1 CH11        |
| PC4       | TBD           | Current sense PhC | ADC1 CH14        |
| PC6       | CN10 pin 4    | nFAULT            | GPIO input       |
| PC13      | CN7 pin 23    | READY to Pi       | GPIO output      |

---

## Hardware Wiring

### Pi 5 ↔ STM32 (SPI2 + READY)

| Pi 5 Pin | Pi 5 Signal  | Wire Color | STM32 Pin | Nucleo Header | STM32 Function     |
|----------|--------------|------------|-----------|---------------|--------------------|
| Pin 19   | GPIO 10 MOSI | Yellow     | PB15      | CN7 pin 28     | SPI2 MOSI          |
| Pin 9    | GND          | Black      | —         | J1 pin 3 (EVM) | GND common         |
| Pin 21   | GPIO 9 MISO  | Orange     | PB14      | CN7 pin 26     | SPI2 MISO          |
| Pin 22   | GPIO 25      | Green      | PC13      | CN7 pin 23     | READY (active low) |
| Pin 23   | GPIO 11 SCK  | Red        | PB13      | CN7 pin 30     | SPI2 SCK           |
| Pin 26   | GPIO 7 CE1   | Brown      | PB12      | CN7 pin 2      | SPI2 NSS           |

### STM32 ↔ DRV8353RS-EVM (3-phase PWM)

| STM32 Pin | Nucleo Header | TIM1 Channel | DRV8353RS-EVM Pin | Phase       |
|-----------|---------------|--------------|-------------------|-------------|
| PA8       | CN9 pin 8     | CH1          | J2 pin 2 (INHA)   | A high-side |
| PA7       | CN5 pin 4     | CH1N         | J2 pin 4 (INLA)   | A low-side  |
| PA9       | CN5 pin 1     | CH2          | J2 pin 6 (INHB)   | B high-side |
| PB0       | CN8 pin 3     | CH2N         | J2 pin 8 (INLB)   | B low-side  |
| PA10      | CN9 pin 3     | CH3          | J2 pin 10 (INHC)  | C high-side |
| PB1       | CN10 pin 24   | CH3N         | J2 pin 12 (INLC)  | C low-side  |

All PWM pins AF1 (TIM1). Dead time handled by DRV8353RS TDRIVE VGS monitoring.

### STM32 ↔ DRV8353RS-EVM (SPI1 — gate driver config)

| STM32 Pin | Nucleo Header | SPI1 Function | DRV8353RS-EVM Pin |
|-----------|---------------|---------------|-------------------|
| PA5       | CN10 pin 11   | SCLK          | J1 pin 14 (SCLK)  |
| PA6       | CN10 pin 13   | MISO          | J1 pin 13 (SDO)   |
| PB5       | CN9 pin 5     | MOSI          | J1 pin 11 (SDI)   |
| PB6       | CN5 pin 3     | CS            | J1 pin 17 (nSCS)  |

### STM32 ↔ DRV8353RS-EVM (Control/Fault)

| STM32 Pin | Nucleo Header | Function | DRV8353RS-EVM Pin |
|-----------|---------------|----------|-------------------|
| PB8       | CN10 pin 3    | ENABLE   | J1 pin 10         |
| PC6       | CN10 pin 4    | nFAULT   | J1 pin 16         |

ENABLE must be driven high before any PWM output. nFAULT is open-drain active low —
pullup already on EVM. Currently read as a plain GPIO status bit (`drv8353_fault()`); it is
not yet wired to an EXTI interrupt, so a fault does not automatically cut PWM in firmware
today (the DRV8353RS itself still kills its gate outputs in hardware).

### STM32 ↔ DRV8353RS-EVM (Current Sense)

| STM32 Pin | Nucleo Header | ADC1 Channel | DRV8353RS-EVM Pin | Phase |
|-----------|---------------|--------------|-------------------|-------|
| PA4       | CN8 pin 5     | CH4          | J1 pin 15 (ISENA) | A     |
| PC1       | CN8 pin 6     | CH11         | J1 pin 13 (ISENB) | B     |
| PC4       | TBD           | CH14         | J1 pin 11 (ISENC) | C     |

ADC1 injected sequence is hardware-triggered off `TIM1_TRGO`, sampling at the center of the
PWM cycle for noise-free readings — see `current_feedback.c`.

### AKM11E Encoder ↔ MAX3096 ↔ STM32

The AKM11E outputs RS-422 differential encoder signals. The MAX3096CPE+ is a 3.3V
quad differential line receiver that converts to single-ended 3.3V logic for the STM32.

**MAX3096 hookup:**

```
Encoder A+  → MAX3096 pin 2  (A1)
Encoder A-  → MAX3096 pin 1  (B1)
MAX3096 pin 3 (Y1) → PA0 (CN8 pin 1) — TIM5 CH1

Encoder B+  → MAX3096 pin 6  (A2)
Encoder B-  → MAX3096 pin 7  (B2)
MAX3096 pin 5 (Y2) → PA1 (CN8 pin 2) — TIM5 CH2
```

**MAX3096 power and enable:**

```
Pin 16 VCC  → 3.3V
Pin 8  GND  → GND
Pin 4  G    → 3.3V   (enable high)
Pin 12 /G   → GND    (enable low)
```

Outputs enabled when G=high AND /G=low.

**Passive components:**

```
120Ω termination resistor across A1/B1 (pins 1-2) at MAX3096 input
120Ω termination resistor across A2/B2 (pins 6-7) at MAX3096 input
0.1µF ceramic cap — VCC to GND, close to pin 16
10µF cap          — VCC to GND, bulk bypass
```

### GND Common

| Device | Connection     |
|--------|----------------|
| Pi 5   | Pin 9          |
| EVM    | J1 pin 3 (GND) |
| PS−    | J5 GND         |

---

## Execution Flow

### Startup — `main()` runs once

```
main()
  ├─ clock_init()                  — HSI → PLL → 100MHz
  ├─ encoder_init()                — configure TIM5 quadrature decoder
  ├─ drive_init()                  — zero drive state machine (not currently driven)
  ├─ spi_init()                    — configure SPI2 + DMA, ping-pong TX telemetry buffer
  ├─ ring_init()                   — trajectory ring buffer (not currently consumed)
  ├─ (wait for user button, PC3)
  ├─ drv8353_init() / configure()  — SPI1 gate driver setup
  ├─ pwm_init()                    — TIM1 20kHz center-aligned PWM + GPIO, MOE=0
  ├─ current_feedback_init()       — ADC1 injected sequence, TIM1_TRGO hardware trigger
  ├─ drv_enable_high()             — assert DRV8353 ENABLE
  ├─ current_feedback_calibrate()  — software-start ADC offset calibration
  ├─ pwm_enable()                  — MOE=1, PWM live
  ├─ SysTick_Config(...)           — configured, but no SysTick_Handler is defined
  └─ while (1) { __WFI(); }        — main sleeps forever
```

### The only ISR driving the control loop — TIM1 at 20 kHz

```
TIM1_UP_TIM10_IRQHandler (every 50µs)
  ├─ encoder_update(tick_ms)
  └─ foc_sysid_step()              — RUN_MODE_SYSID is currently selected in config.h
       ├─ SYSID_STAGE_ALIGN  — lock rotor to theta=0 (d-axis voltage, no torque)
       ├─ SYSID_STAGE_RUN    — dispatches to the active test (config.h: SYSID_TEST),
       │                        e.g. run_cl_step(): closed current loop @ 20kHz,
       │                        closed velocity loop @ 5kHz (every 4th tick)
       └─ SYSID_STAGE_IDLE   — outputs zero, waits
```

Phase currents arrive independently via `ADC_IRQHandler`, which fires on the ADC's own
`JEOC` (injected end-of-conversion) interrupt — hardware-triggered by `TIM1_TRGO`, not
polled from the TIM1 ISR. Telemetry to the Pi is likewise independent: the TIM1 ISR writes
each sample into a ping-pong buffer (`spi_sysid_update_latest()`), and a free-running
circular TX DMA continuously shifts the latest complete frame out over SPI2 — the Pi always
reads whatever was most recently written, without a request/response handshake.

---

## Nested Time Scales (currently active)

```
50µs  ── current loop + PWM update, every TIM1 tick (20 kHz)
200µs ── velocity loop, every 4th TIM1 tick (5 kHz)
```

The position loop (1 kHz, trajectory-sample-driven) is designed but not active — see
[Status](#status).

---

## Interrupts

| IRQ                  | Priority | Rate      | Role                                          |
|----------------------|----------|-----------|------------------------------------------------|
| `TIM1_UP_TIM10_IRQn` | 1        | 20 kHz    | Encoder update, sysid stage machine, current + velocity loops |
| `ADC_IRQn`           | 3        | 20 kHz    | Reads phase currents on injected-conversion complete |
| `DMA1_Stream3_IRQn`  | 2        | per pkt   | SPI2 RX-complete from Pi (currently counts only) |
| `EXTI15_10_IRQn`     | 0        | per pkt   | SPI2 CS edge — rearms TX DMA for next telemetry frame |

`SysTick` is configured at boot but has no handler defined, so nothing currently runs at
1 kHz.

---

## State Machines

There are two state machines in this codebase — only one of them is actually running.

### Active — sysid stage sequence (`sysid/foc_sysid.c`)

This is what the firmware runs today, driven directly from the TIM1 ISR:

```
SYSID_STAGE_ALIGN → SYSID_STAGE_RUN → SYSID_STAGE_IDLE
     (lock rotor)     (dispatches to the      (outputs zero,
                        active test, e.g.       waits)
                        run_cl_step())
```

The active test is selected at compile time via `SYSID_TEST` in `config.h` — options
include open-loop chirp, closed-current-loop step, closed-velocity-loop chirp/step, and a
constant-iq ripple-debug mode (currently selected, for the velocity ripple investigation).

### Designed, not yet wired up — `drive.c`

A full drive state machine exists for the eventual Pi-trajectory-streaming mode, but
`drive_sm_run()` is never called anywhere in the firmware (it was written to run from
`SysTick`, which has no handler). It's left in place as the target architecture for
`RUN_MODE_CLOSED_LOOP`:

```
         open_loop_req                    servo_on_req
              │                                │
    ┌─────────▼──────────┐                     │
    │      STATE_IDLE     │◄────────────────────┼──────────────┐
    └──┬──────────────────┘                     │              │
       │                            ┌───────────▼──────────┐   │
       │                            │    STATE_SERVO_ON     │───┘
    ┌──▼──────────────────┐         │  (closed-loop FOC)   │ ring empty
    │   STATE_OPEN_LOOP   │         └───────────────────────┘ pwm_disable()
    │  (no feedback)      │
    └──┬──────────────────┘
       │ stop_req / pwm_disable()
       ▼
    ┌─────────────────────┐
    │    STATE_FAULT       │  ← fault_req from any state
    │  (latched, PWM off)  │
    └─────────────────────┘
```

(A second, newer prototype FSM, `servo_sm.c`, also exists but isn't part of the build at
all — it's excluded from `CMakeLists.txt` and doesn't currently compile.)

---

## PWM — `pwm.c`

- `pwm_init()` — TIM1 + GPIO, MOE=0
- `pwm_enable()` — set MOE
- `pwm_disable()` — clear MOE
- `pwm_apply_dq(v_d, v_q, theta)` — inverse Park + Clarke → CCR1/2/3
- V_BUS = 12V

---

## SPI Link (current)

The active telemetry path is a fixed-format, continuous sample stream — not the
opcode-driven protocol described below (that protocol exists in `protocol.h` but is only
referenced by the currently-inactive `drive.c`/ring-buffer path).

- **Packet size:** 32 bytes, full duplex (`SysIdSample` struct, size-checked at compile time)
- **CS:** Pi GPIO7 (pin 26), manual, toggles per packet — used to rearm TX DMA, not to gate data
- **STM mode:** SPI2 slave, `CPHA=1`, RX/TX DMA, no software framing/opcodes
- Every 20 kHz tick, the TIM1 ISR writes a fresh sample (position, id/iq, vd/vq, theta,
  phase currents, stage flags) into whichever half of a ping-pong buffer isn't currently
  being DMA'd out; the Pi always reads the latest complete frame on its next transaction.

### Opcode protocol (designed, not currently consumed by the active SPI path)

| Opcode     | Value | Description                       |
|------------|-------|-----------------------------------|
| NOP        | 0x00  | No-op                             |
| BLOCK_HDR  | 0x03  | Start trajectory → STATE_SERVO_ON |
| DATA       | 0x04  | Trajectory sample packet          |
| READY_ACK  | 0x05  | Pi acknowledges READY signal      |
| TELEM_REQ  | 0x06  | Pi requests telemetry frame       |
| OPEN_LOOP  | 0x07  | Start open-loop → STATE_OPEN_LOOP |
| STOP       | 0x08  | Stop → STATE_IDLE                 |

---

## File Map

| File                      | Owns                                                          |
|---------------------------|----------------------------------------------------------------|
| `main.c`                  | Startup, TIM1 ISR entry point                                 |
| `clock.c`                 | HSI → PLL → 100MHz                                            |
| `tim1.c`                  | TIM1 timebase config (center-aligned PWM + update IRQ)        |
| `pwm.c`                   | GPIO/AF setup, enable/disable, `pwm_apply_dq`                 |
| `current_feedback.c`      | ADC1 injected sequence, `ADC_IRQHandler`, calibration         |
| `encoder.c`               | TIM5 quadrature decode, velocity filter                       |
| `drv8353.c`               | SPI1 gate driver config/status                                |
| `spi.c`                   | SPI2 + DMA telemetry link to the Pi                           |
| `sysid/foc_sysid.c`       | **Active** — sysid stage machine, all current test modes      |
| `sysid/foc_trajectory.c`  | Trajectory-following step (for `RUN_MODE_CLOSED_LOOP`, unused today) |
| `loops.c` / `control.c`   | PI controller state and step functions shared by both modes  |
| `drive.c`                 | Designed drive FSM — not currently called (see State Machines) |
| `servo_sm.c`              | Prototype FSM — not in the build                              |
| `ringBuffer.c`, `plant.c` | Ring buffer + simulated plant, used by the not-yet-active trajectory path |
| `protocol.h`              | `TrajSample`, `TelemetryFrame`, opcodes, drive states (design surface for `RUN_MODE_CLOSED_LOOP`) |

---

## Bringup Sequence

1. Rotor lock / alignment — **done**
2. Open-loop electrical spin / encoder polarity check — **done**
3. Closed-loop current validation — **done**, gains from measured system ID
4. Closed-loop velocity — **done**, currently debugging a ripple in step response
5. Closed-loop position (Pi-streamed trajectories) — **not started**

<!-- Images pending: MCU/board photos, pinout reference to be added by user -->
