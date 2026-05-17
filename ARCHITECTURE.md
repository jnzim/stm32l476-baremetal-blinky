# Motion Controller Architecture

## Overview

This is a single-axis BLDC servo drive running on an STM32F446RE Nucleo. The Raspberry Pi 5
generates a trajectory and streams it over SPI. The STM32 closes three nested control loops
in real time and streams telemetry back to the Pi.

There is no traditional main loop. After startup, `main()` sleeps forever in `while(1){}`.
All work happens in three interrupt handlers firing on independent schedules.

---

## Hardware Wiring

### Pi 5 ↔ STM32 (SPI + READY)

| Pi 5 Pin | Pi 5 Signal  | Wire Color | STM32 Pin  | STM32 Function     |
|----------|--------------|------------|------------|--------------------|
| Pin 19   | GPIO 10 MOSI | Yellow     | PB15       | SPI2 MOSI          |
| Pin 20   | GND          | Black      | CN7 Pin 20 | GND                |
| Pin 21   | GPIO 9 MISO  | Orange     | PB14       | SPI2 MISO          |
| Pin 22   | GPIO 25      | Green      | PC13       | READY (active low) |
| Pin 23   | GPIO 11 SCK  | Red        | PB13       | SPI2 SCK           |
| Pin 24   | GPIO 8 CE0   | Brown      | PB12       | SPI2 NSS           |
a
### STM32 ↔ DRV8353RS-EVM (3-phase PWM)

| STM32 Pin | TIM1 Channel | DRV8353RS-EVM | Phase        |
|-----------|--------------|----------------|--------------|
| PA8       | CH1          | INHA           | A high-side  |
| PA7       | CH1N         | INLA           | A low-side   |
| PA9       | CH2          | INHB           | B high-side  |
| PB0       | CH2N         | INLB           | B low-side   |
| PA10      | CH3          | INHC           | C high-side  |
| PB1       | CH3N         | INLC           | C low-side   |

All PWM pins configured AF1 (TIM1). Dead time handled automatically by DRV8353RS
TDRIVE VGS monitoring — not configured in TIM1 BDTR.

---

## Execution Flow

### Startup — `main()` runs once

```
main()
  ├─ clock_init()                 — HSI → PLL → 180MHz
  ├─ plant_init()                 — zero sim plant state
  ├─ pi_init / p_init             — zero controller integrators
  ├─ spi_init()                   — configure SPI2 + DMA
  ├─ encoder_init()               — configure TIM5 quadrature decoder
  ├─ pwm_init()                   — configure TIM1 20kHz center-aligned PWM + GPIO
  ├─ SysTick_Config(180000)       — configure 1kHz SysTick
  └─ while(1) {}                  — main sleeps forever, does nothing
```

After this point main() never does anything again. All real work is interrupt-driven.

---

### Phase 1 — Pi sends first SPI block → DMA ISR fires

```
DMA ISR
  ├─ ring_push(samples)           — fill ring buffer with trajectory samples
  └─ drive_request_enable()       — set enable_req = 1 (flag only, no transition yet)
```

The DMA ISR never transitions the state machine directly. It sets a flag and returns.
State transitions only happen in SysTick.

---

### Phase 2 — Next 1ms tick → SysTick fires

```
SysTick_Handler (1 kHz)
  ├─ drive_update()
  │    ├─ sees enable_req == 1
  │    ├─ STATE_IDLE → STATE_ENABLED
  │    │    ├─ loops_reset()       — integrators zeroed, vel_cmd/iq_cmd cleared
  │    │    ├─ plant_init()        — zero sim plant state
  │    │    └─ pwm_enable()        — set TIM1 MOE, outputs reach gate driver
  │    │
  │    └─ STATE_ENABLED, ring empty → STATE_IDLE
  │         └─ pwm_disable()      — clear MOE, all outputs low
  │
  └─ drive_get_state() == STATE_ENABLED
       ├─ ring_pop(&s)             — consume one trajectory sample
       ├─ update telemetry buffer
       ├─ pos_err = s.pos_cmd - plant.pos_counts
       └─ vel_cmd = p_step()      — position loop output → inner loops
```

SysTick runs the **position loop** (slowest, outermost). It produces `vel_cmd` which
the TIM1 ISR reads on every one of its 20 ticks before the next SysTick fires.

---

### Phase 3 — Every 50µs → TIM1 ISR fires (20 kHz)

```
TIM1_UP_TIM10_IRQHandler (20 kHz)
  ├─ every 4th tick (5 kHz):
  │    vel_err = vel_cmd - plant.vel
  │    iq_cmd  = pi_step(&velocity_loop)   — velocity loop output → current loop
  │
  ├─ every tick (20 kHz):
  │    i_err   = iq_cmd - plant.i_q
  │    v_q_cmd = pi_step(&current_loop)    — current loop output → plant/PWM
  │
  └─ plant_step(v_q_cmd, DT_CURRENT)      — sim only (replaced by ADC/encoder on HW)
```

TIM1 runs the **velocity and current loops** (fastest, innermost). On real hardware,
`plant_step()` is replaced by ADC current reads and encoder velocity reads, and
`v_q_cmd` drives the PWM duty cycle via `pwm_set_duty()`.

---

## Nested Time Scales

```
1ms  ┤ SysTick ── position loop ── consumes 1 trajectory sample
     │
     ├─ 200µs ── velocity loop (every 4th TIM1 tick)
     │
     └─ 50µs ─── current loop (every TIM1 tick)
                  plant_step() / pwm_set_duty()
```

---

## Interrupt Priorities

| IRQ                  | Priority | Rate   | Role                          |
|----------------------|----------|--------|-------------------------------|
| TIM1_UP_TIM10_IRQn   | 1        | 20 kHz | Current + velocity loops, PWM |
| DMA1_Stream3_IRQn    | 2        | per pkt| SPI RX decode, ring fill      |
| SysTick              | 15       | 1 kHz  | Position loop, state machine  |

---

## State Machine — `drive.c`

The state machine is the only place state transitions happen. ISRs set flags; SysTick
calls `drive_update()` which reads flags and transitions.

```
         drive_request_enable()
         (called from DMA ISR on BLOCK_HDR)
              │
    ┌─────────▼──────────┐
    │      STATE_IDLE     │◄────────────────────┐
    └─────────┬──────────┘                      │
              │ enable_req == 1                  │
              │ loops_reset()                    │
              │ plant_init()                     │ ring empty
              │ pwm_enable()                     │ pwm_disable()
    ┌─────────▼──────────┐                      │
    │   STATE_ENABLED     │──────────────────────┘
    └─────────┬──────────┘
              │ fault_req == 1
              │ pwm_disable()
    ┌─────────▼──────────┐
    │    STATE_FAULT      │
    └────────────────────┘
              (fault clear not yet implemented)
```

**Rules:**
- One writer: SysTick via `drive_update()`
- Flag writes are single-byte — atomic on Cortex-M4, no critical section needed
- Fault takes priority over all other transitions from any state
- `drive_is_entry()` returns true only on the first tick of a new state

---

## PWM — `pwm.c`

TIM1 center-aligned complementary PWM at 20kHz. 6 outputs driving DRV8353RS-EVM.

- `pwm_init()` — configure TIM1 + GPIO, MOE=0 (outputs disabled)
- `pwm_enable()` — set MOE, outputs reach gate driver
- `pwm_disable()` — clear MOE, all inputs go low (Hi-Z on DRV)
- `pwm_set_duty(phase, duty)` — set CCR, 0–4499, center-aligned

Dead time is automatic — DRV8353RS TDRIVE monitors VGS and prevents shoot-through.

---

## Ring Buffer — trajectory sample flow

```
Pi (Linux)                          STM32
──────────────────────────────────────────────────────
TrapGenerator → profile[]
  │
  └─ spi_stream_block()  ══SPI/DMA══►  DMA ISR → ring_push()
       send_header=true  (new move)              ring_reset() + drive_request_enable()
       send_header=false (refill)                DATA only, no ring reset
                          PC13 READY ◄────────┤ ring.count <= 2048
                          (active low)
  spi_ready() == true
  → refill chunk                    SysTick → ring_pop() → position loop
```

Refills send DATA packets only — no BLOCK_HDR — to avoid ring_reset() and telem
corruption during an active move.

---

## SPI Protocol

- **Packet size:** 32 bytes every transaction, full duplex
- **CS:** manual via GPIO7 (Pi pin 26), toggles once per packet
- **STM mode:** SPI2 slave, Mode 0, 8-bit, SSM=1
- **DMA RX:** DMA1 Stream3 Ch0, circular, fires TCIF every 32 bytes
- **DMA TX:** DMA1 Stream4 Ch0, circular, replays telem_buf[1] continuously
- **Telem frame:** 32 bytes — pos_cmd, pos_fbk, vel_cmd, vel_fbk, timestamp_ms,
  drive_state, fault_flags, samples_consumed, pos_err, i_q_fbk, v_q_cmd

---

## File Map

| File | Owns |
|------|------|
| `main.c` | Startup sequence, ISR handlers |
| `clock.c` | HSI → PLL → 180MHz system clock |
| `pwm.c` | TIM1 PWM init, enable/disable, duty cycle |
| `drive.c` | State machine, request flags, pwm_enable/disable calls |
| `loops.c` | Controller state, `loops_reset()` |
| `control.c` | `pi_step`, `p_step`, `pi_init`, `p_init` |
| `plant.c` | Sim plant model (replaced by HW on real drive) |
| `spi.c` | SPI2 + DMA, ring buffer fill, telemetry TX |
| `ringBuffer.c` | Ring buffer push/pop |
| `encoder.c` | TIM5 quadrature decoder |
| `protocol.h` | `TrajSample`, `TelemetryFrame` wire formats |

---

## Real Hardware Transition (sim → real)

In sim mode `plant_step()` runs in TIM1 and provides fake position/velocity/current.
On real hardware:

- **Current feedback** — ADC1 reads of DRV8353RS low-side shunt amplifiers
  (PA4=PhA, PC1=PhB, PC4=PhC)
- **Velocity feedback** — TIM5 encoder count delta / dt via `encoder.c`
- **Commutation** — `pwm_set_duty()` replaces `plant_step()` in TIM1 ISR
- **STATE_ALIGN** — rotor alignment pulse before STATE_ENABLED
  (required for AKM11E — no Hall sensors, N2 option)
- **Fault handling** — nFAULT from DRV8353RS wired to STM GPIO,
  calls `drive_request_fault()` on assert
