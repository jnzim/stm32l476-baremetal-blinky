# Motion Controller Architecture

## Overview

This is a single-axis BLDC servo drive running on an STM32F446RE Nucleo. The Raspberry Pi 5
generates a trajectory and streams it over SPI. The STM32 closes three nested control loops
in real time and streams telemetry back to the Pi.

There is no traditional main loop. After startup, `main()` sleeps forever in `while(1){}`.
All work happens in three interrupt handlers firing on independent schedules.

---

## Hardware Wiring

| Pi 5 Pin | Pi 5 Signal  | Wire Color | STM32 Pin | STM32 Function     |
|----------|--------------|------------|-----------|--------------------|
| Pin 19   | GPIO 10 MOSI | Yellow     | PB15      | SPI2 MOSI          |
| Pin 20   | GND          | Black      | CN7 Pin 20| GND                |
| Pin 21   | GPIO 9 MISO  | Orange     | PB14      | SPI2 MISO          |
| Pin 22   | GPIO 25      | Green      | PC13      | READY (active low) |
| Pin 23   | GPIO 11 SCK  | Red        | PB13      | SPI2 SCK           |
| Pin 24   | GPIO 8 CE0   | Brown      | PB12      | SPI2 NSS           |

---

## Execution Flow

### Startup — `main()` runs once

```
main()
  ├─ plant_init()                 — zero sim plant state
  ├─ pi_init / p_init             — zero controller integrators
  ├─ spi_init()                   — configure SPI + DMA
  ├─ tim1_init()                  — configure 20 kHz TIM1
  ├─ SysTick_Config(180000)       — configure 1 kHz SysTick
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
  │    └─ entry == true → loops_reset()    — integrators zeroed, vel_cmd/iq_cmd cleared
  │
  └─ drive_get_state() == STATE_ENABLED
       ├─ ring_pop(&s)                     — consume one trajectory sample
       ├─ update telemetry buffer
       ├─ pos_err = s.pos_cmd - plant.pos_counts
       └─ vel_cmd = p_step() + feedforward — position loop output → inner loops
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
  │    v_q_cmd = pi_step(&current_loop)    — current loop output → plant
  │
  └─ plant_step(v_q_cmd, DT_CURRENT)      — advance sim (replaces ADC/encoder on real HW)
```

TIM1 runs the **velocity and current loops** (fastest, innermost). It reads `vel_cmd`
written by SysTick and `iq_cmd` written by its own velocity sub-rate. On real hardware,
`plant_step()` is replaced by ADC reads (current) and encoder reads (velocity).

---

## Nested Time Scales

```
1ms  ┤ SysTick ── position loop ── consumes 1 trajectory sample
     │
     ├─ 200µs ── velocity loop (every 4th TIM1 tick)
     │
     └─ 50µs ─── current loop (every TIM1 tick)
                  plant_step()
```

The outer loop sets a velocity setpoint once per ms. The inner loops refine current
command 20x per ms, rejecting disturbances faster than the trajectory can see them.

---

## Interrupt Priorities

| IRQ                  | Priority | Rate   | Role                        |
|----------------------|----------|--------|-----------------------------|
| TIM1_UP_TIM10_IRQn   | 0        | 20 kHz | Current + velocity loops    |
| SysTick              | —        | 1 kHz  | Position loop, state machine|
| DMA1_Stream3_IRQn    | 2        | per pkt| SPI RX decode, ring fill    |

---

## State Machine — `drive.c`

The state machine is the only place state transitions happen. ISRs set flags; SysTick
calls `drive_update()` which reads flags and transitions.

```
         drive_request_enable()
         (called from DMA ISR)
              │
    ┌─────────▼──────────┐
    │      STATE_IDLE     │
    └─────────┬──────────┘
              │ enable_req == 1
              │ entry → loops_reset()
    ┌─────────▼──────────┐
    │   STATE_ENABLED     │◄─────────────────────┐
    └─────────┬──────────┘                       │
              │                    (fault clear — not yet implemented)
              │ fault_req == 1
    ┌─────────▼──────────┐
    │    STATE_FAULT      │
    └────────────────────┘
```

**Rules:**
- One writer: SysTick via `drive_update()`
- Flag writes are single-byte — atomic on Cortex-M4, no critical section needed
- Fault takes priority over all other transitions from any state
- `drive_is_entry()` returns true only on the first tick of a new state

---

## Ring Buffer — trajectory sample flow

```
Pi (Linux)                          STM32
──────────────────────────────────────────────────────
TrapGenerator → profile[]
  │
  └─ spi_stream_block()  ══SPI/DMA══►  DMA ISR → ring_push()
                                              │
                          PC13 READY ◄────────┤ ring.count <= 2048
                          (active low)        │
  spi_ready() == true                         │
  → refill chunk                    SysTick → ring_pop() → position loop
```

The ring buffer decouples SPI transfers (bursty, DMA-driven) from sample consumption
(steady, 1 per SysTick tick). PC13 READY signals the Pi to refill when the buffer
drops below half.

---

## SPI Protocol

- **Packet size:** 24 bytes every transaction, full duplex
- **CS:** toggles once per packet (Pi kernel SPI driver, spidev)
- **STM mode:** SPI2 slave, Mode 0, 8-bit, SSM=1 SSI=1 (software NSS)
- **DMA RX:** DMA1 Stream3 Ch0, circular, fires TCIF every 24 bytes
- **DMA TX:** DMA1 Stream4 Ch0, circular, replays telem_buf[1] continuously
- **Known issue:** DMA circular buffer can misalign on first packet if counter
  is mid-revolution when streaming starts — intermittent, under investigation

---

## File Map

| File | Owns |
|------|------|
| `main.c` | Startup, ISR handlers, peripheral init |
| `drive.c` | State machine, request flags |
| `loops.c` | Controller state, `loops_reset()` |
| `control.c` | `pi_step`, `p_step`, `pi_init`, `p_init` |
| `plant.c` | Sim plant model (replaced by HW on real drive) |
| `spi.c` | SPI + DMA, ring buffer fill, telemetry TX |
| `ringBuffer.c` | Ring buffer push/pop |
| `encoder.c` | LS7366R SPI quadrature decoder |
| `protocol.h` | `TrajSample`, `TelemetryFrame` wire formats |

---

## Real Hardware Transition (sim → real)

In sim mode `plant_step()` runs in TIM1 and provides fake position/velocity/current.
On real hardware, replace with:

- **Current feedback** — ADC reads of low-side shunt (or isolated amp on high side)
- **Velocity feedback** — LS7366R encoder count delta / dt via `encoder.c`
- **Commutation** — 3-phase PWM via TIM1 CH1/2/3 complementary outputs
- **STATE_ALIGN** — rotor alignment pulse sequence before STATE_ENABLED
  (required for AKM11E — no Hall sensors, N2 option)