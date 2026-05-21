# Motion Controller Architecture

## Overview

This is a single-axis BLDC servo drive running on an STM32F446RE Nucleo. The Raspberry Pi 5
generates a trajectory and streams it over SPI. The STM32 closes three nested control loops
in real time and streams telemetry back to the Pi.

There is no traditional main loop. After startup, `main()` sleeps forever in `while(1){}`.
All work happens in three interrupt handlers firing on independent schedules.

---

## Hardware

### MCU
- **STM32F446RE** on Nucleo-64 (same physical board layout as F411RE Nucleo-64)
- 180MHz, Cortex-M4 with FPU
- Nucleo-64 morpho pinout image (F411RE label) is the correct physical reference

### Parts to Order / BOM

| Part               | Description                                      | Qty |
|--------------------|--------------------------------------------------|-----|
| NUCLEO-F446RE      | STM32F446RE Nucleo-64 board                      | 1   |
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
pullup already on EVM. Configure PC6 as input with EXTI on falling edge →
`drive_request_fault()`.

### STM32 ↔ DRV8353RS-EVM (Current Sense)

| STM32 Pin | Nucleo Header | ADC1 Channel | DRV8353RS-EVM Pin | Phase |
|-----------|---------------|--------------|-------------------|-------|
| PA4       | CN8 pin 5     | CH4          | J1 pin 15 (ISENA) | A     |
| PC1       | CN8 pin 6     | CH11         | J1 pin 13 (ISENB) | B     |
| PC4       | TBD           | CH14         | J1 pin 11 (ISENC) | C     |

ADC1 triggered by TIM1 center-aligned PWM peak for noise-free sampling.

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
  ├─ clock_init()                 — HSI → PLL → 180MHz
  ├─ plant_init()                 — zero sim plant state
  ├─ pi_init / p_init             — zero controller integrators
  ├─ spi_init()                   — configure SPI2 + DMA
  ├─ encoder_init()               — configure TIM5 quadrature decoder
  ├─ pwm_init()                   — configure TIM1 20kHz center-aligned PWM + GPIO
  ├─ SysTick_Config(180000)       — configure 1kHz SysTick
  └─ while(1) {}                  — main sleeps forever
```

---

### Phase 1 — Pi sends first SPI block → DMA ISR fires

```
DMA ISR
  ├─ ring_push(samples)           — fill ring buffer with trajectory samples
  └─ drive_request_servo_on()     — set servo_on_req = 1 (flag only)
```

---

### Phase 2 — Next 1ms tick → SysTick fires

```
SysTick_Handler (1 kHz)
  ├─ drive_update()
  │    ├─ STATE_IDLE → STATE_SERVO_ON  (servo_on_req)
  │    │    └─ on entry: loops_reset(), plant_init(), pwm_enable()
  │    ├─ STATE_IDLE → STATE_OPEN_LOOP (open_loop_req)
  │    │    └─ on entry: ol_theta=0, pwm_enable()
  │    └─ STATE_SERVO_ON, ring empty → STATE_IDLE, pwm_disable()
  │
  └─ STATE_SERVO_ON:
       ├─ ring_pop(&s)
       ├─ update telemetry
       ├─ pos_err = s.pos_cmd - plant.pos_counts
       └─ vel_cmd = p_step()
```

---

### Phase 3 — Every 50µs → TIM1 ISR fires (20 kHz)

```
TIM1_UP_TIM10_IRQHandler (20 kHz)
  ├─ every 4th tick (5 kHz):
  │    iq_cmd = pi_step(&velocity_loop, vel_cmd - plant.vel)
  ├─ every tick (20 kHz):
  │    v_q_cmd = pi_step(&current_loop, iq_cmd - plant.i_q)
  └─ plant_step(v_q_cmd)          — sim; replaced by pwm_apply_vq() on HW
```

---

## Nested Time Scales

```
1ms  ┤ SysTick ── position loop ── consumes 1 trajectory sample
     ├─ 200µs ── velocity loop (every 4th TIM1 tick)
     └─ 50µs ─── current loop + PWM update (every TIM1 tick)
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

```
         open_loop_req                    servo_on_req
         (SPI2_OP_OPEN_LOOP)              (SPI2_OP_BLOCK_HDR)
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

---

## PWM — `pwm.c`

- `pwm_init()` — TIM1 + GPIO, MOE=0
- `pwm_enable()` — set MOE
- `pwm_disable()` — clear MOE
- `pwm_apply_vq(v_q, v_d, theta)` — inverse Park + Clarke → CCR1/2/3
- V_BUS = 24V

---

## SPI Protocol

- **Packet size:** 32 bytes, full duplex
- **CS:** Pi GPIO7 (pin 26), manual, toggles per packet
- **STM mode:** SPI2 slave, Mode 0, 8-bit, SSM=1

### Opcodes

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

| File           | Owns                                              |
|----------------|---------------------------------------------------|
| `main.c`       | Startup, ISR handlers                             |
| `clock.c`      | HSI → PLL → 180MHz                                |
| `pwm.c`        | TIM1 PWM, enable/disable, pwm_apply_vq            |
| `drive.c`      | State machine, request flags                      |
| `loops.c`      | Controller state, loops_reset()                   |
| `control.c`    | pi_step, p_step, pi_init, p_init, open_loop_step  |
| `plant.c`      | Sim plant (replaced by HW)                        |
| `spi.c`        | SPI2 + DMA, ring fill, telemetry TX               |
| `ringBuffer.c` | Ring buffer push/pop                              |
| `encoder.c`    | TIM5 quadrature decoder                           |
| `protocol.h`   | TrajSample, TelemetryFrame, opcodes, drive states |

---

## Bringup Sequence

1. Rotor lock / alignment (STATE_ALIGN)
2. Open-loop electrical spin / encoder polarity check (STATE_OPEN_LOOP)
3. Closed-loop current validation
4. Closed-loop velocity
5. Closed-loop position (STATE_SERVO_ON)

---

## Real Hardware Transition (sim → real)

- **Current feedback** — ADC1 reads DRV8353RS shunt amps (PA4/PC1/PC4)
- **Velocity feedback** — TIM5 encoder count delta/dt via MAX3096 line receiver
- **Commutation** — `pwm_apply_vq()` replaces `plant_step()` in TIM1 ISR
- **Alignment** — STATE_ALIGN locks rotor to theta=0 (AKM11E has no Hall sensors)
- **Fault** — PC6 nFAULT EXTI falling edge → `drive_request_fault()`
