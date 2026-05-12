# stm32-servo-drive

Bare metal STM32F446RE servo drive firmware. Closes FOC current, velocity, and position control loops from scratch — no HAL, no RTOS, no commercial drive.

Companion trajectory streamer: [servo-trajectory-streamer](https://github.com/jnzim/servo-trajectory-streamer)

## What it does

- Receives trapezoidal trajectory samples from Raspberry Pi over SPI2 + DMA
- Buffers 4096 samples in a ring buffer, asserts PC13 READY for refill at 2048
- Runs three rate-divided control loops in a single hard-RT ISR hierarchy
- Simulated plant model (2nd order motor, Euler integration) for SW validation before hardware
- Streams 24-byte telemetry frames back to Pi on every SPI transaction
- Telemetry includes position command, position feedback, velocity feedback, samples consumed

## Hardware

- STM32F446RE Nucleo-64
- DRV8353RS-EVM gate driver (15A/20A peak, 9-95V, integrated 3-shunt current sensing)
- Kollmorgen AKM11E-ANCN2-00 BLDC servo motor
- 2048 CPR quadrature encoder (TIM5, 4x decode = 8192 counts/rev)
- Raspberry Pi 4/5 as trajectory supervisor via SPI2

## Pin mapping

| Peripheral | Pins | Notes |
|---|---|---|
| TIM1 PWM 3-phase | PA8, PA7, PA9, PB0, PA10, PB1 | Center-aligned |
| TIM5 encoder | PA0, PA1 | 32-bit counter |
| ADC1 current sensing | PA4, PC1, PC4 | Triggered by TIM1 |
| SPI1 DRV8353RS | PA5, PA6, PB5, PB6 | Gate driver config |
| SPI2 Raspberry Pi | PB12, PB13, PB14, PB15 | Trajectory + telem |
| PC13 READY | PC13 | Active low refill signal |

## Architecture

    Pi -> SPI2 -> ring buffer (4096 samples)
                       |
                  1kHz SysTick
                  position loop (P)
                       |
                  velocity setpoint
                       |
                  5kHz TIM1 /4
                  velocity loop (PI)
                       |
                  current setpoint
                       |
                  20kHz TIM1
                  current loop (PI)
                       |
                  plant model / PWM

## Build

    cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
    cmake --build build -j
    st-flash --reset write build/fw.bin 0x08000000

## Status

- SPI2 slave DMA circular: proven — 5007 packets, 0 errors
- Ring buffer + READY refill: proven
- TIM1 20kHz ISR: running
- Plant model 2nd order sim: running, gain tuning in progress
- FOC Park/Clarke transforms: not yet
- Real PWM output: not yet
- Motor spin: not yet


## Simulation Results

![Position and velocity tracking](sim_results.png)

## Project goal

Demonstrate full-stack servo drive competency — trajectory generation, SPI comms, bare metal STM32, and closed-loop FOC from scratch. Target applications: semiconductor capital equipment (KLA, ASML, Aerotech, Lam Research).


