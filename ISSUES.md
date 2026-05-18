# TODO — Custom Servo Drive

## Current State (jz-pwm branch)
- Simulation loops working — position, velocity, current
- 3-phase complementary PWM 20kHz verified on scope
- pwm_enable/disable wired into drive state machine
- FOC output stage stubbed — volts_to_duty, pwm_apply_vq (inverse Park + Clarke)
- FPU enabled in main() — SCB->CPACR
- drive_align_rotor, drive_open_loop_step in drive.c
- config.h created — V_BUS, ALIGN_VOLTAGE, ALIGN_TIME_MS, M_PI, MOTOR_POLE_PAIRS

---

## Picking Up Here — Hardware Bringup

### 1. Wire up DRV8353RS-EVM
- [ ] PWM: PA8/PA7, PA9/PB0, PA10/PB1 → J2 INHA/INLA/INHB/INLB/INHC/INLC
- [ ] Power: J1 pin 2 (3.3V), pin 3 (GND) from Nucleo
- [ ] nFAULT: J1 pin 16 → STM32 GPIO (TBD) → drive_request_fault()
- [ ] Current sense: J1 pin 15/13/11 (ISENA/B/C) → PA4/PC1/PC4
- [ ] Motor phases: J6 MOTA/MOTB/MOTC → AKM11E
- [ ] Bus power: bench supply 12V, 1A current limit → J5

### 2. Encoder
- [ ] Verify power requirement for AKM11E N2 encoder (likely 5V)
- [ ] Find or make encoder cable (blocked)
- [ ] Wire level shifter (5V encoder → 3.3V STM)
- [ ] Verify TIM5 quadrature counts by hand-rotating shaft
- [ ] Add encoder_zero() to firmware
- [ ] Add encoder position to telem for continuous monitoring

### 3. ADC current sensing
- [ ] Write adc_init() — ADC1 triggered by TIM1 center-aligned peak
- [ ] Read ISENA/B/C with no motor connected — verify zero baseline
- [ ] Convert raw counts to amps using DRV8353RS shunt amp gain

### 4. Open loop spin
- [ ] Connect motor (need correct cable or cut/splice)
- [ ] Call drive_open_loop_step() from test state
- [ ] Start at 0.5V, slow angle increment
- [ ] Verify encoder counts change — swap two phases if wrong direction

### 5. Close current loop
- [ ] Replace plant.i_q with ADC feedback
- [ ] Replace plant_step() with pwm_apply_vq() in TIM1 ISR
- [ ] Tune current loop Kp at ±0.5A limit

### 6. Close velocity + position loops
- [ ] Replace plant.vel with encoder velocity
- [ ] Replace plant.pos_counts with encoder position
- [ ] Tune velocity and position loops
- [ ] Run trapezoidal profile, verify telem plots

---

## Known Issues

### Control Loop
- **No feedforward** — position loop will lag on fast profiles. Add vel_cmd feedforward from trajectory sample before hardware bring-up.
- **PLANT_B too high** — set to 0.001 to prevent sim instability. Not realistic for AKM11E. Retune once hardware available.
- **Velocity loop units** — vel_cmd in rad/s internally, vel_fbk logged in counts/s. Standardize before hardware.
- **STATE_ALIGN no timer** — needs tick counter before transitioning to STATE_ENABLED.
- **Velocity loop clamp ±5A** — too loose for no-load hardware bringup. Tighten before first spin.

### Telemetry / Pi
- **telem t0 offset** — Pi starts logging ~100 samples late. t0 alignment approximate.
- **vel_fbk resolution** — int16_t limits range on high speed moves. May need scaling.
- **No move timeout** — if STM stops consuming, Pi loops forever. Need watchdog.
- **No fault detection on Pi** — drive_state and fault_flags not checked. Should abort on DRIVE_FAULT.
- **Continuous monitoring not implemented** — Pi only polls during moves.

### Hardware
- **nFAULT not wired** — DRV8353RS fault pin unmonitored. Must wire before any real PWM output.
- **No alignment pulse** — AKM11E N2 option has no Hall sensors. Rotor alignment required at startup.
- **FOC not complete** — Park/Clarke transforms written but not integrated into control loop.
- **V_BUS in config.h** — must match bench supply before powering motor.

### Build
- **VS Code CMake Tools stomps toolchain** — build from terminal only, not F5 or CMake Tools UI.
- **Explicit libm path in CMakeLists.txt** — hardcoded to local ARM toolchain install path. Will break on other machines.

---

## Resolved
- Startup transient — plant_init() on STATE_ENABLED entry
- Drive state machine — IDLE → ENABLED on BLOCK_HDR, ENABLED → IDLE on ring empty
- Clock not initialized — clock_init() added to main(), was running at 16MHz
- FPU fault on startup — SCB->CPACR enable added before any float ops
- SPI framing — CS toggles per packet, refill sends DATA only (no BLOCK_HDR)
- Telem corruption on refill — BLOCK_HDR suppressed on refills

---

## LinkedIn / Repo
- [ ] Post telem plot + architecture diagram now (sim validated, HW in progress)
- [ ] Post scope shot of 3-phase PWM
- [ ] Post video of first motor spin
- [ ] Post closed-loop position tracking plot (money shot)
