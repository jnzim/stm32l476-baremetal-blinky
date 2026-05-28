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




Current state (pre vaction):

# STM32F411 SPI2 / Raspberry Pi SPI Debug Notes

## Current status

We have a **known-good MOSI/RX baseline**.

### STM32F411 SPI2 slave

Pins:

```c
PB12 = NSS / CS from Pi GPIO25, AF5 hardware NSS
PB13 = SCK, AF5
PB14 = MISO, AF5
PB15 = MOSI, AF5
```

Current good STM setup:

```c
SPI2 slave
Mode 1: CPOL = 0, CPHA = 1
Hardware NSS: SSM = 0
RX DMA only: DMA1 Stream3 Channel 0 circular
No TX DMA
No TX interrupt
No SPI2->DR preload
No EXTI
No manual SPE toggling
```

Current good SPI config:

```c
SPI2->CR1 = SPI_CR1_CPHA;
SPI2->CR2 = SPI_CR2_RXDMAEN;
SPI2->CR1 |= SPI_CR1_SPE;
```

The current baseline file configures PB12–PB15 as AF5 SPI2, uses RX DMA Stream3 circular mode, and only enables `SPI_CR2_RXDMAEN`. The file explicitly says MISO telemetry is disabled/deferred.

### Raspberry Pi

Current good Pi setup:

```cpp
spidev0.0
manual CS on GPIO25
SPI Mode 1
1 MHz
no SPI_NO_CS
```

Important Pi-side discovery:

```cpp
uint8_t mode = SPI_MODE_1;
ioctl(fd, SPI_IOC_WR_MODE, &mode);
```

Do **not** use `SPI_NO_CS`; kernel rejected it with `Invalid argument`.

## Verified good MOSI/RX chain

This is verified working:

```text
Pi TX
→ STM SPI2 MOSI
→ SPI2 RX
→ DMA1 Stream3
→ spi2_rx_buf[]
→ local[]
→ CRC check
→ ring_push()
→ SysTick ring_pop()
```

Known-good result from debugger:

```text
cnt_block_hdr     = 1
cnt_data          = 2002
cnt_error         = 0
samples_consumed / cnt_ring_pop = 2002
last/debug pos    = 100000
last/debug vel    = 0
debug_ring_count  = 0
```

Interpretation:

```text
MOSI/RX path is healthy.
Frame parsing is aligned.
CRC is passing.
Ring buffer consume path is working.
```

## What broke things

### Broken test 1: Add TX DMA fixed pattern

Attempt:

```c
DMA1 Stream4 Channel 0 = SPI2_TX
Fixed 32-byte pattern: 0x01..0x20
SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
```

Result:

```text
MOSI/RX broke.
```

Conclusion:

```text
The first TX DMA graft was not safe.
Do not resume by blindly enabling TXDMAEN.
```

### Broken test 2: One-byte preload

Attempt:

```c
SPI2->CR2 = SPI_CR2_RXDMAEN;

SPI2->DR = 0x55;

SPI2->CR1 |= SPI_CR1_SPE;
```

Result:

```text
Bad.
cnt_block_hdr = 0
cnt_data      = 3
cnt_error     = 381
cnt_ring_pop  = 0
cnt_telem     = 10
```

Conclusion:

```text
The problem is not only DMA1 Stream4.
Even touching the SPI transmit side with SPI2->DR preload disturbed RX/frame parsing.
Do not use preload as next step.
```

## Old MISO-working file

We recovered an older `spi.c` where MISO/TX DMA had worked at some point.

Important old-file behavior:

```c
SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
...
DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];
DMA1_Stream4->NDTR = SPI_FRAME_BYTES;
DMA1_Stream4->CR   =
    (0u << DMA_SxCR_CHSEL_Pos) |
    (1u << DMA_SxCR_DIR_Pos)   |
    DMA_SxCR_MINC              |
    DMA_SxCR_CIRC;
...
DMA1_Stream4->CR |= DMA_SxCR_EN;
...
DMA1_Stream3->CR |= DMA_SxCR_EN;
...
SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;
```

The old file configured both RX and TX DMA and brought SPI online after DMA was armed, but the final CR1 line used `SPI_CR1_SSM | SPI_CR1_SPE`, meaning **software NSS**, despite comments saying hardware NSS.

Old file also appeared to be **Mode 0**, because it did not set `SPI_CR1_CPHA`. The current MOSI-good setup is **Mode 1**.

So old MISO-working code and current MOSI-working code differ in multiple ways:

```text
Old MISO-working:
- TX DMA enabled
- RX DMA enabled
- Software NSS: SSM = 1
- Mode 0, unless CPHA was set elsewhere
- TX DMA points to telem_buf[1]
- DMA request bits set before SPE

Current MOSI-working:
- RX DMA only
- Hardware NSS: SSM = 0
- Mode 1: CPHA = 1
- TX disabled
- No SPI2->DR preload
```

## Key conclusion

We had two different partial-good states:

```text
Old state:
MISO worked, but MOSI/RX was not yet reliable.

Current state:
MOSI/RX is reliable, but MISO/TX breaks RX when touched.
```

So next week the goal is **not** to go back to old software-NSS code.

The user preference/architecture decision is:

```text
Stick with hardware NSS.
```

Reason:

```text
For an SPI slave, hardware NSS is the cleaner architecture.
Pi GPIO25 / PB12 should frame/synchronize transactions.
Current MOSI-good state uses hardware NSS and Mode 1.
```

## Do not do next time

Do **not** start with any of these:

```c
SPI2->DR = 0x55;
```

```c
SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
```

as a quick graft into the current baseline.

Do **not** reintroduce:

```c
SPI_CR1_SSM
```

unless deliberately testing the old software-NSS architecture.

Do **not** update TX DMA `M0AR` live yet.

Do **not** combine fixed-pattern MISO, live telemetry, and RX changes in one step.

## Recommended restart plan

### Step 0: Restore/prove the good baseline

Use the current MOSI-good file.

Expected STM end-of-init:

```c
SPI2->CR1 = SPI_CR1_CPHA;
SPI2->CR2 = SPI_CR2_RXDMAEN;
__DMB();
SPI2->CR1 |= SPI_CR1_SPE;
```

Expected Pi:

```cpp
uint8_t mode = SPI_MODE_1;
ioctl(fd, SPI_IOC_WR_MODE, &mode);
```

Expected debugger:

```text
cnt_error = 0
cnt_data = 2002
samples_consumed = 2002
last_pos_cmd = 100000
last_vel_cmd = 0
```

Do not proceed until this passes.

### Step 1: Compare old MISO file versus current MOSI file

Focus only on these differences:

```text
1. CR1 final value:
   old:     SPI_CR1_SSM | SPI_CR1_SPE
   current: SPI_CR1_CPHA; then OR SPE

2. CR2 timing:
   old:     TXDMAEN | RXDMAEN set before DMA streams/SPE
   current: RXDMAEN only after RX DMA config

3. DMA order:
   old:     configure TX Stream4, enable TX, then configure RX Stream3, enable RX, then SPE
   current: configure RX only, enable RX, then SPE

4. Mode:
   old:     Mode 0 unless CPHA set elsewhere
   current: Mode 1
```

### Step 2: Build a hardware-NSS + Mode-1 + fixed-TX-DMA test

Do this from the current MOSI-good baseline, not from the old file.

Desired architecture:

```text
Hardware NSS retained
Mode 1 retained
RX DMA retained
TX DMA fixed buffer only
No telemetry
No M0AR swaps
No SPI2->DR preload
SPE enabled last
```

Proposed init order:

```c
// Disable SPI and DMA request lines
SPI2->CR1 = 0;
SPI2->CR2 = 0;

// Disable Stream3 and Stream4
// Clear all Stream3 and Stream4 DMA flags

// Configure TX DMA Stream4 first
// M0AR = fixed const 32-byte pattern
// DIR = memory-to-peripheral
// MINC = 1
// CIRC = 1
// no IRQ

// Configure RX DMA Stream3 second
// M0AR = spi2_rx_buf
// DIR = peripheral-to-memory
// MINC = 1
// CIRC = 1
// TCIE = 1

// Enable TX DMA stream
// Enable RX DMA stream

// Configure SPI Mode 1, hardware NSS
SPI2->CR1 = SPI_CR1_CPHA;

// Enable both SPI DMA request lines before SPE
SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

__DMB();

// SPE last
SPI2->CR1 |= SPI_CR1_SPE;
```

Fixed TX test pattern should be simple:

```c
static const uint8_t spi_tx_test[32] =
{
    0x01,0x02,0x03,0x04,
    0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,
    0x0D,0x0E,0x0F,0x10,
    0x11,0x12,0x13,0x14,
    0x15,0x16,0x17,0x18,
    0x19,0x1A,0x1B,0x1C,
    0x1D,0x1E,0x1F,0x20
};
```

Expected Pi MISO:

```text
01 02 03 04 ... 20
```

But RX health is more important than MISO.

Pass criteria:

```text
cnt_error = 0
cnt_data increases correctly
samples_consumed increases correctly
last_pos_cmd reaches 100000
last_vel_cmd reaches 0
Pi MISO receives stable counting pattern
```

Fail criteria:

```text
cnt_error increases
cnt_block_hdr fails
cnt_data stalls
ring pop stalls
MISO works but MOSI breaks
```

If MOSI breaks, revert immediately to RX-only baseline.

### Step 3: If fixed TX DMA still breaks RX

Then test at lower Pi speed:

```text
1 MHz → 250 kHz
```

Do not change STM code at the same time.

Interpretation:

```text
Works at 250 kHz but not 1 MHz:
    likely timing/DMA arbitration/first-byte service issue.

Fails at 250 kHz too:
    likely config/order/NSS/SPI peripheral behavior issue.
```

### Step 4: Only after fixed MISO works

Then replace fixed TX buffer with telemetry.

Do not swap `M0AR` live as the first telemetry test.

Safer next telemetry step:

```text
Use one stable TX buffer.
Update it only between transactions / when CS is inactive.
Or use a double buffer only after fixed-buffer TX DMA is stable.
```

## State to remember

Current safe state:

```text
STM: hardware NSS, Mode 1, RX DMA only
Pi:  Mode 1, manual CS GPIO25, 1 MHz
MOSI/RX: working cleanly
MISO: disabled
```

Next real task:

```text
Reintroduce MISO while preserving hardware NSS and Mode 1.
Start with fixed TX DMA pattern, with both DMA streams armed before SPE.
Do not preload SPI2->DR.
Do not use software NSS.
Do not add telemetry until fixed TX pattern passes.
```
