#pragma once

// =============================================================================
// Run mode
// =============================================================================

#define RUN_MODE_SYSID        0
#define RUN_MODE_CLOSED_LOOP  1
#define RUN_MODE              RUN_MODE_SYSID

// =============================================================================
// Sysid test selector
// =============================================================================

#define SYSID_TEST_CURRENT_LOOP_CHIRP       0
#define SYSID_TEST_CURRENT_LOOP_STEP        1
#define SYSID_TEST_VEL_CHIRP                2
#define SYSID_TEST_CL_VEL_STEP              4
#define RIPPLE_DEBUG                        5
#define SYSID_TEST_CL_VEL_CHIRP             6
#define SYSID_TEST_POSITION_STEP            7
#define SYSID_TEST_CL_POS_CHIRP             8
#define SYSID_TEST_CINE_SWEEP               9

#define SYSID_TEST SYSID_TEST_VEL_CHIRP

// =============================================================================
// SPI telemetry
// =============================================================================
//
// CL_VEL_CHIRP instability events used to end with the whole system frozen.
// Root cause: EXTI15_10_IRQHandler (CS rising edge, NVIC priority 0 --
// highest in the system) was passively waiting for DMA1_Stream4's TX
// enable bit to self-clear, which only happens if the host's SPI
// transaction completed all 32 bytes on its own. A short/glitched host
// transaction leaves the stream permanently mid-transfer with no more
// clock edges ever coming, so it never self-clears -- confirmed live via
// debugger (stall_guard counting all the way down). At priority 0, burning
// the full ~6-8ms bound there blocks the main control loop, ADC, and
// everything else for the duration -- that's what was actually causing the
// "instability" all session; disabling SPI entirely (3 clean passes, no
// freeze) proved it wasn't the velocity loop. Fixed properly in
// EXTI15_10_IRQHandler (spi.c) -- it now force-disables the stream instead
// of waiting on natural completion, so re-enabling telemetry here.
#define SPI_TELEM_ENABLED  1



// =============================================================================
// Math
// =============================================================================

#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif
#define FOC_TWO_PI  6.28318530718f

// =============================================================================
// Motor — AKM11E
// =============================================================================

#define MOTOR_POLE_PAIRS    3u

// =============================================================================
// Encoder
// =============================================================================

#define ENCODER_CPR      8192u
#define COUNTS_PER_REV   ((float)ENCODER_CPR)
#define COUNTS_PER_RAD   (COUNTS_PER_REV / (2.0f * M_PI))

// =============================================================================
// Bus voltage
// =============================================================================

#define V_BUS   12.0f    // matches the PS setting -- 24V caused severe SPI corruption earlier, never re-validated after the actual DMA root cause fix

// =============================================================================
// Current sensing
// =============================================================================

#define SHUNT_R          0.007f
#define SHUNT_GAIN       40.0f
#define VREF             3.3f
#define ADC_COUNTS       4096.0f
#define ADC_ZERO         2048.0f
#define AMPS_PER_COUNT   (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
#define ADC_SMP_84_CYCLES  4u
#define ADC_SMP_15_CYCLES  1u

// =============================================================================
// Control loop timing
// =============================================================================

#define DT_CURRENT   (1.0f / 20000.0f)
#define DT_VELOCITY  (1.0f /  5000.0f)
#define DT_POSITION  (1.0f /  1000.0f)

// =============================================================================
// FOC bring-up voltages
// =============================================================================

#define V_ALIGN  3.0f
#define V_RUN    1.0f
#define ENC_DIR  (+1.0f)
#define FF_GAIN  0.95f


// =============================================================================
// Alignment timing — ALIGN_TICKS at 20 kHz = 100 ms
// =============================================================================

#define ALIGN_TICKS  100000u

// =============================================================================
// Current loop ID Sysid chirp parameters
// =============================================================================

#define SYSID_AMPLITUDE  1.0f
#define SYSID_F_START    1.0f
#define SYSID_F_END      2000.0f
#define SYSID_DURATION   20.0f
#define SYSID_DT         DT_CURRENT

// =============================================================================
// Velocity loop ID Sysid chirp parameters
// =============================================================================


#define VEL_CHIRP_F_START    0.5f
#define VEL_CHIRP_F_END     100.0f
#define VEL_CHIRP_DURATION  100.0f
#define VEL_CHIRP_AMPLITUDE  0.1f    // resting value -- amplitude sweep (0.3/0.5/0.7A) showed strong
                                     // friction-driven nonlinearity, not resolved tonight; tomorrow's
                                     // friction characterization test replaces this approach

// iq_cmd here is open-loop and unclamped by construction -- nothing bounds
// it like VEL_IQ_LIMIT bounds the closed velocity loop's PI output. At
// AMPLITUDE=0.3A this ran away once real motion (real back-EMF) appeared,
// current railed at the ADC's representable limit for ~3.5s, sagged the
// bench PS hard enough (shared return/ground) to brown-out reset the MCU
// -- confirmed via RCC->CSR (BORRSTF/PORRSTF). Safety ceiling, not a normal
// operating limit -- should never actually bind at the intended amplitude.
//
// Tightened from 1.0A -- the clamp only bounds the *commanded* iq_cmd, not
// how hard the current loop's PI can react to a real disturbance. A real
// stick-slip breakaway snap (confirmed via ADC counter staying healthy
// through the event -- not a stale-sensor artifact) still let measured
// current ring out to ~10.5A despite iq_cmd never exceeding 1.0A. Tighter
// clamp limits how much corrective authority the loop has to ring with.
#define VEL_CHIRP_IQ_LIMIT  1.0f    // A -- 0.3A was reactive (well below the ~10.5A ring-out), not
                                    // hardware-derived. Motor rated 2.9A continuous; still well under
                                    // that with real margin for AMPLITUDE=0.2A to sit unclamped.

// Was a DC-biased chirp (kept the stage moving one direction to avoid
// stick-slip breakaway every half cycle) with a soft travel-limit reversal
// -- both removed. The travel-limit reversal introduced real discontinuities
// into the chirp response that corrupted the linear-system-ID fit; bias
// went with it since the reversal logic was its only consumer.

// Settle-onto-zero before the chirp starts -- with the bias removed, this
// only needs to cover current decaying to zero (tau ~= 0.79ms electrical),
// not letting the stage reach a steady-state kinetic speed anymore. 20
// ticks @ this test's raw 20kHz rate = 1ms, comfortably past tau. Kept as
// its own constant (not reused from CL_VEL_CHIRP_SETTLE_TICKS below) since
// that one runs at the 5kHz-decimated velocity-loop rate -- same tick count
// would mean a different real time on each.
#define VEL_CHIRP_SETTLE_TICKS  20u

// =============================================================================
// Closed velocity loop chirp — sweeps vel_cmd (not iq) with the vel PI
// closed, to identify the CLOSED-loop vel_cmd -> vel_meas response for
// designing the outer position P controller.

// =============================================================================

#define CL_VEL_CHIRP_F_START     0.5f
#define CL_VEL_CHIRP_F_END     200.0f
#define CL_VEL_CHIRP_DURATION   60.0f
#define CL_VEL_CHIRP_AMPLITUDE  10.0f    // rad/s

// =============================================================================
// Position loop — pure P, phase-margin-targeted design from the closed-
// velocity-loop chirp (see closed_vel_bode_plot.py): crossover picked where
// H(s)'s measured phase lag = 30 deg -> PM_pos = 60 deg, Kp sized from the
// ACTUAL measured gain there, not assumed unity. Re-fit after the wiring fix
// and VEL_KP/KI re-tune: crossover = 23.64 Hz, measured PM = 60.0 deg,
// GM = 7.3 dB. Verify against SYSID_TEST_CL_POS_CHIRP before trusting it.
// =============================================================================

#define POSITION_LOOP_KP        150.6f   // (rad/s) per rad of position error
#define POSITION_VEL_LIMIT      20.0f    // rad/s -- clamp P output, no windup state to clamp

// =============================================================================
// Position step test parameters
// =============================================================================

// NOTE: cl_step_tick in run_position_step() increments inside the
// position-loop-decimated block (1 kHz, POS_LOOP_DECIMATE=20), NOT the raw
// 20kHz ISR tick -- so these counts are in 1kHz ticks, not 20kHz ticks.
#define POSITION_STEP_AMPLITUDE_RAD  0.2f      // ~11.5 deg -- gentle first test
#define POSITION_STEP_SETTLE_TICKS   1000u     // 1s @ 1kHz
#define POSITION_STEP_HOLD_TICKS     2000u     // 2s @ 1kHz

// =============================================================================
// Closed position loop chirp — sweeps pos_cmd with the FULL cascade closed
// (position P -> velocity PI -> current PI), to directly MEASURE the actual
// closed-loop response of the whole system (pos_meas/pos_cmd), rather than
// relying on the H(s)/s construction used to design POSITION_LOOP_KP.
//
// F_END was originally 30 Hz, sized for the OLD conservative Kp_pos=51.81
// (~7-8 Hz expected BW). With the faster phase-margin-targeted Kp_pos=157.7,
// a 30 Hz sweep top wasn't high enough to see the closed-loop -3dB point OR
// the open-loop phase reach -180 deg (mag was still only -1.3dB and phase
// only -108 deg at 30 Hz) -- widened to 150 Hz for headroom above both.
// =============================================================================

#define CL_POS_CHIRP_F_START     0.1f     // Hz
#define CL_POS_CHIRP_F_END     150.0f     // Hz
#define CL_POS_CHIRP_DURATION   60.0f     // s
#define CL_POS_CHIRP_AMPLITUDE   0.15f   // rad

// =============================================================================
// Cine sweep — same closed-loop position chirp, but sized for FILMING. Now
// deliberately swept PAST both gains' real tracking limit -- the point isn't
// just to look pretty, it's to visibly show the system run out of bandwidth
// (or hit its velocity clamp) as frequency climbs: measured amplitude
// shrinking below commanded, growing phase lag, eventually degenerating into
// a blurred/reduced-amplitude wobble instead of clean tracking. The overlay
// graph (cine_overlay_render.py) makes the breakdown legible even once the
// physical motion itself is too fast to resolve by eye.
//
// AMPLITUDE was cut from 0.4 to 0.25 rad so the failure you see is genuinely
// BANDWIDTH-driven for as much of the sweep as possible, not an artifact of
// hitting POSITION_VEL_LIMIT early: a perfectly-tracking loop needs peak
// velocity ~2*pi*f*AMPLITUDE, which only reaches the 20 rad/s clamp around
// 12.7 Hz at this amplitude (vs. ~8 Hz at the old 0.4 rad) -- past that point
// the clamp becomes part of the limitation too, which is a real, legitimate
// system limit, not a bug in the test.
//
// Run it once per gain you want to compare on camera (rebuild/reflash between
// runs): POSITION_LOOP_KP=51.81 for the "before" take, 157.7 for "after".
// Expect the old gain to visibly give up well before the new one.
//
// F_START raised from 0.1 to 3Hz -- the slow end was dead time, nothing
// visibly interesting happens until well into the sweep. F_END raised from
// 40 to 80Hz -- 40 wasn't high enough to kill the visible motion entirely,
// it was still clearly vibrating at the end. DURATION cut from 40 to 20s to
// keep the sweep rate (time per decade) roughly the same over the smaller,
// higher-frequency range instead of dragging the interesting part out.
// =============================================================================

#define CINE_SWEEP_F_START      3.0f    // Hz
#define CINE_SWEEP_F_END       80.0f    // Hz
#define CINE_SWEEP_DURATION    20.0f    // s
#define CINE_SWEEP_AMPLITUDE    0.25f   // rad