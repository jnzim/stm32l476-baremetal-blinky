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

#define SYSID_TEST SYSID_TEST_CINE_SWEEP


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

#define V_BUS   12.0f

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


#define VEL_CHIRP_F_START    2.0f
#define VEL_CHIRP_F_END     100.0f
#define VEL_CHIRP_DURATION  100.0f
#define VEL_CHIRP_AMPLITUDE  0.1f

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
// ACTUAL measured gain there, not assumed unity. Crossover = 24.3 Hz,
// measured PM = 60.0 deg, GM = 7.2 dB.
//
// Superseded the original decade-below-BW heuristic (Kp=51.81, crossover
// ~7-8 Hz, PM never measured) after the whole-system closed-position-loop
// chirp (SYSID_TEST_CL_POS_CHIRP) confirmed that heuristic's predicted BW
// to within ~15% of the measured 7.01 Hz -- enough confidence in the H(s)/s
// model to trust its faster, margin-verified alternative too.
// =============================================================================

#define POSITION_LOOP_KP        157.7f   // (rad/s) per rad of position error
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