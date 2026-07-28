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

#define SYSID_TEST_CHIRP         0
#define SYSID_TEST_CL_STEP       1
#define SYSID_TEST_VEL_CHIRP     2
#define SYSID_STEPED_SINE        3
#define SYSID_TEST_CL_VEL_STEP   4
#define RIPPLE_DEBUG             5

#define SYSID_TEST RIPPLE_DEBUG

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

#define V_ALIGN  1.5f
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

#define SYSID_AMPLITUDE  0.1f
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