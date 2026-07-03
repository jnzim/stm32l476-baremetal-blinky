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

#define SYSID_TEST_CHIRP    0
#define SYSID_TEST_CL_STEP  1
#define SYSID_TEST          SYSID_TEST_CHIRP

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

// #define SHUNT_R          0.007f
// #define SHUNT_GAIN       20.0f
// #define VREF             3.3f
// #define ADC_COUNTS       4096.0f
// #define ADC_ZERO         2048.0f
// #define AMPS_PER_COUNT   (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
// #define ADC_SMP_84_CYCLES  4u


#define SHUNT_R          0.007f      // 7mΩ shunt resistor
#define CSA_GAIN         40.0f       // DRV8353 CSA gain register = 11b
#define DIVIDER_RATIO    (9.76f / (383.0f + 9.76f))   // R7/(R1+R7) = 0.02484
#define SHUNT_GAIN  (CSA_GAIN * DIVIDER_RATIO * 2.0f)  // ×2 for single-ended vs differential
#define VREF             3.3f
#define ADC_COUNTS       4096.0f
#define ADC_ZERO         2048.0f
#define AMPS_PER_COUNT   (VREF / (ADC_COUNTS * SHUNT_GAIN))  // = 0.000806 A/count
#define ADC_SMP_84_CYCLES  4u

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
#define V_RUN    3.0f
#define ENC_DIR  (+1.0f)
#define FF_GAIN  0.95f

// =============================================================================
// Alignment timing — ALIGN_TICKS at 20 kHz = 100 ms
// =============================================================================

#define ALIGN_TICKS  2000u

// =============================================================================
// Sysid chirp parameters
// =============================================================================

#define SYSID_AMPLITUDE  1.0f
#define SYSID_F_START    1.0f
#define SYSID_F_END      2000.0f
#define SYSID_DURATION   20.0f
#define SYSID_DT         DT_CURRENT