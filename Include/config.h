// config.h — motor and control constants, STM32 only
#pragma once


#define RUN_MODE_SYSID        0
#define RUN_MODE_CLOSED_LOOP  1
#define RUN_MODE              RUN_MODE_SYSID

// ── Math ──────────────────────────────────────────────────────────────────────
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif
#define FOC_TWO_PI 6.28318530718f

// ── Motor — AKM11E ───────────────────────────────────────────────────────────
#define MOTOR_POLE_PAIRS    3u          // AKM11E 

// ── Encoder ───────────────────────────────────────────────────────────────────
#define ENCODER_CPR         8192u                       // counts per revolution
#define COUNTS_PER_REV      ((float)ENCODER_CPR)
#define COUNTS_PER_RAD      (COUNTS_PER_REV / (2.0f * M_PI))

// ── Bus voltage ───────────────────────────────────────────────────────────────
#define V_BUS               12.0f       // volts — update to match bench supply

#define FF_GAIN             0.95f        // vel FF gain~

#define SHUNT_R            0.007f
#define SHUNT_GAIN         20.0f
#define VREF               3.3f
#define ADC_COUNTS         4096.0f
#define ADC_ZERO           2048.0f

#define AMPS_PER_COUNT     (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
#define ADC_SMP_84_CYCLES  (4u) 


/* =============================================================================
 * Bench bring-up voltages
 *
 * These are intentionally modest.
 * Increase only after encoder angle and current feedback look sane.
 * =============================================================================*/
#define V_ALIGN  3.0f
#define V_RUN    3.0f
#define ENC_DIR  (+1.0f)



/* =============================================================================
 * SYSID globals
 *
 * =============================================================================*/
#define SYSID_AMPLITUDE     1.0f
#define SYSID_F_START       1.0f
#define SYSID_F_END         2000.0f
#define SYSID_DURATION      20.0f               // 20 seconds, ~1 second per decade
#define SYSID_DT            (1.0f / 20000.0f)

/* =============================================================================
 * Alignment timing
 *
 * ALIGN_TICKS is counted from SysTick_Handler().
 * SysTick = 20 kHz, so: 100uSec
 * =============================================================================*/
#define ALIGN_TICKS 2000u
