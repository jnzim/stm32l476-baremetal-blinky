// config.h — motor and control constants, STM32 only
#pragma once

// ── Math ──────────────────────────────────────────────────────────────────────
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

// ── Motor — AKM11E ───────────────────────────────────────────────────────────
#define MOTOR_POLE_PAIRS    4u          // AKM11E — verify from datasheet

// ── Encoder ───────────────────────────────────────────────────────────────────
#define ENCODER_CPR         8192u                       // counts per revolution
#define COUNTS_PER_REV      ((float)ENCODER_CPR)
#define COUNTS_PER_RAD      (COUNTS_PER_REV / (2.0f * M_PI))

// ── Bus voltage ───────────────────────────────────────────────────────────────
#define V_BUS               48.0f       // volts — update to match bench supply

// ── Alignment ─────────────────────────────────────────────────────────────────
#define ALIGN_VOLTAGE       1.0f        // volts — low enough to align without overcurrent
#define ALIGN_TIME_MS       500u        // ms — time to hold rotor at alignment angle

#define FF_GAIN             0.95f        // vel FF gain~