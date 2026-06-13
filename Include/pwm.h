#pragma once
#include <stdint.h>

// pwm_init — configure TIM1 + GPIO, outputs disabled (MOE=0)
void pwm_init(void);

// pwm_enable — set MOE, PWM signals reach gate driver
void pwm_enable(void);

// pwm_disable — clear MOE, all outputs low
void pwm_disable(void);

// pwm_set_duty — set CCR for one phase, 0–4499
// phase: 0=A, 1=B, 2=C
void pwm_set_duty(uint8_t phase, uint16_t duty);

// volts_to_duty — convert phase voltage to CCR value
// v: [-V_BUS, +V_BUS] → returns 0–4499
uint16_t volts_to_duty(float v);

// pwm_apply_vq — full FOC output stage
// v_q: q-axis voltage (current loop output)
// v_d: d-axis voltage (0 unless field weakening)
// theta: electrical rotor angle in radians (from encoder)
void pwm_apply_vq(float v_q, float v_d, float theta);

