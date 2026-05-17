#pragma once
#include <stdint.h>

// pwm_init — configure TIM1 + GPIO, outputs disabled (MOE=0)
// Call from main() before enabling interrupts
void pwm_init(void);

// pwm_enable — set MOE, PWM signals reach gate driver
// Call from drive state machine on STATE_ENABLED entry
void pwm_enable(void);

// pwm_disable — clear MOE, all outputs low (Hi-Z on DRV8353RS)
// Call on fault, STATE_IDLE, or any unsafe condition
void pwm_disable(void);

// pwm_set_duty — set duty cycle for one phase
// phase: 0=A, 1=B, 2=C   duty: 0–4499
void pwm_set_duty(uint8_t phase, uint16_t duty);