// foc_sysid.h — system identification mode FOC
//
// Called from TIM1_UP_TIM10_IRQHandler when RUN_MODE == RUN_MODE_SYSID.
//
// Owns the full ISR execution in sysid mode:
//   - d-axis alignment stage
//   - chirp sweep (open-loop Vd excitation, theta=0, existing implementation)
//   - CL step response (closed current loop, step on iq_cmd reference)
//
// The RPi sysid reader project handles 10 kHz data telem independently.
// This file just keeps writing spi_sysid_update_latest() every ISR tick.

#pragma once

// foc_sysid_step — call from TIM1 ISR at 20 kHz, RUN_MODE_SYSID only
void foc_sysid_step(void);

// foc_sysid_reset — call on fault recovery or re-enable
void foc_sysid_reset(void);




