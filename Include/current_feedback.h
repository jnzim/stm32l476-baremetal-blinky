// current_feedback.h — 3-phase current sense via DRV8353RS shunt amplifiers
//
// PC0/ADC1_IN10 = ISENA, PC1/ADC1_IN11 = ISENB, PC2/ADC1_IN12 = ISENC
// Shunt = 7 mΩ, Gain = 10 V/V, Zero = 1.65V = 2048 counts

#pragma once

#include <stdint.h>

// Raw DMA buffer — inspect in debugger during bring-up
// Should read ~2048 at rest (zero current)
extern volatile uint16_t current_adc_raw[3];

// Call after tim1_init() and pwm_init(), before calibrate
void current_feedback_init(void);

// Call before pwm_enable() — motor stationary, ~13 ms
void current_feedback_calibrate(void);

// Call from ISR after DMA poll — returns amps
// ia + ib + ic should sum to ~0
void current_feedback_get(float *ia, float *ib, float *ic);