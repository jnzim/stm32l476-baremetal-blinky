// encoder.h — TIM5 quadrature encoder interface
// STM32F446RE bare metal

#pragma once
#include <stdint.h>

// =============================================================================
// EncoderState — current position and velocity
// Written by TIM1 ISR, read by velocity loop and telem
// =============================================================================
typedef struct {
    int32_t  position;      // (encoder counts, absolute from startup)
    int32_t  velocity;      // (dx/dt, counts/sec)
    uint32_t timestamp_ms;  // (time of last update)
} EncoderState;

// extern = defined in encoder.c, accessible here
// volatile = TIM1 ISR writes this, superloop and velocity loop read it,
//            compiler must not cache or reorder
extern volatile EncoderState encoder;

// =============================================================================
// Public interface
// =============================================================================
void    encoder_init(void);         // (call once at startup after SystemClock_Config())
int32_t encoder_get_position(void); // (returns TIM5 counter as signed 32-bit counts)
int32_t encoder_get_velocity(void); // (returns last computed dx/dt, counts/sec)