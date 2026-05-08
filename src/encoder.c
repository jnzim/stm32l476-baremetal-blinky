// encoder.c — TIM5 quadrature encoder, STM32F446RE bare metal
//
// TIM5 (Timer 5) is a 32-bit timer — counts up/down without overflow
// at any reasonable motor speed. PA0 = TIM5_CH1, PA1 = TIM5_CH2.
// Encoder mode: TIM5 counter tracks quadrature A/B signals directly.
// No CPU involvement — hardware counts edges automatically.

#include "encoder.h"
#include "stm32f4xx.h"

// defined here, declared extern in encoder.hpp
volatile EncoderState encoder = {0, 0, 0};

// previous position saved each velocity calculation
static int32_t last_position = 0;


// =============================================================================
// encoder_init
// Call once at startup after SystemClock_Config
// =============================================================================
void encoder_init(void) {

    // -------------------------------------------------------------------------
    // STEP 1 — enable clocks
    // -------------------------------------------------------------------------
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock — PA0, PA1
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;   // TIM5 (Timer 5) clock — APB1 bus

    // -------------------------------------------------------------------------
    // STEP 2 — configure PA0, PA1 as alternate function AF2 = TIM5_CH1, TIM5_CH2
    // -------------------------------------------------------------------------

    // MODER (Mode Register) — set PA0 and PA1 to alternate function mode (10)
    GPIOA->MODER &= ~( (3u << 0) | (3u << 2) );  // clear PA0, PA1
    GPIOA->MODER |=  ( (2u << 0) | (2u << 2) );  // set alternate function

    // AFR (Alternate Function Register) — AF2 = TIM5 on PA0, PA1
    // AFR[0] covers pins 0-7, each pin gets 4 bits
    // PA0 = bits [3:0], PA1 = bits [7:4]
    GPIOA->AFR[0] &= ~( (0xFu << 0) | (0xFu << 4) );  // clear PA0, PA1 fields
    GPIOA->AFR[0] |=  ( (2u   << 0) | (2u   << 4) );  // AF2 = TIM5

    // pull-up on both pins — encoder signals should not float
    // PUPDR (Pull-Up Pull-Down Register) — 01 = pull-up
    GPIOA->PUPDR &= ~( (3u << 0) | (3u << 2) );
    GPIOA->PUPDR |=  ( (1u << 0) | (1u << 2) );

    // -------------------------------------------------------------------------
    // STEP 3 — configure TIM5 in encoder mode
    //
    // Encoder mode uses TIM5_CH1 (PA0) and TIM5_CH2 (PA1) as the A and B
    // quadrature inputs. The hardware counts edges automatically and the
    // counter increments or decrements based on direction.
    //
    // SMS (Slave Mode Selection) in SMCR (Slave Mode Control Register):
    //   001 = encoder mode 1 — counts on TI1 edges only
    //   010 = encoder mode 2 — counts on TI2 edges only
    //   011 = encoder mode 3 — counts on both TI1 and TI2 edges (4x resolution)
    //   We use mode 3 for maximum resolution.
    //
    // CCMR1 (Capture Compare Mode Register 1):
    //   IC1F and IC2F = input filter — set to 0 for no filtering initially
    //   CC1S and CC2S = channel direction — 01 = input mapped to TI1/TI2
    // -------------------------------------------------------------------------

    TIM5->CR1  = 0;  // start clean
    TIM5->SMCR = 0;

    // CCMR1 — configure CH1 and CH2 as inputs
    // CC1S (Capture Compare 1 Selection) = 01 — CH1 mapped to TI1 input
    // CC2S (Capture Compare 2 Selection) = 01 — CH2 mapped to TI2 input
    TIM5->CCMR1 = (1u << 0)   // CC1S = 01 — input TI1
                | (1u << 8);   // CC2S = 01 — input TI2

    // SMCR — encoder mode 3, count on both edges
    TIM5->SMCR = (3u << 0);  // SMS = 011 — encoder mode 3

    // CCER (Capture Compare Enable Register) — polarity
    // CC1P and CC2P = 0 — non-inverted, rising edge
    // Leave at reset value (0) — both channels non-inverted
    TIM5->CCER = 0;

    // ARR (Auto Reload Register) — maximum count value
    // TIM5 is 32-bit so set to max — counter wraps at 0xFFFFFFFF
    // We read it as signed int32_t so it naturally handles negative counts
    TIM5->ARR = 0xFFFFFFFFu;

    // reset counter to 0 at startup
    TIM5->CNT = 0;

    // CR1 CEN (Counter ENable) — start the counter
    TIM5->CR1 |= TIM_CR1_CEN;
}


// =============================================================================
// encoder_get_position
// Returns current TIM5 counter as signed 32-bit encoder counts.
// Positive = forward, negative = reverse from startup position.
// =============================================================================
int32_t encoder_get_position(void) {
    return (int32_t)TIM5->CNT;
}


// =============================================================================
// encoder_get_velocity
// Returns last computed dx/dt in counts/sec.
// Updated by the TIM1 ISR — call this from velocity loop, not here.
// =============================================================================
int32_t encoder_get_velocity(void) {
    return encoder.velocity;
}


// =============================================================================
// encoder_update
// Call from TIM1 ISR at 20 kHz (every 50 µs).
// Reads TIM5 counter, computes dx/dt, updates encoder struct.
//
// dx/dt = (current_position - last_position) / dt
// dt = 50e-6 seconds = 1/20000
// multiply by 20000 instead of dividing by 50e-6 — integer math, no float
// =============================================================================
void encoder_update(uint32_t timestamp_ms) {
    int32_t current = (int32_t)TIM5->CNT;
    int32_t delta   = current - last_position;  // counts since last update

    encoder.position     = current;
    encoder.velocity     = delta * 20000;        // counts/sec — 20 kHz sample rate
    encoder.timestamp_ms = timestamp_ms;

    last_position = current;
}