#include "encoder.h"
#include "stm32f4xx.h"

// Hardware mapping
#define ENC_TIM              TIM2
#define ENC_TIM_CLK_EN()     (RCC->APB1ENR |= RCC_APB1ENR_TIM2EN)

#define ENC_GPIO             GPIOA
#define ENC_GPIO_CLK_EN()    (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)

#define ENC_CH1_PIN          0u      // PA0 = TIM2_CH1
#define ENC_CH2_PIN          1u      // PA1 = TIM2_CH2
#define ENC_AF               1u      // AF1 = TIM2 on PA0/PA1

#define ENC_SAMPLE_HZ        20000   // encoder_update() called from 20 kHz TIM1 ISR

// defined here, declared extern in encoder.h
volatile EncoderState encoder = {0, 0, 0};

// previous position saved each velocity calculation
static int32_t last_position = 0;


// =============================================================================
// encoder_init
// Call once at startup after system clock setup.
// =============================================================================
void encoder_init(void)
{
    // -------------------------------------------------------------------------
    // STEP 1 — enable clocks
    // -------------------------------------------------------------------------
    ENC_GPIO_CLK_EN();    // GPIOA clock — PA0, PA1
    ENC_TIM_CLK_EN();     // TIM2 clock — APB1 bus

    // -------------------------------------------------------------------------
    // STEP 2 — configure PA0, PA1 as alternate function AF1 = TIM2_CH1/TIM2_CH2
    // -------------------------------------------------------------------------

    // MODER: alternate function mode = 10
    ENC_GPIO->MODER &= ~((3u << (ENC_CH1_PIN * 2u)) |
                         (3u << (ENC_CH2_PIN * 2u)));

    ENC_GPIO->MODER |=  ((2u << (ENC_CH1_PIN * 2u)) |
                         (2u << (ENC_CH2_PIN * 2u)));

    // AFR[0]: PA0/PA1 alternate function.
    // AF1 = TIM2.
    ENC_GPIO->AFR[0] &= ~((0xFu << (ENC_CH1_PIN * 4u)) |
                          (0xFu << (ENC_CH2_PIN * 4u)));

    ENC_GPIO->AFR[0] |=  ((ENC_AF << (ENC_CH1_PIN * 4u)) |
                          (ENC_AF << (ENC_CH2_PIN * 4u)));

    // No internal pullups/pulldowns.
    // MAX3096 actively drives Y1/Y2.
    ENC_GPIO->PUPDR &= ~((3u << (ENC_CH1_PIN * 2u)) |
                         (3u << (ENC_CH2_PIN * 2u)));

    // -------------------------------------------------------------------------
    // STEP 3 — configure TIM2 in encoder mode
    //
    // Encoder mode uses TIM2_CH1 and TIM2_CH2 as quadrature A/B inputs.
    //
    // SMCR.SMS:
    //   001 = encoder mode 1 — counts on TI1 edges only
    //   010 = encoder mode 2 — counts on TI2 edges only
    //   011 = encoder mode 3 — counts on both TI1 and TI2 edges, 4x resolution
    //
    // CCMR1:
    //   CC1S = 01 — CH1 input mapped to TI1
    //   CC2S = 01 — CH2 input mapped to TI2
    //   IC1F/IC2F = 0 — no digital filter initially
    // -------------------------------------------------------------------------

    ENC_TIM->CR1   = 0;
    ENC_TIM->CR2   = 0;
    ENC_TIM->SMCR  = 0;
    ENC_TIM->CCMR1 = 0;
    ENC_TIM->CCER  = 0;
    ENC_TIM->PSC   = 0;

    // CH1 and CH2 as inputs
    ENC_TIM->CCMR1 = (1u << 0)     // CC1S = 01 — input TI1
                   | (1u << 8);    // CC2S = 01 — input TI2

    // Encoder mode 3: count on both TI1 and TI2 edges
    ENC_TIM->SMCR = (3u << 0);     // SMS = 011

    // Non-inverted polarity on both channels
    ENC_TIM->CCER = 0;

    // TIM2 is 32-bit
    ENC_TIM->ARR = 0xFFFFFFFFu;
    ENC_TIM->CNT = 0;

    last_position = 0;
    encoder.position = 0;
    encoder.velocity = 0;
    encoder.timestamp_ms = 0;

    // Start counter
    ENC_TIM->CR1 |= TIM_CR1_CEN;
}


// =============================================================================
// encoder_get_position
// Returns current TIM2 counter as signed 32-bit encoder counts.
// Positive/negative direction depends on A/B wiring.
// =============================================================================
int32_t encoder_get_position(void)
{
    return (int32_t)ENC_TIM->CNT;
}


// =============================================================================
// encoder_get_velocity
// Returns last computed dx/dt in counts/sec.
// Updated by encoder_update(), currently called from the 20 kHz TIM1 ISR.
// =============================================================================
int32_t encoder_get_velocity(void)
{
    return encoder.velocity;
}


// =============================================================================
// encoder_zero
// Reset encoder position and velocity state.
// =============================================================================
void encoder_zero(void)
{
    ENC_TIM->CNT = 0;
    last_position = 0;

    encoder.position = 0;
    encoder.velocity = 0;
}


// =============================================================================
// encoder_update
// Call from TIM1 ISR at 20 kHz.
// Reads TIM2 counter, computes dx/dt, updates encoder struct.
//
// velocity = delta_counts * sample_rate_hz
// =============================================================================
void encoder_update(uint32_t timestamp_ms)
{
    int32_t current         = (int32_t)ENC_TIM->CNT;
    int32_t delta           = current - last_position;

    encoder.position        = current;
    encoder.velocity        = delta * ENC_SAMPLE_HZ;
    encoder.timestamp_ms    = timestamp_ms;

    last_position = current;
   
}

