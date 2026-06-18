// current_feedback.c — 3-phase current sense via DRV8353RS onboard shunt amps
//
// STM32F411/F446 bare metal
//
// Hardware:
//   PC0 / ADC1_IN10 -> ISENA
//   PC1 / ADC1_IN11 -> ISENB
//   PC2 / ADC1_IN12 -> ISENC
//
// Assumptions:
//   shunt resistors = 0.007 ohm
//   DRV8353 CSA gain = 10 V/V
//   ADC reference = 3.3 V
//   zero current is approximately VREF / 2
//
// Bring-up mode:
//   ADC1 injected sequence samples PC0/PC1/PC2.
//   ADC injected conversion is started by software from the TIM1 ISR.
//
// Why software-start for now:
//   TIM1 ISR fires.
//   ADC injected software-start works.
//   TIM1_CC4 external ADC trigger is not working yet.
//   This gets real current samples flowing now.
//
// Important:
//   Do NOT use ADC_SR_JEOC from the current project headers.
//   In this project it appeared to be 0x08.
//   On STM32F4/F411, JEOC is bit 2 = 0x04.

#include "current_feedback.h"
#include "board_f411.h"
#include "clock.h"
#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>


// =============================================================================
// Current-sense scaling
// =============================================================================

#define SHUNT_R         0.007f
#define SHUNT_GAIN      10.0f
#define VREF            3.3f
#define ADC_COUNTS      4096.0f
#define ADC_ZERO        2048.0f

#define AMPS_PER_COUNT  (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))


// =============================================================================
// ADC status bits
// =============================================================================
//
// STM32F4 ADC SR bits:
//   AWD   = bit 0 = 0x01
//   EOC   = bit 1 = 0x02
//   JEOC  = bit 2 = 0x04
//   JSTRT = bit 3 = 0x08
//   STRT  = bit 4 = 0x10
//   OVR   = bit 5 = 0x20
//
// Use this instead of ADC_SR_JEOC.

#define ADC_SR_JEOC_BIT   (1u << 2)


// =============================================================================
// ADC injected data
// =============================================================================

volatile uint16_t current_adc_raw[3] =
{
    2048u,
    2048u,
    2048u
};


// =============================================================================
// Calibration offsets
// =============================================================================

float adc_offset[3] =
{
    ADC_ZERO,
    ADC_ZERO,
    ADC_ZERO
};


// =============================================================================
// Sample status / debug counters
// =============================================================================

static volatile bool current_sample_valid = false;
static volatile uint32_t current_sample_count = 0u;
static volatile uint32_t current_missed_count = 0u;


// =============================================================================
// current_feedback_sample_once
//
// Starts one injected ADC sequence by software and reads JDR1/JDR2/JDR3.
//
// This function is used by:
//   - current_feedback_calibrate()
//   - current_feedback_update()
//
// For now this is the known-good path.
// =============================================================================

static bool current_feedback_sample_once(void)
{
    uint32_t timeout = 10000u;

    /*
     * Software-start injected conversion mode.
     *
     * JEXTEN must be disabled, otherwise external trigger mode owns injected
     * conversion start.
     */
    ADC1->CR2 &= ~ADC_CR2_JEXTEN;

    /*
     * Clear real JEOC bit before starting.
     */
    ADC1->SR &= ~ADC_SR_JEOC_BIT;

    /*
     * Start injected sequence.
     */
    ADC1->CR2 |= ADC_CR2_JSWSTART;

    while ((ADC1->SR & ADC_SR_JEOC_BIT) == 0u)
    {
        if (timeout-- == 0u)
        {
            current_missed_count++;
            current_sample_valid = false;
            return false;
        }
    }

    current_adc_raw[0] = ADC1->JDR1;
    current_adc_raw[1] = ADC1->JDR2;
    current_adc_raw[2] = ADC1->JDR3;

    ADC1->SR &= ~ADC_SR_JEOC_BIT;

    current_sample_count++;
    current_sample_valid = true;

    return true;
}


// =============================================================================
// current_feedback_init
//
// Configure:
//   - PC0/PC1/PC2 as analog inputs
//   - ADC1 injected sequence: IN10, IN11, IN12
//
// No DMA.
// No ADC continuous mode.
// No external injected trigger for now.
// =============================================================================

void current_feedback_init(void)
{
    // -------------------------------------------------------------------------
    // Enable peripheral clocks
    // -------------------------------------------------------------------------

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;


    // -------------------------------------------------------------------------
    // PC0 / PC1 / PC2 analog mode
    //
    // PC0 = ADC1_IN10 = phase A current
    // PC1 = ADC1_IN11 = phase B current
    // PC2 = ADC1_IN12 = phase C current
    // -------------------------------------------------------------------------

    GPIOC->MODER |=  (3u << (PIN_CUR_A * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_B * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_C * 2u));

    GPIOC->PUPDR &= ~(3u << (PIN_CUR_A * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_B * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_C * 2u));


    // -------------------------------------------------------------------------
    // ADC common control
    //
    // ADCPRE = PCLK2 / 4.
    // If PCLK2 = 84 MHz, ADC clock = 21 MHz.
    // -------------------------------------------------------------------------

    ADC->CCR = ADC_CCR_ADCPRE_0;


    // -------------------------------------------------------------------------
    // Start from clean ADC config
    // -------------------------------------------------------------------------

    ADC1->CR1 = 0u;
    ADC1->CR2 = 0u;
    ADC1->SR  = 0u;


    // -------------------------------------------------------------------------
    // Enable scan mode for multi-channel injected sequence
    // -------------------------------------------------------------------------

    ADC1->CR1 |= ADC_CR1_SCAN;


    // -------------------------------------------------------------------------
    // Sample time
    //
    // Channels 10, 11, 12 are in SMPR1.
    //
    // 4 = 84 ADC cycles.
    // At 21 MHz ADC clock:
    //
    //   3 * (84 + 12) / 21 MHz ~= 13.7 us
    //
    // This is okay for bring-up at 20 kHz, but it eats ISR time.
    // Later, reduce sample time once the current feedback path is proven.
    // -------------------------------------------------------------------------

    ADC1->SMPR1 =
        (4u << ADC_SMPR1_SMP10_Pos) |
        (4u << ADC_SMPR1_SMP11_Pos) |
        (4u << ADC_SMPR1_SMP12_Pos);


    // -------------------------------------------------------------------------
    // Injected sequence
    //
    // JL = 2 gives 3 injected conversions.
    //
    // For 3 injected conversions on STM32F4:
    //
    //   JSQ2 -> JDR1
    //   JSQ3 -> JDR2
    //   JSQ4 -> JDR3
    //
    // Sequence:
    //   IN10 -> JDR1
    //   IN11 -> JDR2
    //   IN12 -> JDR3
    // -------------------------------------------------------------------------

    ADC1->JSQR =
        (2u  << ADC_JSQR_JL_Pos)   |
        (10u << ADC_JSQR_JSQ2_Pos) |
        (11u << ADC_JSQR_JSQ3_Pos) |
        (12u << ADC_JSQR_JSQ4_Pos);


    // -------------------------------------------------------------------------
    // Bring-up mode:
    //
    // Disable external injected trigger.
    // current_feedback_update() starts injected conversion with JSWSTART.
    // -------------------------------------------------------------------------

    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);


    // -------------------------------------------------------------------------
    // Enable ADC
    // -------------------------------------------------------------------------

    ADC1->CR2 |= ADC_CR2_ADON;


    // -------------------------------------------------------------------------
    // Clear stale flags
    // -------------------------------------------------------------------------

    ADC1->SR = 0u;


    // -------------------------------------------------------------------------
    // Local state
    // -------------------------------------------------------------------------

    current_adc_raw[0] = 2048u;
    current_adc_raw[1] = 2048u;
    current_adc_raw[2] = 2048u;

    adc_offset[0] = ADC_ZERO;
    adc_offset[1] = ADC_ZERO;
    adc_offset[2] = ADC_ZERO;

    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_calibrate
//
// Call with:
//   - motor stationary
//   - PWM bridge outputs disabled
//   - DRV current-sense amplifiers enabled and settled
//
// Uses software-started injected ADC samples to find zero-current offsets.
// =============================================================================

void current_feedback_calibrate(void)
{
    const uint32_t N = 512u;

    uint32_t sum_a = 0u;
    uint32_t sum_b = 0u;
    uint32_t sum_c = 0u;
    uint32_t good = 0u;

    /*
     * Let ADC and DRV current-sense amplifiers settle.
     */
    for (volatile uint32_t d = 0u; d < 200000u; d++)
    {
        __NOP();
    }

    /*
     * Throw away first samples.
     */
    for (uint32_t i = 0u; i < 32u; i++)
    {
        (void)current_feedback_sample_once();
    }

    for (uint32_t i = 0u; i < N; i++)
    {
        if (current_feedback_sample_once())
        {
            sum_a += current_adc_raw[0];
            sum_b += current_adc_raw[1];
            sum_c += current_adc_raw[2];
            good++;
        }
    }

    if (good > 0u)
    {
        adc_offset[0] = (float)sum_a / (float)good;
        adc_offset[1] = (float)sum_b / (float)good;
        adc_offset[2] = (float)sum_c / (float)good;
    }
    else
    {
        adc_offset[0] = ADC_ZERO;
        adc_offset[1] = ADC_ZERO;
        adc_offset[2] = ADC_ZERO;
    }

    /*
     * Reset runtime counters after calibration.
     */
    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_update
//
// Called once from the TIM1 update ISR.
//
// Bring-up path:
//   TIM1 ISR gives timing.
//   JSWSTART starts injected ADC conversion.
//   JDR1/JDR2/JDR3 are read immediately after JEOC.
//
// This blocks for one injected sequence.
// At current settings that is roughly 14 us.
// =============================================================================

void current_feedback_update(void)
{
    (void)current_feedback_sample_once();
}


// =============================================================================
// current_feedback_sample_valid
// =============================================================================

bool current_feedback_sample_valid(void)
{
    return current_sample_valid;
}


// =============================================================================
// current_feedback_sample_count
// =============================================================================

uint32_t current_feedback_sample_count(void)
{
    return current_sample_count;
}


// =============================================================================
// current_feedback_missed_count
// =============================================================================

uint32_t current_feedback_missed_count(void)
{
    return current_missed_count;
}


// =============================================================================
// current_feedback_get_phase_amps
//
// Converts raw ADC counts to phase current in amps using calibrated offsets.
//
// Sign depends on amplifier polarity.
// If current direction is inverted later, flip signs here.
// =============================================================================

void current_feedback_get_phase_amps(float *ia, float *ib, float *ic)
{
    float a = ((float)current_adc_raw[0] - adc_offset[0]) * AMPS_PER_COUNT;
    float b = ((float)current_adc_raw[1] - adc_offset[1]) * AMPS_PER_COUNT;
    float c = ((float)current_adc_raw[2] - adc_offset[2]) * AMPS_PER_COUNT;

    if (ia != 0)
    {
        *ia = a;
    }

    if (ib != 0)
    {
        *ib = b;
    }

    if (ic != 0)
    {
        *ic = c;
    }
}


// =============================================================================
// current_feedback_get_dq
//
// Convert measured phase currents to d/q current using Clarke + Park.
//
// Uses ia and ib. ic is measured for debug/sanity, but not required because:
//
//   ia + ib + ic = 0
//
// Clarke:
//
//   i_alpha = ia
//   i_beta  = (ia + 2*ib) / sqrt(3)
//
// Park:
//
//   i_d =  i_alpha*cos(theta) + i_beta*sin(theta)
//   i_q = -i_alpha*sin(theta) + i_beta*cos(theta)
//
// This matches the inverse Park convention used in pwm_apply_vq().
// =============================================================================

void current_feedback_get_dq(float theta, float *i_d, float *i_q)
{
    float ia = 0.0f;
    float ib = 0.0f;

    current_feedback_get_phase_amps(&ia, &ib, 0);

    float i_alpha = ia;
    float i_beta  = (ia + (2.0f * ib)) * 0.57735026919f;

    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    if (i_d != 0)
    {
        *i_d = (i_alpha * cos_t) + (i_beta * sin_t);
    }

    if (i_q != 0)
    {
        *i_q = (-i_alpha * sin_t) + (i_beta * cos_t);
    }
}


// =============================================================================
// Offset accessors
// =============================================================================

float current_feedback_get_offset_a(void)
{
    return adc_offset[0];
}


float current_feedback_get_offset_b(void)
{
    return adc_offset[1];
}


float current_feedback_get_offset_c(void)
{
    return adc_offset[2];
}
