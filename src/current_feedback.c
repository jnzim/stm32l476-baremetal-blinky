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
//   TIM1 TRGO triggers the injected conversion.
//   Results are read from JDR1/JDR2/JDR3.
//
// This version does NOT use DMA.
// This version does NOT use ADC continuous mode.
// This version does NOT use software-started conversions.
//
// TIM1 owns the sample timing.

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
// ADC injected trigger selection
//
// STM32F4 ADC injected external trigger JEXTSEL encoding:
//
//   0000: TIM1_CC4
//   0001: TIM1_TRGO
//
// We are using TIM1_TRGO for the first synchronized bring-up.
//
// TIM1 must also be configured with:
//
//   TIM1->CR2 MMS = 010, update event as TRGO
//
// =============================================================================

#define ADC_JEXTSEL_TIM1_TRGO   (1u)


// =============================================================================
// ADC injected data
//
// current_feedback_update() writes:
//
//   [0] = phase A current sense, PC0 / ADC1_IN10 / JDR1
//   [1] = phase B current sense, PC1 / ADC1_IN11 / JDR2
//   [2] = phase C current sense, PC2 / ADC1_IN12 / JDR3
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
// current_feedback_init
//
// Configure:
//   - PC0/PC1/PC2 as analog inputs
//   - ADC1 injected sequence: IN10, IN11, IN12
//   - injected trigger source: TIM1_TRGO
//
// No DMA.
// No ADC continuous mode.
// No software start.
//
// ADC waits for TIM1_TRGO.
// =============================================================================

void current_feedback_init(void)
{
    /*
     * Enable GPIOC clock.
     *
     * PC0/PC1/PC2 are the analog current feedback pins.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /*
     * Enable ADC1 clock.
     *
     * ADC1 is on APB2.
     */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;


    /*
     * PC0/PC1/PC2 analog mode.
     *
     * MODER bits:
     *   00 = input
     *   01 = output
     *   10 = alternate function
     *   11 = analog
     */
    GPIOC->MODER |=  (3u << (PIN_CUR_A * 2u));   // PC0 / ADC1_IN10
    GPIOC->MODER |=  (3u << (PIN_CUR_B * 2u));   // PC1 / ADC1_IN11
    GPIOC->MODER |=  (3u << (PIN_CUR_C * 2u));   // PC2 / ADC1_IN12

    /*
     * No pullups/pulldowns on analog inputs.
     */
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_A * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_B * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_C * 2u));


    /*
     * ADC common control register.
     *
     * ADC->CCR controls settings shared by ADC1/ADC2/ADC3.
     *
     * ADCPRE = ADC prescaler.
     *
     * ADC_CCR_ADCPRE_0 means PCLK2 / 4.
     */
    ADC->CCR = ADC_CCR_ADCPRE_0;


    /*
     * Start from clean ADC config.
     */
    ADC1->CR1 = 0u;
    ADC1->CR2 = 0u;


    /*
     * CR1 SCAN:
     *
     * Enables multi-channel sequence mode.
     *
     * For injected conversion, this lets one trigger sample:
     *
     *   IN10 -> IN11 -> IN12
     */
    ADC1->CR1 |= ADC_CR1_SCAN;


    /*
     * SMPR1 = sample time register for channels 10..18.
     *
     * 2 means 28 ADC cycles on STM32F4.
     *
     * If current readings are noisy or strange later, try a longer sample time.
     */
    ADC1->SMPR1 =
        (2u << ADC_SMPR1_SMP10_Pos) |    // IN10 / PC0 / phase A
        (2u << ADC_SMPR1_SMP11_Pos) |    // IN11 / PC1 / phase B
        (2u << ADC_SMPR1_SMP12_Pos);     // IN12 / PC2 / phase C


    /*
     * JSQR = injected sequence register.
     *
     * JL = injected sequence length:
     *
     *   JL = 0 -> 1 conversion
     *   JL = 1 -> 2 conversions
     *   JL = 2 -> 3 conversions
     *   JL = 3 -> 4 conversions
     *
     * We want 3 conversions.
     *
     * STM32F4 injected sequence alignment:
     *
     * For 3 injected conversions, use JSQ2, JSQ3, JSQ4.
     *
     * Mapping:
     *
     *   JSQ2 = IN10 -> JDR1
     *   JSQ3 = IN11 -> JDR2
     *   JSQ4 = IN12 -> JDR3
     */
    ADC1->JSQR =
        (2u  << ADC_JSQR_JL_Pos)   |     // 3 injected conversions
        (10u << ADC_JSQR_JSQ2_Pos) |     // first result: JDR1 = IN10 / PC0
        (11u << ADC_JSQR_JSQ3_Pos) |     // second result: JDR2 = IN11 / PC1
        (12u << ADC_JSQR_JSQ4_Pos);      // third result: JDR3 = IN12 / PC2


    /*
     * CR2 JEXTSEL = injected external trigger source.
     *
     * We want ADC injected conversions to start from TIM1_TRGO.
     *
     * TIM1_TRGO is generated by TIM1 when:
     *
     *   TIM1->CR2 MMS = 010
     *
     * For STM32F4 injected ADC trigger selection:
     *
     *   JEXTSEL = 0001 means TIM1_TRGO.
     */
    ADC1->CR2 &= ~ADC_CR2_JEXTSEL;
    ADC1->CR2 |=  (ADC_JEXTSEL_TIM1_TRGO << ADC_CR2_JEXTSEL_Pos);


    /*
     * CR2 JEXTEN = injected external trigger enable.
     *
     *   00 = disabled
     *   01 = rising edge
     *   10 = falling edge
     *   11 = both edges
     *
     * Use rising edge for first bring-up.
     */
    ADC1->CR2 &= ~ADC_CR2_JEXTEN;
    ADC1->CR2 |=  ADC_CR2_JEXTEN_0;


    /*
     * Enable ADC.
     *
     * No SWSTART.
     * No CONT.
     * No DMA.
     *
     * ADC now waits for TIM1_TRGO.
     */
    ADC1->CR2 |= ADC_CR2_ADON;


    /*
     * Clear stale flags.
     */
    ADC1->SR = 0u;

    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_wait_for_injected_sample
//
// Startup/calibration helper only.
//
// This blocks until TIM1-triggered injected ADC conversion completes.
// Do NOT call this from the 20 kHz ISR.
// =============================================================================

static void current_feedback_wait_for_injected_sample(void)
{
    while ((ADC1->SR & ADC_SR_JEOC) == 0u)
    {
        /*
         * Wait for TIM1-triggered injected conversion.
         *
         * If this hangs:
         *   - TIM1 TRGO is not configured
         *   - ADC JEXTSEL is wrong
         *   - ADC JEXTEN is not enabled
         *   - TIM1 is not running
         */
    }

    current_adc_raw[0] = ADC1->JDR1;
    current_adc_raw[1] = ADC1->JDR2;
    current_adc_raw[2] = ADC1->JDR3;

    ADC1->SR &= ~ADC_SR_JEOC;
}


// =============================================================================
// current_feedback_calibrate
//
// Call with:
//   - motor stationary
//   - PWM outputs disabled
//   - DRV current-sense amplifiers enabled and settled
//
// Uses TIM1-triggered injected ADC samples to find zero-current offsets.
// Blocking is okay here because this runs once during startup.
// =============================================================================

void current_feedback_calibrate(void)
{
    delay_ms(100);
    const uint32_t N = 512u;

    uint32_t sum_a = 0u;
    uint32_t sum_b = 0u;
    uint32_t sum_c = 0u;

    /*
     * Let ADC and DRV current-sense amplifiers settle.
     */
    for (volatile uint32_t d = 0u; d < 200000u; d++)
    {
        __NOP();
    }

    /*
     * Discard stale injected conversion flag before averaging.
     */
    ADC1->SR &= ~ADC_SR_JEOC;

    for (uint32_t i = 0u; i < N; i++)
    {
        current_feedback_wait_for_injected_sample();

        sum_a += current_adc_raw[0];
        sum_b += current_adc_raw[1];
        sum_c += current_adc_raw[2];
    }

    adc_offset[0] = (float)sum_a / (float)N;
    adc_offset[1] = (float)sum_b / (float)N;
    adc_offset[2] = (float)sum_c / (float)N;

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
// Non-blocking.
//
// For synchronized current sampling:
//   TIM1 TRGO starts ADC injected conversion.
//   ADC sets JEOC when injected sequence is complete.
//   This function copies JDR1/JDR2/JDR3 into current_adc_raw[].
//
// If no sample is ready, do not wait.
// =============================================================================

void current_feedback_update(void)
{
    /*
     * JEOC = injected end-of-conversion flag.
     *
     * This means:
     *   - TIM1_TRGO occurred
     *   - injected ADC sequence completed
     *   - JDR1/JDR2/JDR3 contain fresh results
     */
    if ((ADC1->SR & ADC_SR_JEOC) != 0u)
    {
        current_adc_raw[0] = ADC1->JDR1;
        current_adc_raw[1] = ADC1->JDR2;
        current_adc_raw[2] = ADC1->JDR3;

        ADC1->SR &= ~ADC_SR_JEOC;

        current_sample_count++;
        current_sample_valid = true;
    }
    else
    {
        current_missed_count++;
        current_sample_valid = false;
    }
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
        *ia = a;

    if (ib != 0)
        *ib = b;

    if (ic != 0)
        *ic = c;
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
        *i_d = (i_alpha * cos_t) + (i_beta * sin_t);

    if (i_q != 0)
        *i_q = (-i_alpha * sin_t) + (i_beta * cos_t);
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