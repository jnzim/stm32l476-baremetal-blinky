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
// Sampling mode:
//   ADC1 injected sequence triggered by TIM1_CC4 (hardware trigger).
//   TIM1 CH4 CCR4 = PWM_CENTER fires at the PWM quiet point in center-aligned
//   mode. ADC converts automatically — no software start in the ISR.
//
//   current_feedback_update() is called from the TIM1 ISR.
//   It checks JEOC and reads JDR1/JDR2/JDR3 if conversion is complete.
//
// Calibration:
//   current_feedback_calibrate() uses software-start (JSWSTART) by temporarily
//   disabling JEXTEN. Hardware trigger is restored after calibration completes.
//
// Important:
//   Do NOT use ADC_SR_JEOC from the current project headers.
//   On STM32F4/F411, JEOC is bit 2 = 0x04.
//   Project headers have been seen with wrong value 0x08.

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

#define SHUNT_R            0.007f
#define SHUNT_GAIN         10.0f
#define VREF               3.3f
#define ADC_COUNTS         4096.0f
#define ADC_ZERO           2048.0f

#define AMPS_PER_COUNT     (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))
#define ADC_SMP_84_CYCLES  (4u)


#if defined(ADC_SR_JEOC)
#if ADC_SR_JEOC != (1u << 2)
#error "ADC_SR_JEOC is wrong for STM32F4/F411"
#endif
#endif


// =============================================================================
// ADC status bits
//
// STM32F4 ADC SR bits:
//   AWD   = bit 0 = 0x01
//   EOC   = bit 1 = 0x02
//   JEOC  = bit 2 = 0x04
//   JSTRT = bit 3 = 0x08
//   STRT  = bit 4 = 0x10
//   OVR   = bit 5 = 0x20
// =============================================================================

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

static volatile bool     current_sample_valid  = false;
static volatile uint32_t current_sample_count  = 0u;
static volatile uint32_t current_missed_count  = 0u;


// =============================================================================
// current_feedback_sample_once
//
// Software-start one injected ADC sequence and read JDR1/JDR2/JDR3.
// Used only by current_feedback_calibrate().
// Temporarily disables JEXTEN so JSWSTART owns the conversion.
// =============================================================================

static bool current_feedback_sample_once(void)
{
    uint32_t timeout = 10000u;
    

    ADC1->CR2 &= ~ADC_CR2_JEXTEN;
    ADC1->SR  &= ~ADC_SR_JEOC_BIT;
    ADC1->CR2 |=  ADC_CR2_JSWSTART;

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
// =============================================================================

void current_feedback_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    GPIOC->MODER |=  (3u << (PIN_CUR_A * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_B * 2u));
    GPIOC->MODER |=  (3u << (PIN_CUR_C * 2u));

    GPIOC->PUPDR &= ~(3u << (PIN_CUR_A * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_B * 2u));
    GPIOC->PUPDR &= ~(3u << (PIN_CUR_C * 2u));

    ADC->CCR  = ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0u;
    ADC1->CR2 = 0u;
    ADC1->SR  = 0u;

    ADC1->CR1 |= ADC_CR1_SCAN;

    ADC1->SMPR1 &= ~((7u << 0) | (7u << 3) | (7u << 6));
    ADC1->SMPR1 |=  ((ADC_SMP_84_CYCLES << 0) |
                     (ADC_SMP_84_CYCLES << 3) |
                     (ADC_SMP_84_CYCLES << 6));

    ADC1->JSQR =
        (2u  << ADC_JSQR_JL_Pos)   |
        (10u << ADC_JSQR_JSQ2_Pos) |
        (11u << ADC_JSQR_JSQ3_Pos) |
        (12u << ADC_JSQR_JSQ4_Pos);

    /*
     * Hardware trigger: TIM1_CC4, rising edge.
     *
     * JEXTSEL = 4 = TIM1_CC4 on STM32F411.
     * JEXTEN  = 1 = rising edge.
     *
     * TIM1 CCR4 = PWM_CENTER (1249) in center-aligned mode.
     * Fires at the PWM quiet point — all switches fully settled.
     */
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    // ADC1->CR2 |=  ((4u << ADC_CR2_JEXTSEL_Pos) |
    //                (1u << ADC_CR2_JEXTEN_Pos));
    ADC1->CR2 |=  ((0u << ADC_CR2_JEXTSEL_Pos) |
               (1u << ADC_CR2_JEXTEN_Pos));

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SR   = 0u;

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
// =============================================================================

void current_feedback_calibrate(void)
{
    const uint32_t N = 512u;

    uint32_t sum_a = 0u;
    uint32_t sum_b = 0u;
    uint32_t sum_c = 0u;
    uint32_t good  = 0u;

    for (volatile uint32_t d = 0u; d < 200000u; d++) { __NOP(); }

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
     * Re-enable hardware trigger after software-start calibration.
     */
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    // ADC1->CR2 |=  ((4u << ADC_CR2_JEXTSEL_Pos) |
    //                (1u << ADC_CR2_JEXTEN_Pos));
ADC1->CR2 |=  ((0u << ADC_CR2_JEXTSEL_Pos) |
               (1u << ADC_CR2_JEXTEN_Pos));
    ADC1->SR = 0u;

    /* Force ADC off then back on to clear any stuck state from JSWSTART */
    ADC1->CR2 &= ~ADC_CR2_ADON;
    for (volatile uint32_t d = 0u; d < 1000u; d++) { __NOP(); }
    ADC1->CR2 |=  ADC_CR2_ADON;
    for (volatile uint32_t d = 0u; d < 1000u; d++) { __NOP(); }
    ADC1->SR   = 0u;

    current_sample_valid = false;
    current_sample_count = 0u;
    current_missed_count = 0u;
}


// =============================================================================
// current_feedback_update
//
// Called from TIM1 ISR at 20 kHz.
// Checks JEOC (injected end-of-conversion) flag set by TIM1_CC4 hardware
// trigger. Reads JDR1/JDR2/JDR3 if conversion is complete.
// Non-blocking — if ADC has not finished, keeps previous values.
// =============================================================================

void current_feedback_update(void)
{
    if (ADC1->SR & ADC_SR_JEOC_BIT)
    {
        current_adc_raw[0] = ADC1->JDR1;
        current_adc_raw[1] = ADC1->JDR2;
        current_adc_raw[2] = ADC1->JDR3;
        ADC1->SR          &= ~ADC_SR_JEOC_BIT;
        current_sample_valid = true;
        current_sample_count++;
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
// =============================================================================

void current_feedback_get_phase_amps(float *ia, float *ib, float *ic)
{
    float a = ((float)current_adc_raw[0] - adc_offset[0]) * AMPS_PER_COUNT;
    float b = ((float)current_adc_raw[1] - adc_offset[1]) * AMPS_PER_COUNT;
    float c = -(a + b);

    if (ia != 0) { *ia = a; }
    if (ib != 0) { *ib = b; }
    if (ic != 0) { *ic = c; }
}


// =============================================================================
// current_feedback_get_dq
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

    if (i_d != 0) { *i_d = ( i_alpha * cos_t) + (i_beta * sin_t); }
    if (i_q != 0) { *i_q = (-i_alpha * sin_t) + (i_beta * cos_t); }
}


// =============================================================================
// Offset accessors
// =============================================================================

float current_feedback_get_offset_a(void) { return adc_offset[0]; }
float current_feedback_get_offset_b(void) { return adc_offset[1]; }
float current_feedback_get_offset_c(void) { return adc_offset[2]; }