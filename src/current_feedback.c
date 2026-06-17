// current_feedback.c — 3-phase current sense via DRV8353RS onboard shunt amplifiers
//
// Standalone module. Does NOT modify pwm.c, tim1.c, or main.c beyond two
// init calls and one read call added to main.c.
//
// Hardware confirmed:
//   PC0 / ADC1_IN10 → ISENA (phase A)
//   PC1 / ADC1_IN11 → ISENB (phase B)
//   PC2 / ADC1_IN12 → ISENC (phase C)
//   Shunt resistors R14/R15/R16 = R007 = 7 mΩ
//   DRV8353RS CSA gain = 10 V/V (default)
//   VREF = 3.3V
//   Zero current = VREF/2 = 1.65V = ADC count 2048
//
// Trigger:
//   ADC triggered by TIM1 TRGO (update event).
//   TIM1 is already running from tim1_init().
//   DMA2 Stream0 Channel0 fills current_adc_raw[3] in circular mode.
//   CPU polls TCIF0 in ISR to confirm DMA done before reading.
//
// Ownership:
//   PC0, PC1, PC2 (analog input)
//   ADC1
//   DMA2 Stream0 Channel0
//
// Must not touch:
//   PA0/PA1      — encoder TIM2
//   PA5/PA6/PA7  — DRV SPI1
//   PA8/PA9/PA10 — TIM1 PWM
//   PB0/PB1      — DRV ENABLE/nFAULT
//
// Bring-up sequence in main():
//   tim1_init();                    // already there — TIM1 running
//   pwm_init();                     // already there
//   current_feedback_init();        // ADD — sets up ADC + DMA
//   current_feedback_calibrate();   // ADD — measures zero offset, PWM off
//   pwm_enable();                   // already there
//
// In TIM1_UP_TIM10_IRQHandler() add temporarily at top:
//   while (!(DMA2->LISR & DMA_LISR_TCIF0));
//   DMA2->LIFCR = DMA_LIFCR_CTCIF0;
//   float ia, ib, ic;
//   current_feedback_get(&ia, &ib, &ic);
//   // watch ia/ib/ic in debugger or send to telemetry

#include "current_feedback.h"
#include "board_f411.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "clock.h"

// ── Constants ─────────────────────────────────────────────────────────────────

#define SHUNT_R         0.007f      // ohms — R007 confirmed on EVM
#define SHUNT_GAIN      10.0f       // V/V  — DRV8353RS default CSA gain
#define VREF            3.3f        // volts
#define ADC_COUNTS      4096.0f     // 12-bit
#define ADC_ZERO        2048.0f     // counts at zero current (VREF/2)

// Amps per ADC count after offset removal:
//   3.3 / (4096 x 0.007 x 10) = 0.01151 A/count
#define AMPS_PER_COUNT  (VREF / (ADC_COUNTS * SHUNT_R * SHUNT_GAIN))

// ── DMA result buffer ─────────────────────────────────────────────────────────
// [0] = ISENA = phase A = PC0 = IN10
// [1] = ISENB = phase B = PC1 = IN11
// [2] = ISENC = phase C = PC2 = IN12

volatile uint16_t current_adc_raw[3];

// ── Per-channel zero offset ───────────────────────────────────────────────────
// Set by current_feedback_calibrate(). Default = 2048.

float adc_offset[3] = { ADC_ZERO, ADC_ZERO, ADC_ZERO };


// =============================================================================
// current_feedback_init
//
// Call after tim1_init() and pwm_init(), before current_feedback_calibrate().
// TIM1 must be running so ADC trigger fires during calibration.
// =============================================================================
void current_feedback_init(void)
{
    // Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    // PC0/PC1/PC2 → analog
    GPIOC->MODER |=  (3u << (PIN_CUR_A * 2u))
                  |  (3u << (PIN_CUR_B * 2u))
                  |  (3u << (PIN_CUR_C * 2u));
    GPIOC->PUPDR &= ~((3u << (PIN_CUR_A * 2u))
                    | (3u << (PIN_CUR_B * 2u))
                    | (3u << (PIN_CUR_C * 2u)));

    // DMA2 Stream0 Channel0
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)current_adc_raw;
    DMA2_Stream0->NDTR = 3u;
    DMA2_Stream0->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MSIZE_0            |
        DMA_SxCR_PSIZE_0            |
        DMA_SxCR_MINC               |
        DMA_SxCR_CIRC               |
        DMA_SxCR_EN;

    // ADC prescaler
    ADC->CCR = ADC_CCR_ADCPRE_0;

    // ADC1 scan mode
    ADC1->CR1 = ADC_CR1_SCAN;

    // ADC1 continuous, DMA — no trigger for now
    ADC1->CR2 =
        ADC_CR2_DDS  |
        ADC_CR2_DMA  |
        ADC_CR2_CONT |
        ADC_CR2_ADON;

    // Sequence: 3 conversions, IN10 → IN11 → IN12
    ADC1->SQR1 = (2u << ADC_SQR1_L_Pos);
    ADC1->SQR3 =
        (10u << ADC_SQR3_SQ1_Pos) |
        (11u << ADC_SQR3_SQ2_Pos) |
        (12u << ADC_SQR3_SQ3_Pos);

    // Sample time — IN10/11/12 in SMPR1
    ADC1->SMPR1 =
        (2u << ADC_SMPR1_SMP10_Pos) |
        (2u << ADC_SMPR1_SMP11_Pos) |
        (2u << ADC_SMPR1_SMP12_Pos);

    // Start conversion — MUST be last
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

// =============================================================================
// current_feedback_calibrate
//
// Motor stationary, PWM outputs disabled (MOE=0).
// Averages 256 samples per channel → true zero-current ADC offset.
// Call before pwm_enable().
// =============================================================================

void current_feedback_calibrate(void)
{
    
     delay_ms(100);    // let DRV8353RS CSA and ADC settle
    
    const uint32_t N = 256u;
    uint32_t sum[3] = { 0u, 0u, 0u };

    for (uint32_t i = 0u; i < N; i++)
    {
        // Wait one PWM period for TIM1 to trigger ADC and DMA to complete
        for (volatile uint32_t d = 0u; d < 5000u; d++);

        while (!(DMA2->LISR & DMA_LISR_TCIF0));
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;

        sum[0] += current_adc_raw[0];
        sum[1] += current_adc_raw[1];
        sum[2] += current_adc_raw[2];
    }

    adc_offset[0] = (float)sum[0] / (float)N;
    adc_offset[1] = (float)sum[1] / (float)N;
    adc_offset[2] = (float)sum[2] / (float)N;

    // Sanity check: offsets should be near 2048.
    // If any offset is outside 1700-2300:
    //   - check 3.3V supply at PC0/PC1/PC2
    //   - check wiring from DRV ISENA/B/C to PC0/1/2
    //   - check DRV CSA gain register via SPI
}


// =============================================================================
// current_feedback_get
//
// Call from ISR after DMA complete confirmed:
//
//   while (!(DMA2->LISR & DMA_LISR_TCIF0));
//   DMA2->LIFCR = DMA_LIFCR_CTCIF0;
//   current_feedback_get(&ia, &ib, &ic);
//
// Returns amps. Positive = current into motor phase.
// Sanity check: ia + ib + ic should be ~0A.
// =============================================================================

void current_feedback_get(float *ia, float *ib, float *ic)ç
{
    *ia = ((float)current_adc_raw[0] - adc_offset[0]) * AMPS_PER_COUNT; //2086
    *ib = ((float)current_adc_raw[1] - adc_offset[1]) * AMPS_PER_COUNT; //2089
    *ic = ((float)current_adc_raw[2] - adc_offset[2]) * AMPS_PER_COUNT; //2036
}