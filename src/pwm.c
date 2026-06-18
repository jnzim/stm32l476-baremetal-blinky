
// pwm.c — TIM1 3-phase PWM for DRV8353RS-EVM gate driver, 3x PWM mode
// STM32F411/F446 bare metal
//
// Ownership:
//   This module owns:
//     - TIM1 PWM setup
//     - PA8 / PA9 / PA10 PWM pins
//     - phase voltage -> CCR conversion
//     - inverse Park / inverse Clarke voltage output
//
// DRV8353 3x PWM mode:
//   DRIVER_CONTROL = 0x0020
//
//   INHA / INHB / INHC receive PWM from TIM1 CH1/CH2/CH3.
//   INLA / INLB / INLC are per-phase enables and are tied high in hardware.
//   The DRV generates complementary low-side drive internally.
//
// Therefore:
//   - No TIM1 complementary outputs are used.
//   - No STM32 dead-time insertion is used.
//   - TIM1 only drives CH1 / CH2 / CH3.
//
// Pin map:
//   PA8  / TIM1_CH1 -> INHA
//   PA9  / TIM1_CH2 -> INHB
//   PA10 / TIM1_CH3 -> INHC
//
// Pins this module must not touch:
//   PA5 / PA6 / PA7 = DRV SPI1
//   PB0             = DRV ENABLE
//   PB1             = DRV nFAULT
//
// PWM timing:
//   Center-aligned PWM.
//   APB2 timer clock assumed 100 MHz.
//   ARR = 2499 gives:
//
//      f_pwm = 100 MHz / (2 * (2499 + 1)) = 20 kHz
//
// Note:
//   If APB2 timer clock is 84 MHz, use ARR = 2099 for 20 kHz.
//
// Bring-up status:
//   pwm_apply_vq() convention has been proven with open-loop voltage-vector
//   rotation. Do not change signs casually. Encoder alignment is the next place
//   where convention/offset must be handled.

#include "pwm.h"
#include "board_f411.h"
#include "stm32f4xx.h"

#include <math.h>
#include <stdint.h>


// =============================================================================
// PWM geometry
// =============================================================================

#define PWM_ARR      2499u
#define PWM_CENTER   (PWM_ARR / 2u)


// =============================================================================
// Bus voltage
//
// Set this to match the bench supply.
// During bring-up this has been 12 V.
// =============================================================================

#define V_BUS        12.0f


void pwm_init(void)
{
    // -------------------------------------------------------------------------
    // Enable peripheral clocks
    // -------------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    (void)RCC->APB2ENR;
    (void)RCC->AHB1ENR;


    // -------------------------------------------------------------------------
    // Configure PA8 / PA9 / PA10 as AF1 for TIM1_CH1/CH2/CH3
    // -------------------------------------------------------------------------

    GPIOA->MODER &= ~((3u << (PIN_PWM_A * 2u)) |
                      (3u << (PIN_PWM_B * 2u)) |
                      (3u << (PIN_PWM_C * 2u)));

    GPIOA->MODER |=  ((2u << (PIN_PWM_A * 2u)) |
                      (2u << (PIN_PWM_B * 2u)) |
                      (2u << (PIN_PWM_C * 2u)));

    GPIOA->OSPEEDR &= ~((3u << (PIN_PWM_A * 2u)) |
                        (3u << (PIN_PWM_B * 2u)) |
                        (3u << (PIN_PWM_C * 2u)));

    GPIOA->OSPEEDR |=  ((1u << (PIN_PWM_A * 2u)) |
                        (1u << (PIN_PWM_B * 2u)) |
                        (1u << (PIN_PWM_C * 2u)));

    GPIOA->AFR[1] &= ~((0xFu << ((PIN_PWM_A - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_B - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_C - 8u) * 4u)));

    GPIOA->AFR[1] |=  ((1u << ((PIN_PWM_A - 8u) * 4u)) |
                       (1u << ((PIN_PWM_B - 8u) * 4u)) |
                       (1u << ((PIN_PWM_C - 8u) * 4u)));


    // -------------------------------------------------------------------------
    // Clean TIM1 config
    // -------------------------------------------------------------------------

    TIM1->CR1  = 0u;
    TIM1->CR2  = 0u;
    TIM1->SMCR = 0u;
    TIM1->DIER = 0u;
    TIM1->CCER = 0u;
    TIM1->BDTR = 0u;


    // -------------------------------------------------------------------------
    // TIM1 base setup
    //
    // Center-aligned mode 1.
    // RCR = 1 gives one update interrupt per full PWM period.
    // -------------------------------------------------------------------------

    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->PSC = 0u;
    TIM1->ARR = PWM_ARR;
    TIM1->RCR = 1u;


    // -------------------------------------------------------------------------
    // TIM1 CH1/CH2/CH3 PWM outputs
    //
    // CH1/2/3:
    //   PWM mode 1, preload enabled.
    //
    // CH4:
    //   Internal compare trigger for ADC injected conversion.
    //   Toggle mode gives a real OC4 event edge.
    //   No GPIO is configured for CH4.
    // -------------------------------------------------------------------------

    TIM1->CCMR1 =
        TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
        TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    // TIM1->CCMR2 =
    //     TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE |
    //     TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1;    // OC4M = 011 toggle mode

    TIM1->CCMR2 =
    TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE |
    TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;    /* OC4M = 110 PWM mode 1 */


    // -------------------------------------------------------------------------
    // Initial phase duty: 50%
    // -------------------------------------------------------------------------

    TIM1->CCR1 = PWM_CENTER;
    TIM1->CCR2 = PWM_CENTER;
    TIM1->CCR3 = PWM_CENTER;


    // -------------------------------------------------------------------------
    // TIM1 CH4 ADC trigger at PWM center
    // -------------------------------------------------------------------------

    //TIM1->CCR4 = PWM_CENTER;
TIM1->CCR4 = 1u;

    // -------------------------------------------------------------------------
    // Enable CH1/CH2/CH3 PWM compare outputs and CH4 compare event.
    //
    // CH4 has no GPIO configured, so CC4E does not drive a physical pin.
    // -------------------------------------------------------------------------

    TIM1->CCER = TIM_CCER_CC1E |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3E |
                 TIM_CCER_CC4E;


    // -------------------------------------------------------------------------
    // Keep bridge outputs disabled until pwm_enable().
    //
    // TIM1 still runs, so ADC calibration can receive TIM1_CC4 triggers.
    // -------------------------------------------------------------------------

    TIM1->BDTR = 0u;


    // -------------------------------------------------------------------------
    // Force preload transfer, clear flags, enable TIM1 update interrupt.
    // -------------------------------------------------------------------------

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0u;

    TIM1->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);


    // -------------------------------------------------------------------------
    // Start TIM1.
    // -------------------------------------------------------------------------

    TIM1->CR1 |= TIM_CR1_CEN;
}

   


// =============================================================================
// pwm_enable / pwm_disable
//
// Uses TIM1 BDTR.MOE to enable/disable the PWM outputs.
// =============================================================================

void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}


void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}


// =============================================================================
// pwm_set_duty — write one phase CCR
//
// phase:
//   0 = A / TIM1_CH1 / PA8
//   1 = B / TIM1_CH2 / PA9
//   2 = C / TIM1_CH3 / PA10
//
// duty:
//   CCR count in range 0..PWM_ARR
// =============================================================================

void pwm_set_duty(uint8_t phase, uint16_t duty)
{
    if (duty > PWM_ARR)
        duty = PWM_ARR;

    switch (phase)
    {
        case 0:
            TIM1->CCR1 = duty;
            break;

        case 1:
            TIM1->CCR2 = duty;
            break;

        case 2:
            TIM1->CCR3 = duty;
            break;

        default:
            break;
    }
}


// =============================================================================
// volts_to_duty — phase voltage command -> CCR count
//
// Mapping:
//
//   -V_BUS -> 0
//    0 V   -> PWM_CENTER
//   +V_BUS -> PWM_ARR approximately
//
// This is centered sinusoidal PWM scaling. It is intentionally simple for
// bring-up. Later, this can be replaced or extended with SVPWM.
// =============================================================================

uint16_t volts_to_duty(float v)
{
    float normalized = v / V_BUS;

    if (normalized > 1.0f)
        normalized = 1.0f;

    if (normalized < -1.0f)
        normalized = -1.0f;

    return (uint16_t)(normalized * (float)PWM_CENTER + (float)PWM_CENTER);
}


// =============================================================================
// pwm_apply_phase_volts — apply three phase voltage commands
//
// va/vb/vc are phase voltage commands centered around zero.
// This function converts them to TIM1 CCR values.
// =============================================================================

void pwm_apply_phase_volts(float va, float vb, float vc)
{
    pwm_set_duty(0, volts_to_duty(va));
    pwm_set_duty(1, volts_to_duty(vb));
    pwm_set_duty(2, volts_to_duty(vc));
}


// =============================================================================
// pwm_apply_vq — inverse Park + inverse Clarke voltage output
//
// Inputs:
//
//   v_q:
//     q-axis voltage command.
//     During open-loop bring-up this is a fixed small value, e.g. 1.5 V.
//     Later this comes from the q-axis current-loop PI controller.
//
//   v_d:
//     d-axis voltage command.
//     During bring-up this is usually 0.
//     Later this comes from the d-axis current-loop PI controller.
//
//   theta:
//     electrical angle in radians.
//     During open-loop bring-up this is a synthetic ramp.
//     Later this must come from:
//
//        theta_elec = pole_pairs * theta_mech + encoder_offset
//
// Transform:
//
//   [v_d, v_q] + theta_elec
//        -> inverse Park
//        -> [v_alpha, v_beta]
//        -> inverse Clarke
//        -> [va, vb, vc]
//        -> centered PWM CCR values
//
// This convention has been proven by open-loop rotation.
// Freeze this math until encoder alignment testing.
// =============================================================================

void pwm_apply_vq(float v_q, float v_d, float theta)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    // Inverse Park transform.
    float v_alpha = (v_d * cos_t) - (v_q * sin_t);
    float v_beta  = (v_d * sin_t) + (v_q * cos_t);

    // Inverse Clarke transform.
    float va =  v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540378f * v_beta;
    float vc = -0.5f * v_alpha - 0.86602540378f * v_beta;

    pwm_apply_phase_volts(va, vb, vc);
}
