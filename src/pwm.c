// pwm.c — TIM1 3-phase PWM for DRV8353RS-EVM gate driver (3x PWM mode)
// STM32F411 bare metal
//
// Owns TIM1 and the 3 PWM GPIO pins. Called once from main() at startup.
//
// DRV8353 is configured in 3x PWM mode (DRV_CFG_DRIVER_CONTROL = 0x0020):
//   - INHA/INHB/INHC carry PWM from TIM1 CH1/CH2/CH3
//   - INLA/INLB/INLC are per-phase ENABLES, tied high (3.3 V) in hardware
//   - The DRV generates the complementary low-side drive internally and
//     inserts dead time per OCP_CONTROL.DEAD_TIME (100 ns configured).
//   - Therefore: NO TIM1 complementary channels, NO TIM1 dead time needed.
//
// Pin map (AF1 = TIM1 on STM32F411, board_f411.h):
//   PA8  / TIM1_CH1 -> INHA  (EVM J2-2)
//   PA9  / TIM1_CH2 -> INHB  (EVM J2-6)
//   PA10 / TIM1_CH3 -> INHC  (EVM J2-10)
//
// Pins this module must NEVER touch (owned by DRV SPI / control):
//   PA7 = SPI1_MOSI, PB0 = DRV ENABLE, PB1 = DRV nFAULT
//
// Timing:
//   TIM1 clock = APB2 timer clock. VERIFY against your clock_init():
//     APB2 = 100 MHz -> ARR = 2499 gives 100e6/(2*2500) = 20.0 kHz
//     APB2 =  84 MHz -> ARR = 2099 gives  84e6/(2*2100) = 20.0 kHz
//   Center-aligned for clean mid-period current sampling (ADC trigger later).
//
// MOE starts cleared — outputs disabled until pwm_enable().
// pwm_disable() on fault, idle, or any unsafe condition.

#include "pwm.h"
#include "board_f411.h"
#include "stm32f4xx.h"
#include <math.h>

// ── Timer geometry — SET FOR YOUR APB2 CLOCK ─────────────────────────────────
#define PWM_ARR      2499u            // 100 MHz APB2 -> 20 kHz (use 2099 @ 84 MHz)
#define PWM_CENTER   (PWM_ARR / 2u)   // 50% duty = zero net phase voltage

// ── Bus voltage — set to match bench supply ──────────────────────────────────
#define V_BUS        12.0f            // volts — start low for safety

// ─────────────────────────────────────────────────────────────────────────────
// pwm_init — configure TIM1 + PA8/9/10, leave outputs disabled (MOE=0)
// ─────────────────────────────────────────────────────────────────────────────
void pwm_init(void)
{
    // ── Clocks ───────────────────────────────────────────────────────────────
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    // ── PA8 / PA9 / PA10 -> AF1 (TIM1 CH1/CH2/CH3) ──────────────────────────
    // ONLY these three pins. PA7/PB0/PB1 belong to the DRV SPI/control path.
    GPIOA->MODER  &= ~((3u << (PIN_PWM_A * 2u)) |
                       (3u << (PIN_PWM_B * 2u)) |
                       (3u << (PIN_PWM_C * 2u)));
    GPIOA->MODER  |=  ((2u << (PIN_PWM_A * 2u)) |
                       (2u << (PIN_PWM_B * 2u)) |
                       (2u << (PIN_PWM_C * 2u)));

    // Medium speed — 20 kHz edges on jumper wires; avoid high-speed ringing
    // (same lesson as the DRV SPI bring-up).
    GPIOA->OSPEEDR &= ~((3u << (PIN_PWM_A * 2u)) |
                        (3u << (PIN_PWM_B * 2u)) |
                        (3u << (PIN_PWM_C * 2u)));
    GPIOA->OSPEEDR |=  ((1u << (PIN_PWM_A * 2u)) |
                        (1u << (PIN_PWM_B * 2u)) |
                        (1u << (PIN_PWM_C * 2u)));

    // AFR[1] covers pins 8..15: nibble index = pin - 8
    GPIOA->AFR[1] &= ~((0xFu << ((PIN_PWM_A - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_B - 8u) * 4u)) |
                       (0xFu << ((PIN_PWM_C - 8u) * 4u)));
    GPIOA->AFR[1] |=  ((1u   << ((PIN_PWM_A - 8u) * 4u)) |
                       (1u   << ((PIN_PWM_B - 8u) * 4u)) |
                       (1u   << ((PIN_PWM_C - 8u) * 4u)));

    // ── TIM1 base — center-aligned mode 1, 20 kHz ────────────────────────────
    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;

    // In center-aligned mode an update event fires at BOTH overflow (peak)
    // and underflow (valley) = 2x PWM frequency. RCR=1 halves that back to
    // once per PWM period for the control ISR.
    TIM1->RCR = 1;

    // ── CH1/CH2/CH3 — PWM mode 1, preload enabled ────────────────────────────
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;

    // ── Initial duty: 50% — zero net voltage vector ──────────────────────────
    TIM1->CCR1 = PWM_CENTER;
    TIM1->CCR2 = PWM_CENTER;
    TIM1->CCR3 = PWM_CENTER;

    // ── Enable the 3 main outputs only (no complementary channels) ───────────
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    // ── BDTR — MOE=0: outputs held inactive until pwm_enable() ───────────────
    TIM1->BDTR = 0;

    // ── Timer starts disabled; main() calls pwm_enable() after config ────────
    // The PWM output happens in hardware (CCR -> pin). Control comes from
    // TIM10's ISR, which calls pwm_apply_vq() to update the CCRs.
    TIM1->CR1 |= TIM_CR1_CEN;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_enable / pwm_disable — gate TIM1 outputs via MOE
// With MOE=0 the INHx inputs sit inactive-low; in 3x mode with INLx high the
// DRV holds the phase in low-side-on (brake-low). Use DRV COAST/ENABLE for Hi-Z.
// ─────────────────────────────────────────────────────────────────────────────
void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_set_duty — set CCR for one phase. phase: 0=A, 1=B, 2=C. duty: 0..PWM_ARR
// ─────────────────────────────────────────────────────────────────────────────
void pwm_set_duty(uint8_t phase, uint16_t duty)
{
    if (duty > PWM_ARR) duty = PWM_ARR;
    switch (phase) {
        case 0: TIM1->CCR1 = duty; break;
        case 1: TIM1->CCR2 = duty; break;
        case 2: TIM1->CCR3 = duty; break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// volts_to_duty — phase voltage -> CCR value
//   +V_BUS -> PWM_ARR (100%), 0 V -> PWM_CENTER (50%), -V_BUS -> 0 (0%)
// ─────────────────────────────────────────────────────────────────────────────
uint16_t volts_to_duty(float v)
{
    float normalized = v / V_BUS;
    if (normalized >  1.0f) normalized =  1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    return (uint16_t)(normalized * (float)PWM_CENTER + (float)PWM_CENTER);
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_apply_vq — inverse Park + inverse Clarke
//   [v_d, v_q] -(theta)-> [v_alpha, v_beta] -> [va, vb, vc] -> CCRx
// Bring-up usage: v_d = 0, v_q from current loop, theta = electrical angle.
// ─────────────────────────────────────────────────────────────────────────────
void pwm_apply_vq(float v_q, float v_d, float theta)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);
    float v_alpha = v_d * cos_t - v_q * sin_t;
    float v_beta  = v_d * sin_t + v_q * cos_t;

    float va =  v_alpha;
    float vb = -0.5f * v_alpha + 0.8660254f * v_beta;
    float vc = -0.5f * v_alpha - 0.8660254f * v_beta;

    pwm_set_duty(0, volts_to_duty(va));
    pwm_set_duty(1, volts_to_duty(vb));
    pwm_set_duty(2, volts_to_duty(vc));
}