// pwm.c — TIM1 3-phase complementary PWM for DRV8353RS-EVM gate driver
//
// Owns all TIM1 configuration and the 6 PWM GPIO pins.
// Called once from main() during startup.
//
// Pin map (AF1 = TIM1 on STM32F446RE):
//   PA8  → TIM1_CH1  → INHA  Phase A high-side
//   PA7  → TIM1_CH1N → INLA  Phase A low-side
//   PA9  → TIM1_CH2  → INHB  Phase B high-side
//   PB0  → TIM1_CH2N → INLB  Phase B low-side
//   PA10 → TIM1_CH3  → INHC  Phase C high-side
//   PB1  → TIM1_CH3N → INLC  Phase C low-side
//
// TIM1 clock:  APB2=90MHz × 2 = 180MHz
// ARR=4499, center-aligned → period = 2×4500 / 180MHz = 50µs = 20kHz
//
// Dead time: NOT configured — DRV8353RS inserts dead time automatically
// via VGS monitoring (TDRIVE state machine). No shoot-through risk.
//
// MOE (Main Output Enable) starts cleared — all outputs disabled at startup.
// Call pwm_enable() from drive state machine when entering STATE_ENABLED.
// Call pwm_disable() on fault, STATE_IDLE, or any unsafe condition.

#include "pwm.h"
#include "stm32f4xx.h"
#include <math.h>

// ── Bus voltage — set to match bench supply ───────────────────────────────────
// Used to normalize v_q/v_d to duty cycle range
// Update when bus voltage changes
#define V_BUS   12.0f                           // volts — start low for safety

// ── ARR center point — 50% duty = zero voltage ───────────────────────────────
#define PWM_CENTER  2249u                       // ARR/2 = 4499/2

// ─────────────────────────────────────────────────────────────────────────────
// pwm_init — configure TIM1 and GPIO, leave outputs disabled
// ─────────────────────────────────────────────────────────────────────────────
void pwm_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;                         // flush write pipeline

    // ── GPIOA: PA7, PA8, PA9, PA10 → AF1 (TIM1) ─────────────────────────────
    GPIOA->MODER  &= ~((3u << 14) | (3u << 16) | (3u << 18) | (3u << 20));
    GPIOA->MODER  |=  ((2u << 14) | (2u << 16) | (2u << 18) | (2u << 20));
    GPIOA->OSPEEDR |= ((3u << 14) | (3u << 16) | (3u << 18) | (3u << 20));
    GPIOA->AFR[0] &= ~(0xFu << 28);
    GPIOA->AFR[0] |=  (1u   << 28);                                // PA7 = AF1
    GPIOA->AFR[1] &= ~((0xFu << 0) | (0xFu << 4) | (0xFu << 8));
    GPIOA->AFR[1] |=  ((1u   << 0) | (1u   << 4) | (1u   << 8)); // PA8,9,10 = AF1

    // ── GPIOB: PB0, PB1 → AF1 (TIM1) ────────────────────────────────────────
    GPIOB->MODER  &= ~((3u << 0) | (3u << 2));
    GPIOB->MODER  |=  ((2u << 0) | (2u << 2));
    GPIOB->OSPEEDR |= ((3u << 0) | (3u << 2));
    GPIOB->AFR[0] &= ~((0xFu << 0) | (0xFu << 4));
    GPIOB->AFR[0] |=  ((1u   << 0) | (1u   << 4));                // PB0,1 = AF1

    // ── TIM1 base — center-aligned mode, 20kHz ───────────────────────────────
    TIM1->CR1  = TIM_CR1_CMS_0;
    TIM1->PSC  = 0;
    TIM1->ARR  = 4499;

    // ── PWM channels — mode 1, preload enabled ───────────────────────────────
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;

    // ── Initial duty: 50% — zero net voltage vector ───────────────────────────
    TIM1->CCR1 = PWM_CENTER;
    TIM1->CCR2 = PWM_CENTER;
    TIM1->CCR3 = PWM_CENTER;

    // ── Enable all 6 outputs, active high polarity ────────────────────────────
    TIM1->CCER = TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E  | TIM_CCER_CC3NE;

    // ── BDTR — MOE=0, outputs disabled until pwm_enable() ────────────────────
    TIM1->BDTR = 0;

    // ── Update interrupt — current/velocity loop ISR ──────────────────────────
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->SR   = 0;

    // ── NVIC ──────────────────────────────────────────────────────────────────
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_enable — set MOE, allow PWM signals to reach gate driver
// ─────────────────────────────────────────────────────────────────────────────
void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_disable — clear MOE, all gate driver inputs go low
// ─────────────────────────────────────────────────────────────────────────────
void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_set_duty — set duty cycle for one phase
// phase: 0=A, 1=B, 2=C   duty: 0–4499
// ─────────────────────────────────────────────────────────────────────────────
void pwm_set_duty(uint8_t phase, uint16_t duty)
{
    if (duty > 4499) duty = 4499;
    switch (phase) {
        case 0: TIM1->CCR1 = duty; break;
        case 1: TIM1->CCR2 = duty; break;
        case 2: TIM1->CCR3 = duty; break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// volts_to_duty — convert phase voltage to TIM1 CCR value
//
// Maps voltage linearly to duty cycle:
//   +V_BUS → 4499 (100%)
//    0V    → 2249 (50%, zero current)
//   -V_BUS → 0    (0%)
//
// v: phase voltage in volts, range [-V_BUS, +V_BUS]
// returns: CCR value 0–4499
// ─────────────────────────────────────────────────────────────────────────────
uint16_t volts_to_duty(float v)
{
    float normalized = v / V_BUS;
    if (normalized >  1.0f) normalized =  1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    return (uint16_t)(normalized * (float)PWM_CENTER + (float)PWM_CENTER);
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_apply_vq — inverse Park + inverse Clarke + SVM stub
//
// Converts v_q and v_d (rotor frame) to 3-phase duty cycles.
// theta: electrical rotor angle in radians
//
// Signal chain:
//   [v_d, v_q] → inverse Park → [v_alpha, v_beta] → inverse Clarke → [va, vb, vc]
//   → volts_to_duty() → CCR1/CCR2/CCR3
//
// For initial hardware bringup with encoder:
//   v_d = 0 (no field weakening)
//   v_q = output of current loop
//   theta = electrical angle from encoder
// ─────────────────────────────────────────────────────────────────────────────
void pwm_apply_vq(float v_q, float v_d, float theta)
{
    // ── Inverse Park transform ────────────────────────────────────────────────
    // Rotates from rotor (d,q) frame back to stationary (alpha,beta) frame
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);
    float v_alpha = v_d * cos_t - v_q * sin_t;
    float v_beta  = v_d * sin_t + v_q * cos_t;

    // ── Inverse Clarke transform ──────────────────────────────────────────────
    // Converts 2-phase stationary (alpha,beta) to 3-phase (a,b,c)
    float va =  v_alpha;
    float vb = -0.5f * v_alpha + 0.8660254f * v_beta;   // -1/2 * α + √3/2 * β
    float vc = -0.5f * v_alpha - 0.8660254f * v_beta;   // -1/2 * α - √3/2 * β

    // ── Set duty cycles ───────────────────────────────────────────────────────
    pwm_set_duty(0, volts_to_duty(va));
    pwm_set_duty(1, volts_to_duty(vb));
    pwm_set_duty(2, volts_to_duty(vc));
}