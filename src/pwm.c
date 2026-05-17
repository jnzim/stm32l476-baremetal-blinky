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
    // MODER: alternate function = 10
    GPIOA->MODER  &= ~((3u << 14) | (3u << 16) | (3u << 18) | (3u << 20));
    GPIOA->MODER  |=  ((2u << 14) | (2u << 16) | (2u << 18) | (2u << 20));

    // Speed: very high on all four pins
    GPIOA->OSPEEDR |= ((3u << 14) | (3u << 16) | (3u << 18) | (3u << 20));

    // AFR[0]: PA7 → bits [31:28] = AF1
    GPIOA->AFR[0] &= ~(0xFu << 28);
    GPIOA->AFR[0] |=  (1u   << 28);

    // AFR[1]: PA8 → [3:0], PA9 → [7:4], PA10 → [11:8] = AF1
    GPIOA->AFR[1] &= ~((0xFu << 0) | (0xFu << 4) | (0xFu << 8));
    GPIOA->AFR[1] |=  ((1u   << 0) | (1u   << 4) | (1u   << 8));

    // ── GPIOB: PB0, PB1 → AF1 (TIM1) ────────────────────────────────────────
    GPIOB->MODER  &= ~((3u << 0) | (3u << 2));
    GPIOB->MODER  |=  ((2u << 0) | (2u << 2));
    GPIOB->OSPEEDR |= ((3u << 0) | (3u << 2));

    // AFR[0]: PB0 → [3:0], PB1 → [7:4] = AF1
    GPIOB->AFR[0] &= ~((0xFu << 0) | (0xFu << 4));
    GPIOB->AFR[0] |=  ((1u   << 0) | (1u   << 4));

    // ── TIM1 base — center-aligned mode, 20kHz ───────────────────────────────
    // CMS_0: counter counts 0→ARR→0, update IRQ fires at bottom (count=0)
    TIM1->CR1  = TIM_CR1_CMS_0;
    TIM1->PSC  = 0;                              // prescaler ÷1 = 180MHz
    TIM1->ARR  = 4499;                           // 180MHz / (2×4500) = 20kHz

    // ── PWM channels — mode 1, preload enabled ───────────────────────────────
    // PWM mode 1: output high when CNT < CCR, low otherwise
    // Preload: CCR latched, takes effect on next update event
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |  // CH1
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;   // CH2
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;  // CH3

    // ── Initial duty cycle: 50% on all phases ─────────────────────────────────
    // ARR=4499 → 50% = 2249
    // Safe neutral — equal duty on all phases = zero net voltage vector
    TIM1->CCR1 = 2249;
    TIM1->CCR2 = 2249;
    TIM1->CCR3 = 2249;

    // ── Enable all 6 outputs, active high polarity ────────────────────────────
    // CC1E/CC1NE: main and complementary output enable
    // Polarity bits cleared = active high (matches DRV8353RS input logic)
    TIM1->CCER = TIM_CCER_CC1E  | TIM_CCER_CC1NE |   // Phase A
                 TIM_CCER_CC2E  | TIM_CCER_CC2NE |   // Phase B
                 TIM_CCER_CC3E  | TIM_CCER_CC3NE;    // Phase C

    // ── BDTR — Break and Dead Time Register ──────────────────────────────────
    // MOE=0: all outputs disabled until pwm_enable() is called
    // No dead time — DRV8353RS handles this via TDRIVE VGS monitoring
    // BKE=0: nFAULT break input not yet wired (future improvement)
    TIM1->BDTR = 0;

    // ── Update interrupt — drives current/velocity loop ISR ──────────────────
    TIM1->DIER = TIM_DIER_UIE;                  // update interrupt enable
    TIM1->EGR  = TIM_EGR_UG;                    // force load ARR/PSC/CCR shadow regs
    TIM1->SR   = 0;                              // clear UIF set by EGR_UG

    
    // ── NVIC ──────────────────────────────────────────────────────────────────
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    // ── Start counter ─────────────────────────────────────────────────────────
    // Outputs remain off (MOE=0) until pwm_enable() is called
    TIM1->CR1 |= TIM_CR1_CEN;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_enable — set MOE, allow PWM signals to reach gate driver
// Call from drive state machine on STATE_ENABLED entry
// ─────────────────────────────────────────────────────────────────────────────
void pwm_enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_disable — clear MOE, all gate driver inputs go low (Hi-Z on DRV)
// Call on fault, STATE_IDLE entry, or any unsafe condition
// ─────────────────────────────────────────────────────────────────────────────
void pwm_disable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// ─────────────────────────────────────────────────────────────────────────────
// pwm_set_duty — set duty cycle for one phase
// phase: 0=A, 1=B, 2=C
// duty:  0 to 4499 (0% to 100%), center-aligned
// ─────────────────────────────────────────────────────────────────────────────
void pwm_set_duty(uint8_t phase, uint16_t duty)
{
    if (duty > 4499) duty = 4499;
    switch (phase) {
        case 0: TIM1->CCR1 = duty; break;   // Phase A
        case 1: TIM1->CCR2 = duty; break;   // Phase B
        case 2: TIM1->CCR3 = duty; break;   // Phase C
        default: break;
    }
}