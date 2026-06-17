// --- tim1.c ---
#include "stm32f4xx.h"
#include "tim1.h"

void tim1_init(void)
{
    /*
     * Enable TIM1 peripheral clock.
     *
     * TIM1 is on APB2.
     * No TIM1 registers are safe to touch until this clock is enabled.
     */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    /*
     * TIM1 clock setup.
     *
     * PSC = prescaler.
     * PSC = 0 means timer counts at full TIM1 timer clock.
     *
     * ARR = auto-reload register.
     * In center-aligned PWM, timer counts:
     *
     *   0 -> ARR -> 0 -> ARR -> ...
     *
     * So the full PWM period is based on the up+down count.
     */
    TIM1->PSC = 0;
    TIM1->ARR = 2500;

    /*
     * CR1 = control register 1.
     *
     * CMS = center-aligned mode selection.
     *
     * TIM_CR1_CMS_0 means center-aligned mode 1.
     *
     * Center-aligned PWM is preferred for FOC because switching edges are
     * symmetric around the PWM center.
     */
    TIM1->CR1 = TIM_CR1_CMS_0;

    /*
     * RCR = repetition counter.
     *
     * Advanced timers like TIM1 can generate update events every N+1 timer
     * update opportunities.
     *
     * RCR = 1 means update event occurs every 2 update opportunities.
     *
     * In center-aligned mode, this is often used so the effective update
     * interrupt rate matches the desired PWM/control rate.
     */
    TIM1->RCR = 1;

    /*
     * CR2 = control register 2.
     *
     * MMS = master mode selection.
     *
     * This controls what TIM1 outputs on its internal TRGO signal.
     *
     * TRGO is not a physical pin here. It is an internal trigger signal that
     * other peripherals, like ADC1, can listen to.
     *
     * MMS = 010 means:
     *
     *   TIM1 update event -> TIM1_TRGO pulse
     *
     * This is what allows ADC1 injected conversions to start at a known point
     * in the PWM cycle.
     */
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |=  (2u << TIM_CR2_MMS_Pos);   // MMS = 010: update event as TRGO

    /*
     * EGR = event generation register.
     *
     * UG = update generation.
     *
     * Writing UG forces the timer to reload prescaler/repetition/ARR shadow
     * state immediately. This gives a clean starting state.
     */
    TIM1->EGR = TIM_EGR_UG;

    /*
     * SR = status register.
     *
     * Clear pending flags before enabling interrupt.
     */
    TIM1->SR = 0;

    /*
     * DIER = DMA/interrupt enable register.
     *
     * UIE = update interrupt enable.
     *
     * This enables TIM1_UP_TIM10_IRQHandler() on update events.
     *
     * Separate concept from TRGO:
     *
     *   DIER.UIE gives you the CPU interrupt.
     *   CR2.MMS gives the ADC an internal trigger.
     */
    TIM1->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /*
     * CEN = counter enable.
     *
     * Start TIM1.
     */
    TIM1->CR1 |= TIM_CR1_CEN;
}