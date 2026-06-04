// --- tim1.c ---
#include "stm32f4xx.h"
#include "tim1.h"



void tim1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    TIM1->PSC = 0;
    TIM1->ARR = 2500;
    TIM1->CR1 = TIM_CR1_CMS_0;
    TIM1->RCR = 1;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0;
    TIM1->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}

