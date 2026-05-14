// clock.c — HSI → PLL → 180MHz for STM32F446RE
// APB1 = 45MHz (÷4), APB2 = 90MHz (÷2)

#include "clock.h"
#include "stm32f4xx.h"

void clock_init(void)
{
    // ── Enable HSI ────────────────────────────────────────────────────────────
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // ── Configure PLL: HSI(16MHz) / M=8 * N=180 / P=2 = 180MHz ──────────────
    RCC->PLLCFGR = (8u   << RCC_PLLCFGR_PLLM_Pos) |   // M=8  → VCO input = 2MHz
                   (180u << RCC_PLLCFGR_PLLN_Pos)  |   // N=180 → VCO = 360MHz
                   (0u   << RCC_PLLCFGR_PLLP_Pos)  |   // P=2  → SYSCLK = 180MHz
                   RCC_PLLCFGR_PLLSRC_HSI;

    // ── Enable PLL ────────────────────────────────────────────────────────────
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // ── Flash latency for 180MHz (5 wait states) ──────────────────────────────
    FLASH->ACR = FLASH_ACR_LATENCY_5WS |
                 FLASH_ACR_PRFTEN      |
                 FLASH_ACR_ICEN        |
                 FLASH_ACR_DCEN;

    // ── APB prescalers: APB1=÷4 (45MHz), APB2=÷2 (90MHz) ────────────────────
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    // ── Switch SYSCLK to PLL ──────────────────────────────────────────────────
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // ── Update CMSIS core clock variable ─────────────────────────────────────
    SystemCoreClock = 180000000;
}