#include <stdint.h>
#include "stm32f0_regs.h"

#define GPIO_MODE_INPUT   (0u)
#define GPIO_MODE_OUTPUT  (1u)
/* 2u = alternate, 3u = analog */

static void delay(volatile uint32_t n)
{
    while (n--) { __asm volatile ("nop"); }
}

/* ===== GPIOB helpers (no pointer passing, no macro-address tricks) ===== */
static inline void gpiob_set_mode(uint32_t pin, uint32_t mode)
{
    const uint32_t shift = pin * 2u;          // MODER has 2 bits per pin
    GPIOB_MODER &= ~(3u << shift);            // clear field
    GPIOB_MODER |=  ((mode & 3u) << shift);   // write field
}

static inline void gpiob_set_pushpull(uint32_t pin)
{
    GPIOB_OTYPER &= ~(1u << pin);             // 0 = push-pull
}

static inline void gpiob_set_speed_low(uint32_t pin)
{
    const uint32_t shift = pin * 2u;
    GPIOB_OSPEEDR &= ~(3u << shift);
    GPIOB_OSPEEDR |=  (1u << shift);          // 01 = low speed
}

static inline void gpiob_set_nopull(uint32_t pin)
{
    const uint32_t shift = pin * 2u;
    GPIOB_PUPDR &= ~(3u << shift);            // 00 = no pull
}

static inline void gpiob_set(uint32_t pin)
{
    GPIOB_BSRR = (1u << pin);                 // set pin
}

static inline void gpiob_reset(uint32_t pin)
{
    GPIOB_BSRR = (1u << (pin + 16u));         // reset pin
}

int main(void)
{
    /* Optional safety during bring-up */
    RCC_CR  &= ~RCC_CR_CSSON;
    RCC_CIR  = 0xFFFFFFFFu;

    /* Enable GPIOB clock */
    RCC_AHBENR |= RCC_AHBENR_IOPBEN;

    /* PBx config */
    gpiob_set_mode(LED_PIN, GPIO_MODE_OUTPUT);
    gpiob_set_pushpull(LED_PIN);
    gpiob_set_speed_low(LED_PIN);
    gpiob_set_nopull(LED_PIN);

    while (1) 
    {
        gpiob_set(LED_PIN);
        delay(250000);
        gpiob_reset(LED_PIN);
        delay(250000);
    }
}
