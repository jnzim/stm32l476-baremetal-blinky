#include <stdint.h>

/* -------------------- Base addresses (STM32F0) -------------------- */
#define RCC_BASE        0x40021000UL
#define GPIOB_BASE      0x48000400UL

/* -------------------- RCC registers -------------------- */
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CIR         (*(volatile uint32_t *)(RCC_BASE + 0x08UL))
#define RCC_AHBENR      (*(volatile uint32_t *)(RCC_BASE + 0x14UL))

/* RCC bits */
#define RCC_CR_CSSON    (1u << 19)      /* Clock Security System enable */
#define RCC_AHBENR_IOPBEN (1u << 18)    /* GPIOB clock enable */

/* -------------------- GPIOB registers -------------------- */
#define GPIOB_MODER     (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OTYPER    (*(volatile uint32_t *)(GPIOB_BASE + 0x04UL))
#define GPIOB_OSPEEDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))
#define GPIOB_PUPDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x0CUL))
#define GPIOB_BSRR      (*(volatile uint32_t *)(GPIOB_BASE + 0x18UL))

/* NUCLEO-F042K6 user LED (LD3) is PB3 */
#define LED_PIN         3u

static void delay(volatile uint32_t n)
{
    while (n--) {
        __asm volatile ("nop");
    }
}

int main(void)
{
    /* ----------------------------------------------------------------
       Prevent “mystery NMI” from clock security / clock interrupt flags
       ---------------------------------------------------------------- */
    RCC_CR  &= ~RCC_CR_CSSON;     /* Disable Clock Security System */
    RCC_CIR  = 0xFFFFFFFFu;       /* Clear all RCC clock interrupt flags */

    /* Enable GPIOB peripheral clock */
    RCC_AHBENR |= RCC_AHBENR_IOPBEN;

    /* Configure PB3 as general purpose output (MODER3 = 01) */
    GPIOB_MODER &= ~(3u << (LED_PIN * 2));
    GPIOB_MODER |=  (1u << (LED_PIN * 2));

    /* Push-pull output */
    GPIOB_OTYPER &= ~(1u << LED_PIN);

    /* Low speed is fine */
    GPIOB_OSPEEDR &= ~(3u << (LED_PIN * 2));
    GPIOB_OSPEEDR |=  (1u << (LED_PIN * 2));

    /* No pull-up / pull-down */
    GPIOB_PUPDR &= ~(3u << (LED_PIN * 2));

    while (1) {
        /* Set PB3 */
        GPIOB_BSRR = (1u << LED_PIN);
        delay(250000);

        /* Reset PB3 */
        GPIOB_BSRR = (1u << (LED_PIN + 16u));
        delay(250000);
    }
}
