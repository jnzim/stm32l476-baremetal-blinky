#include <stdint.h>

#define PERIPH_BASE     (0x40000000UL)
#define AHB2PERIPH_BASE (PERIPH_BASE + 0x08000000UL)  // STM32L4 AHB2
#define GPIOA_BASE      (AHB2PERIPH_BASE + 0x0000UL)  // 0x48000000

#define RCC_BASE        (PERIPH_BASE + 0x10000UL)     // 0x40021000
#define RCC_AHB2ENR     (*(volatile uint32_t*)(RCC_BASE + 0x4CUL))

#define GPIOA_MODER     (*(volatile uint32_t*)(GPIOA_BASE + 0x00UL))
#define GPIOA_OTYPER    (*(volatile uint32_t*)(GPIOA_BASE + 0x04UL))
#define GPIOA_BSRR      (*(volatile uint32_t*)(GPIOA_BASE + 0x18UL))

static void delay(volatile uint32_t n)
{
    while (n--) {
        __asm volatile ("nop");
    }
}

int main(void)
{
    /* Enable GPIOA clock (AHB2ENR bit0 = GPIOAEN) */
    RCC_AHB2ENR |= (1u << 0);

    /* PA5 output mode: MODER5 = 01 */
    GPIOA_MODER &= ~(3u << (5u * 2u));
    GPIOA_MODER |=  (1u << (5u * 2u));

    /* Push-pull: OT5 = 0 */
    GPIOA_OTYPER &= ~(1u << 5u);

    while (1) {
        /* set PA5 */
        GPIOA_BSRR = (1u << 5u);
        delay(200000);

        /* reset PA5 */
        GPIOA_BSRR = (1u << (5u + 16u));
        delay(200000);
    }
}