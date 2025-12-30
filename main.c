#include <stdint.h>

#define RCC_BASE        0x40021000UL
#define RCC_AHBENR      (*(volatile uint32_t*)(RCC_BASE + 0x14UL))

#define GPIOB_BASE      0x48000400UL
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00UL))
#define GPIOB_BSRR      (*(volatile uint32_t*)(GPIOB_BASE + 0x18UL))

static void delay(volatile uint32_t n) { while (n--) __asm volatile("nop"); }

int main(void)
{
    // Enable GPIOB clock on STM32F0 (IOPBEN = bit 18 in RCC_AHBENR)
    RCC_AHBENR |= (1U << 18);

    // PB3 output (MODER3 = 01)
    GPIOB_MODER &= ~(3U << (3 * 2));
    GPIOB_MODER |=  (1U << (3 * 2));

    while (1)
    {
        GPIOB_BSRR = (1U << 3);        // PB3 high -> LD3 ON
        delay(800000);
        GPIOB_BSRR = (1U << (3+16));   // PB3 low  -> LD3 OFF
        delay(800000);
    }
}
