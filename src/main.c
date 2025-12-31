// ============================================================
// main.c  (STM32F0 bare-metal: LED blink + USART2 TX)
// ------------------------------------------------------------
// What this file demonstrates:
//  1) Enabling peripheral clocks in RCC
//  2) Configuring a GPIO pin as output and toggling it via BSRR
//  3) Configuring PA2/PA3 as USART2 alternate function pins
//  4) Initializing USART2 for 115200 8N1 and printing strings
//
// Assumptions (typical STM32F0 bring-up defaults):
//  - System clock running from HSI ~ 8 MHz (no PLL setup here)
//  - USART2 on PA2 (TX), PA3 (RX), AF1 (common on STM32F0)
//  - LED wired to PB3 (change in board.h if different)
// ============================================================

#include <stdint.h>
#include "stm32f0_regs.h"
#include "board.h"

// ----------------------------
// Simple busy-wait delay.
// This is NOT time-accurate; it just burns CPU cycles.
// Use for quick bring-up only.
// ----------------------------
static void delay(volatile uint32_t n)
{
    while (n--) { __asm volatile ("nop"); }
}

// ============================================================
// GPIO helpers
// These are generic helpers that work for any GPIO port
// because we pass the GPIO "base address" (GPIOA_BASE, GPIOB_BASE, ...)
// ============================================================

// Set pin mode (2 bits per pin in MODER)
static inline void gpio_set_mode(uint32_t gpio_base, uint32_t pin, uint32_t mode)
{
    const uint32_t shift = pin * 2u;
    GPIO_MODER(gpio_base) &= ~(3u << shift);           // clear the 2-bit field
    GPIO_MODER(gpio_base) |=  ((mode & 3u) << shift);  // set new mode
}

// Set output type (push-pull = 0, open-drain = 1)
static inline void gpio_set_pushpull(uint32_t gpio_base, uint32_t pin)
{
    GPIO_OTYPER(gpio_base) &= ~(1u << pin);            // 0 = push-pull
}

// Disable internal pulls (00 = no pull)
static inline void gpio_set_nopull(uint32_t gpio_base, uint32_t pin)
{
    const uint32_t shift = pin * 2u;
    GPIO_PUPDR(gpio_base) &= ~(3u << shift);           // 00 = no pull
}

// Atomic SET using BSRR (write 1 to bit [0..15])
static inline void gpio_set(uint32_t gpio_base, uint32_t pin)
{
    GPIO_BSRR(gpio_base) = (1u << pin);
}

// Atomic RESET using BSRR (write 1 to bit [16..31])
static inline void gpio_reset(uint32_t gpio_base, uint32_t pin)
{
    GPIO_BSRR(gpio_base) = (1u << (pin + 16u));
}

// Configure alternate function for a pin (AFRL for pins 0..7, AFRH for 8..15)
static inline void gpio_set_af(uint32_t gpio_base, uint32_t pin, uint32_t af)
{
    const uint32_t field = (pin & 7u) * 4u;  // 4 bits per pin inside AFRL/AFRH

    if (pin < 8u)
    {
        GPIO_AFRL(gpio_base) &= ~(0xFu << field);
        GPIO_AFRL(gpio_base) |=  ((af & 0xFu) << field);
    }
    else
    {
        GPIO_AFRH(gpio_base) &= ~(0xFu << field);
        GPIO_AFRH(gpio_base) |=  ((af & 0xFu) << field);
    }
}

// ============================================================
// USART2 (UART) helpers
// ============================================================

// For baud calculation, we need to know the peripheral clock driving USART2.
// With default STM32F0 bring-up, HSI is ~8 MHz and often used as system clock.
// If you later switch clocks (PLL, HSE, etc), update this value.
#define UART_PCLK_HZ   (8000000u)
#define UART_BAUD      (115200u)

// Initialize USART2 for 115200 8N1 (no parity, 1 stop bit)
// Steps:
//  1) Enable clocks for GPIOA and USART2
//  2) Put PA2/PA3 into Alternate Function mode
//  3) Select AF number for those pins (AF1 for USART2 on many STM32F0 parts)
//  4) Program BRR (baud)
//  5) Enable transmitter/receiver and then enable USART
static void usart2_init(void)
{
    // --- Turn on clocks to the peripherals we are about to touch ---
    RCC_AHBENR  |= RCC_AHBENR_IOPAEN;          // GPIOA clock (for PA2/PA3)
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;       // USART2 clock

    // --- Configure PA2/PA3 as alternate function pins ---
    gpio_set_mode(UART_GPIO_BASE, UART_TX_PIN, GPIO_MODE_AF);
    gpio_set_mode(UART_GPIO_BASE, UART_RX_PIN, GPIO_MODE_AF);

    // Optional: no internal pull resistors (external circuitry usually defines idle levels)
    gpio_set_nopull(UART_GPIO_BASE, UART_TX_PIN);
    gpio_set_nopull(UART_GPIO_BASE, UART_RX_PIN);

    // Select AF mapping (AF1 is a common mapping for USART2 on STM32F0)
    gpio_set_af(UART_GPIO_BASE, UART_TX_PIN, UART_AF);
    gpio_set_af(UART_GPIO_BASE, UART_RX_PIN, UART_AF);

    // --- Configure USART2 ---
    USART2_CR1 = 0; // disable/clear settings (UE=0)

    // Baud rate: BRR = PCLK / BAUD  (oversampling by 16 default)
    // Use rounding to be closer to requested baud.
    USART2_BRR = (UART_PCLK_HZ + (UART_BAUD / 2u)) / UART_BAUD;

    // Enable Transmitter and Receiver (8N1 is default when CR1/CR2/CR3 are 0)
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE;

    // Finally enable USART
    USART2_CR1 |= USART_CR1_UE;
}

// Send one character (blocking).
// TXE = "Transmit data register empty" → safe to write TDR.
static void usart2_putc(char c)
{
    while ((USART2_ISR & USART_ISR_TXE) == 0) { }
    USART2_TDR = (uint8_t)c;
}

// Send a null-terminated string (blocking).
static void usart2_puts(const char *s)
{
    while (*s)
    {
        usart2_putc(*s++);
    }
}

// Convenience: send string + CRLF (nice for terminals)
static void usart2_puts_ln(const char *s)
{
    usart2_puts(s);
    usart2_puts("\r\n");
}

// ============================================================
// main()
// ============================================================
int main(void)
{
    // ------------------------------------------------------------
    // Optional "safety" bring-up:
    // - Disable clock security system (CSS) so a missing HSE doesn't NMI you
    // - Clear any clock interrupt flags
    // ------------------------------------------------------------
    RCC_CR  &= ~RCC_CR_CSSON;
    RCC_CIR  = 0xFFFFFFFFu;

    // ------------------------------------------------------------
    // Enable GPIO clock for the LED port (GPIOB)
    // IMPORTANT: Peripherals are "off" until their RCC clock enable bit is set.
    // ------------------------------------------------------------
    RCC_AHBENR |= RCC_AHBENR_IOPBEN;

    // ------------------------------------------------------------
    // Configure LED pin as output:
    //  - Mode = output
    //  - Output type = push-pull
    //  - No pull resistors
    // ------------------------------------------------------------
    gpio_set_mode(LED_GPIO_BASE, LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pushpull(LED_GPIO_BASE, LED_PIN);
    gpio_set_nopull(LED_GPIO_BASE, LED_PIN);

    // ------------------------------------------------------------
    // Init UART and print a boot message.
    // If this prints garbage, your UART_PCLK_HZ or AF/pins are wrong.
    // ------------------------------------------------------------
    usart2_init();
    usart2_puts_ln("boot");

    // ------------------------------------------------------------
    // Main loop: blink LED and print status.
    // ------------------------------------------------------------
    while (1)
    {
        gpio_set(LED_GPIO_BASE, LED_PIN);
        usart2_puts_ln("LED ON");
        delay(250000);

        gpio_reset(LED_GPIO_BASE, LED_PIN);
        usart2_puts_ln("LED OFF");
        delay(250000);
    }
}
