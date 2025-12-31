#ifndef STM32F0_REGS_H
#define STM32F0_REGS_H

#include <stdint.h>

/* ============================================================
 * STM32F0 – Minimal Register Definitions
 * CHIP-LEVEL ONLY (no board wiring)
 * ============================================================
 */


/* ================= RCC (Reset and Clock Control) ================= */

/* RCC_BASE
 * Base address of the Reset and Clock Control peripheral.
 * All RCC registers are offsets from this address.
 */
#define RCC_BASE        (0x40021000UL)

/* RCC_CR — Clock Control Register
 * Controls system clock sources (HSI, HSE, PLL)
 * and reports oscillator ready status.
 */
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00UL))

/* RCC_CIR — Clock Interrupt Register
 * Holds clock ready/clock failure interrupt flags.
 * Writing 1s clears pending clock-related interrupts.
 */
#define RCC_CIR         (*(volatile uint32_t *)(RCC_BASE + 0x08UL))

/* RCC_AHBENR — AHB Peripheral Clock Enable Register
 * Enables clocks for AHB-connected peripherals
 * such as GPIO ports and DMA.
 */
#define RCC_AHBENR      (*(volatile uint32_t *)(RCC_BASE + 0x14UL))

/* RCC_APB1ENR — APB1 Peripheral Clock Enable Register
 * Enables clocks for APB1 peripherals
 * such as USART2, I2C, and some timers.
 */
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x1CUL))


/* RCC_CR_CSSON
 * Enables the Clock Security System (CSS).
 * Generates an NMI if the external clock fails.
 */
#define RCC_CR_CSSON            (1u << 19)

/* RCC_AHBENR_IOPAEN
 * Enables clocking to GPIO Port A.
 * Must be set before accessing GPIOA registers.
 */
#define RCC_AHBENR_IOPAEN       (1u << 17)

/* RCC_AHBENR_IOPBEN
 * Enables clocking to GPIO Port B.
 * Must be set before accessing GPIOB registers.
 */
#define RCC_AHBENR_IOPBEN       (1u << 18)

/* RCC_APB1ENR_USART2EN
 * Enables clocking to USART2 peripheral.
 * USART2 will not function unless this bit is set.
 */
#define RCC_APB1ENR_USART2EN    (1u << 17)


/* ================= GPIO (General Purpose I/O) ================= */

/* GPIOA_BASE
 * Base address of GPIO Port A registers.
 */
#define GPIOA_BASE      (0x48000000UL)

/* GPIOB_BASE
 * Base address of GPIO Port B registers.
 */
#define GPIOB_BASE      (0x48000400UL)

/* GPIO_MODER
 * GPIO port mode register.
 * Selects input/output/alternate/analog mode.
 * Two bits per pin.
 */
#define GPIO_MODER(base)    (*(volatile uint32_t *)((base) + 0x00UL))

/* GPIO_OTYPER
 * GPIO output type register.
 * Selects push-pull or open-drain for output pins.
 * One bit per pin.
 */
#define GPIO_OTYPER(base)   (*(volatile uint32_t *)((base) + 0x04UL))

/* GPIO_OSPEEDR
 * GPIO output speed register.
 * Controls slew rate of output pins.
 * Two bits per pin.
 */
#define GPIO_OSPEEDR(base)  (*(volatile uint32_t *)((base) + 0x08UL))

/* GPIO_PUPDR
 * GPIO pull-up/pull-down register.
 * Enables/disables internal pull resistors.
 * Two bits per pin.
 */
#define GPIO_PUPDR(base)    (*(volatile uint32_t *)((base) + 0x0CUL))

/* GPIO_IDR
 * GPIO input data register.
 * Reads the logic level present on input pins.
 */
#define GPIO_IDR(base)      (*(volatile uint32_t *)((base) + 0x10UL))

/* GPIO_ODR
 * GPIO output data register.
 * Writes output pin values (non-atomic).
 */
#define GPIO_ODR(base)      (*(volatile uint32_t *)((base) + 0x14UL))

/* GPIO_BSRR
 * GPIO bit set/reset register.
 * Atomic control of output pins.
 * Bits 0–15 set pins, bits 16–31 reset pins.
 */
#define GPIO_BSRR(base)     (*(volatile uint32_t *)((base) + 0x18UL))

/* GPIO_AFRL
 * GPIO alternate function low register.
 * Selects alternate function for pins 0–7.
 * Four bits per pin.
 */
#define GPIO_AFRL(base)     (*(volatile uint32_t *)((base) + 0x20UL))

/* GPIO_AFRH
 * GPIO alternate function high register.
 * Selects alternate function for pins 8–15.
 * Four bits per pin.
 */
#define GPIO_AFRH(base)     (*(volatile uint32_t *)((base) + 0x24UL))


/* GPIO mode values */
#define GPIO_MODE_INPUT    (0u)   /* Input mode */
#define GPIO_MODE_OUTPUT   (1u)   /* General-purpose output */
#define GPIO_MODE_AF       (2u)   /* Alternate function */
#define GPIO_MODE_ANALOG   (3u)   /* Analog mode */


/* ================= USART2 ================= */

/* USART2_BASE
 * Base address of USART2 peripheral registers.
 */
#define USART2_BASE     (0x40004400UL)

/* USART2_CR1
 * USART control register 1.
 * Enables USART, transmitter, and receiver.
 */
#define USART2_CR1      (*(volatile uint32_t *)(USART2_BASE + 0x00UL))

/* USART2_BRR
 * USART baud rate register.
 * Determines serial communication speed.
 */
#define USART2_BRR      (*(volatile uint32_t *)(USART2_BASE + 0x0CUL))

/* USART2_ISR
 * USART interrupt and status register.
 * Reports TX/RX status and error flags.
 */
#define USART2_ISR      (*(volatile uint32_t *)(USART2_BASE + 0x1CUL))

/* USART2_RDR
 * USART receive data register.
 * Reading retrieves the next received byte.
 */
#define USART2_RDR      (*(volatile uint32_t *)(USART2_BASE + 0x24UL))

/* USART2_TDR
 * USART transmit data register.
 * Writing sends a byte.
 */
#define USART2_TDR      (*(volatile uint32_t *)(USART2_BASE + 0x28UL))


/* USART control/status bits */

/* USART_CR1_UE
 * Enables the USART peripheral.
 */
#define USART_CR1_UE     (1u << 0)

/* USART_CR1_RE
 * Enables the USART receiver.
 */
#define USART_CR1_RE     (1u << 2)

/* USART_CR1_TE
 * Enables the USART transmitter.
 */
#define USART_CR1_TE     (1u << 3)

/* USART_ISR_RXNE
 * Receive data register not empty.
 */
#define USART_ISR_RXNE   (1u << 5)

/* USART_ISR_TXE
 * Transmit data register empty.
 */
#define USART_ISR_TXE    (1u << 7)

#endif /* STM32F0_REGS_H */
