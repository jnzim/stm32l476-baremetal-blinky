#pragma once
#include <stdint.h>

/* ===== STM32F0 minimal RCC + GPIOB register map ===== */

#define RCC_BASE        (0x40021000UL)
#define GPIOB_BASE      (0x48000400UL)

/* RCC registers */
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CIR         (*(volatile uint32_t *)(RCC_BASE + 0x08UL))
#define RCC_AHBENR      (*(volatile uint32_t *)(RCC_BASE + 0x14UL))

/* RCC bits */
#define RCC_CR_CSSON       (1u << 19)
#define RCC_AHBENR_IOPBEN  (1u << 18)

/* GPIOB registers */
#define GPIOB_MODER     (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OTYPER    (*(volatile uint32_t *)(GPIOB_BASE + 0x04UL))
#define GPIOB_OSPEEDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))
#define GPIOB_PUPDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x0CUL))
#define GPIOB_BSRR      (*(volatile uint32_t *)(GPIOB_BASE + 0x18UL))

/* Change this if your LED isn't on PB3 */
#define LED_PIN         (3u)
