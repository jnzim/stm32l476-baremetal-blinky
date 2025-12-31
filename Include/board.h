#ifndef BOARD_H
#define BOARD_H

#include "stm32f0_regs.h"

/* ============================================================
 * Board-specific wiring
 * ============================================================
 */

/* User LED */
#define LED_GPIO_BASE   GPIOB_BASE
#define LED_PIN         (3u)

/* USART2 pin mapping (typical STM32F0 / Nucleo) */
#define UART_GPIO_BASE  GPIOA_BASE
#define UART_TX_PIN     (2u)   /* PA2 */
#define UART_RX_PIN     (3u)   /* PA3 */
#define UART_AF         (1u)   /* AF1 = USART2 */

#endif /* BOARD_H */
