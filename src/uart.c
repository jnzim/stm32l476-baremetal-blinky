// uart.c — USART3 telem transmit, STM32F446RE bare metal
//
// PB10 = USART3 TX (AF7), APB1 = 45MHz
// 921600 baud, 8N1, interrupt-driven
//
// SysTick calls uart_send_telem() every 1ms.
// Frame (24 bytes) is copied into tx_buf, then sent byte by byte
// via USART3 TXE interrupt. At 921600 baud, 24 bytes = ~260us.
// If previous frame is still sending, new frame is dropped.

#include "uart.h"
#include "stm32f4xx.h"
#include "protocol.h"
#include <string.h>

// ── TX buffer ─────────────────────────────────────────────────────────────────
static uint8_t  tx_buf[sizeof(TelemetryFrame)];
static volatile uint8_t  tx_idx  = 0;
static volatile uint8_t  tx_len  = 0;
static volatile uint8_t  tx_busy = 0;

// ─────────────────────────────────────────────────────────────────────────────
// uart_init
// ─────────────────────────────────────────────────────────────────────────────
void uart_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;

    // ── PB10: AF7 = USART3_TX ────────────────────────────────────────────────
    GPIOB->MODER  &= ~(3u << 20);
    GPIOB->MODER  |=  (2u << 20);      // alternate function
    GPIOB->AFR[1] &= ~(0xFu << 8);
    GPIOB->AFR[1] |=  (7u   << 8);     // AF7 = USART3
    GPIOB->OSPEEDR |= (3u << 20);      // very high speed

    // ── USART3: 921600 baud @ 45MHz APB1, 8N1, TX only ───────────────────────
    // BRR: APB1=45MHz, baud=921600
    // DIV = 45000000 / 921600 = 48.828
    // Mantissa = 48 = 0x30, Fraction = 0.828 * 16 = 13 = 0xD
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = (48u << 4) | 13u;
    USART3->CR1 = USART_CR1_TE         // transmitter enable
                | USART_CR1_UE;        // USART enable
    // TXE interrupt enabled only when transmitting

    // end of uart_init() — test transmission
    while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = 0xAA;
    while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = 0x55;

    // ── NVIC ──────────────────────────────────────────────────────────────────
    NVIC_SetPriority(USART3_IRQn, 3);  // lowest priority — below all control loops
    NVIC_EnableIRQ(USART3_IRQn);
}

// ─────────────────────────────────────────────────────────────────────────────
// uart_send_telem
// Called from SysTick at 1kHz. Drops frame if previous still sending.
// ─────────────────────────────────────────────────────────────────────────────
void uart_send_telem(const volatile TelemetryFrame* frame)
{
    if (tx_busy) return;    // drop frame — previous still in flight

    memcpy(tx_buf, (void*)frame, sizeof(TelemetryFrame));
    tx_idx  = 0;
    tx_len  = sizeof(TelemetryFrame);
    tx_busy = 1;

    // Enable TXE interrupt to start transmission
    USART3->CR1 |= USART_CR1_TXEIE;
}

// ─────────────────────────────────────────────────────────────────────────────
// USART3_IRQHandler — sends one byte per TXE interrupt
// ─────────────────────────────────────────────────────────────────────────────
void USART3_IRQHandler(void)
{
    if (!(USART3->SR & USART_SR_TXE)) return;

    if (tx_idx < tx_len) {
        USART3->DR = tx_buf[tx_idx++];
    } else {
        // All bytes sent — disable TXE interrupt
        USART3->CR1 &= ~USART_CR1_TXEIE;
        tx_busy = 0;
    }
}