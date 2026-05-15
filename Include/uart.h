// uart.h — USART3 telem transmit, STM32F446RE bare metal
// PB10 = USART3 TX (AF7)
// 921600 baud, 8N1, interrupt-driven TX
// uart_send_telem() called from SysTick at 1kHz

#pragma once
#include "stm32f4xx.h"
#include "protocol.h"

void uart_init(void);
void uart_send_telem(const volatile TelemetryFrame* frame);