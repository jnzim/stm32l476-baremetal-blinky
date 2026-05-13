// spi.h — SPI2 (Serial Peripheral Interface 2) slave interface
// STM32F446RE bare metal
//
// TelemetryFrame, opcodes, fault flags, and drive states live in protocol.h
// which is shared between this firmware and the Pi-side C++ application.

#pragma once
#include <stdint.h>
#include "protocol.h"

// =============================================================================
// Telem double-buffer — written by TIM1 ISR, read by SPI2 DMA TX
// Two slots — ISR writes to inactive slot, DMA reads from active slot.
// extern = defined in spi.c
// volatile = ISR and DMA touch this memory, compiler must not cache or reorder
// =============================================================================
extern volatile TelemetryFrame telem_buf[2];
extern volatile uint8_t        telem_write_idx;

extern volatile uint8_t sim_active;

extern volatile uint32_t samples_consumed;
// =============================================================================
// Public interface
// =============================================================================
void spi_init(void);  // call once at startup after SystemClock_Config


void spi_process(void);