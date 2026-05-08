// spi2.hpp — SPI2 (Serial Peripheral Interface 2) slave interface
// STM32F446RE bare metal
//
// Include this wherever you need to:
//   - read telemetry buffers (TIM1 ISR)
//   - push to the ring buffer (CommSM)
//   - call spi2_init() (main)

#pragma once
#include <stdint.h>

// =============================================================================
// TelemetryFrame — 24 bytes, sent to Pi on every SPI2 transaction via DMA
// __attribute__((packed)) = no padding, fields laid out exactly as written.
// Required because Pi casts raw rx bytes directly to this struct.
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t  pos_cmd;       // (position command, encoder counts)
    int32_t  pos_fbk;       // (position feedback from TIM5 encoder, encoder counts)
    int16_t  vel_cmd;       // (velocity command, counts/sec)
    int16_t  vel_fbk;       // (dx/dt computed in velocity loop, counts/sec)
    uint32_t timestamp_ms;  // (STM millisecond counter — Pi uses this to correct
                            //  for Linux scheduler jitter on Bode plot frequency axis)
    uint8_t  drive_state;   // (DriveStateMachine state enum — see drive_sm.hpp)
    uint8_t  fault_flags;   // (fault bitmask — see fault flags below)
    uint8_t  _pad[6];       // (reserved, always 0x00, pads frame to 24 bytes)
} TelemetryFrame;

// =============================================================================
// Fault flags — bitmask in TelemetryFrame.fault_flags
// =============================================================================
#define FAULT_OCP      (1u << 0)  // (OverCurrent Protection — DRV8353RS tripped)
#define FAULT_OVP      (1u << 1)  // (OverVoltage Protection)
#define FAULT_UVP      (1u << 2)  // (UnderVoltage Protection)
#define FAULT_OVERTEMP (1u << 3)  // (motor thermistor over temperature)
#define FAULT_ENC      (1u << 4)  // (encoder loss or TIM5 error)
#define FAULT_ALIGN    (1u << 5)  // (startup rotor alignment failed)
#define FAULT_CRC      (1u << 6)  // (SPI2 receive CRC error on command path)
#define FAULT_DRV      (1u << 7)  // (DRV8353RS nFAULT pin asserted)

// =============================================================================
// TrajSample — one trajectory sample from Pi, 8 bytes
// Packed into DATA packets (opcode 0x04) and pushed to the ring buffer.
// =============================================================================
typedef struct __attribute__((packed)) {
    int32_t pos_cmd;  // (position setpoint, encoder counts, little-endian)
    int32_t vel_ff;   // (velocity feedforward, counts/sec, little-endian)
} TrajSample;

// =============================================================================
// SPI2 opcode definitions
// =============================================================================
#define SPI2_OP_NOP        0x00u  // (idle / pad)
#define SPI2_OP_BLOCK_HDR  0x03u  // (start of 2048-sample refill block)
#define SPI2_OP_DATA       0x04u  // (single trajectory sample)
#define SPI2_OP_READY_ACK  0x05u  // (Pi acknowledges PC13 READY signal)
#define SPI2_OP_TELEM_REQ  0x06u  // (Pi requests telemetry frame)

// =============================================================================
// Telem double-buffer — written by TIM1 ISR (1 kHz), read by SPI2 DMA TX
// extern = defined in spi2.c, accessible here
// volatile = DMA and ISR touch this memory, compiler must not cache or reorder
// =============================================================================
extern volatile TelemetryFrame telem_buf[2];
extern volatile uint8_t        telem_write_idx;

// =============================================================================
// SPI2 transaction size — both sides must agree
// =============================================================================
#define SPI2_TRANSACTION_BYTES  24u

// =============================================================================
// Public interface
// =============================================================================
void spi2_init(void);  // (call once at startup, after SystemClock_Config())