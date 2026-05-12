// drive.h — servo drive state machine
// States: IDLE → ALIGN → ENABLED → IDLE
//         any state → FAULT on fault condition
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

typedef enum {
    STATE_IDLE    = 0,
    STATE_ALIGN   = 1,
    STATE_ENABLED = 2,
    STATE_FAULT   = 3
} DriveState;

// Initialize state machine — call once at startup
void drive_init(void);

// Update state machine — call from SysTick at 1kHz
void drive_update(void);

// Transition requests — called from ISR or SPI handler
void drive_request_enable(void);   // BLOCK_HDR received → request enable
void drive_request_fault(void);    // fault detected → latch fault

// State query — called from TIM1 ISR to gate loop execution
DriveState drive_get_state(void);

// True only on first tick of a new state — use for one-time init
uint8_t drive_is_entry(void);

#endif