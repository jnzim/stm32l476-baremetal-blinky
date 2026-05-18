// drive.h — servo drive state machine
// States: IDLE → ALIGN → ENABLED → IDLE
//         any state → FAULT on fault condition
#pragma once
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

// True only on first tick of a new state
uint8_t drive_is_entry(void);

// Alignment — apply d-axis voltage at theta=0, v_q=0
// Call repeatedly from STATE_ALIGN for ALIGN_TIME_MS
void drive_align_rotor(void);

// Open-loop spin — advance electrical angle by d_theta each call
// v_mag: voltage magnitude (volts)
// d_theta: angle increment per call (radians)
void drive_open_loop_step(float v_mag, float d_theta);