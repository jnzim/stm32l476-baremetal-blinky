// drive.c — servo drive state machine
//
// Architecture:
//   drive_update() runs in SysTick at 1kHz — only place state transitions happen
//   TIM1 ISR calls drive_get_state() to gate loop execution — reads only, never writes
//   DMA ISR calls drive_request_enable() / drive_request_fault() — sets flags only
//
// This keeps the state machine deterministic:
//   - One writer (SysTick), multiple readers (TIM1, DMA)
//   - Flag writes are single-byte — atomic on Cortex-M4, no critical section needed
//   - State transitions never happen inside a high-priority ISR

#include "drive.h"
#include "spi.h"
#include "loops.h"
#include "plant.h"

// ── State ─────────────────────────────────────────────────────────────────────
static DriveState state      = STATE_IDLE;
static DriveState state_prev = STATE_IDLE;

// ── Request flags — set by ISRs, cleared by drive_update() ───────────────────
static volatile uint8_t enable_req = 0;
static volatile uint8_t fault_req  = 0;

extern PlantState plant;
static uint8_t entry_flag = 0;

void drive_init(void)
{
    state      = STATE_IDLE;
    state_prev = STATE_IDLE;
    enable_req = 0;
    fault_req  = 0;
}

void drive_request_enable(void)
{
    enable_req = 1;
}

void drive_request_fault(void)
{
    fault_req = 1;
}

DriveState drive_get_state(void)
{
    return state;
}

uint8_t drive_is_entry(void)
{
    uint8_t e = entry_flag;
    entry_flag = 0;
    return e;
}

void drive_update(void)
{
    if (fault_req)
    {
        state     = STATE_FAULT;
        fault_req = 0;
        return;
    }

    state_prev = state;

    switch (state) {

    case STATE_IDLE:
        if (enable_req) {
            enable_req = 0;
            loops_reset();
            plant_init(&plant);
            state      = STATE_ENABLED;
            entry_flag = 1;
        }
        break;

    case STATE_ALIGN:
        state = STATE_ENABLED;
        break;

    case STATE_ENABLED:
        break;

    case STATE_FAULT:
        break;
    }
}