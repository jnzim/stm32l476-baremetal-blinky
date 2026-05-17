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
#include "ringBuffer.h"

// ── State ─────────────────────────────────────────────────────────────────────
static DriveState state      = STATE_IDLE;
static DriveState state_prev = STATE_IDLE;

// ── Request flags — set by ISRs, cleared by drive_update() ───────────────────
// volatile: written by DMA ISR, read by SysTick — compiler must not cache
static volatile uint8_t enable_req = 0;
static volatile uint8_t fault_req  = 0;

// ── entry_flag — set on state transition, cleared by drive_is_entry() ────────
extern PlantState plant;
static uint8_t entry_flag = 0;

// ─────────────────────────────────────────────────────────────────────────────
// drive_init — call once at startup before any ISRs are enabled
// ─────────────────────────────────────────────────────────────────────────────
void drive_init(void)
{
    state      = STATE_IDLE;
    state_prev = STATE_IDLE;
    enable_req = 0;
    fault_req  = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_request_enable — called from DMA ISR on BLOCK_HDR receipt
// Sets flag only — transition happens in drive_update() at next SysTick tick
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_enable(void)
{
    enable_req = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_request_fault — called from any ISR on fault condition
// Fault is latched — cleared only by explicit fault reset (not implemented yet)
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_fault(void)
{
    fault_req = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_get_state — called from TIM1 ISR to gate loop execution
// Read only — never transitions state
// ─────────────────────────────────────────────────────────────────────────────
DriveState drive_get_state(void)
{
    return state;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_is_entry — returns 1 on first tick of a new state, clears automatically
// ─────────────────────────────────────────────────────────────────────────────
uint8_t drive_is_entry(void)
{
    uint8_t e = entry_flag;
    entry_flag = 0;
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_update — call from SysTick at 1kHz
//
// Processes request flags and executes state transitions.
// Fault takes priority over all other transitions.
// ─────────────────────────────────────────────────────────────────────────────
void drive_update(void)
{
    // fault takes priority from any state — latches immediately
    if (fault_req)
    {
        state     = STATE_FAULT;
        fault_req = 0;
        return;
    }

    state_prev = state;

    switch (state) {

    case STATE_IDLE:
        // waiting for BLOCK_HDR from Pi — DMA ISR sets enable_req
        if (enable_req) 
        {
            enable_req = 0;
            loops_reset();
            plant_init(&plant);
            state      = STATE_ENABLED;
            entry_flag = 1;
        }
        break;

    case STATE_ALIGN:
        // real HW: fire rotor alignment pulse, wait for completion
        // AKM11E has no Hall sensors (N2 option) — alignment required at startup
        // sim mode: this state is never entered
        state = STATE_ENABLED;
        break;

    case STATE_ENABLED:
        if (ring.count == 0 && first_sample_ready && samples_consumed > 0)
        {
            state      = STATE_IDLE;
            entry_flag = 1;
            first_sample_ready = 0;
        }
        break;

    case STATE_FAULT:
        // PWM disabled — waiting for explicit fault clear
        // TODO: add fault reset request and transition back to IDLE
        break;
    }
}