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
#include "pwm.h"
#include "config.h"
#include <math.h>

// ── State ─────────────────────────────────────────────────────────────────────
static DriveState state      = STATE_IDLE;
static DriveState state_prev = STATE_IDLE;

// ── Request flags — set by ISRs, cleared by drive_update() ───────────────────
static volatile uint8_t enable_req = 0;
static volatile uint8_t fault_req  = 0;

// ── entry_flag — set on state transition, cleared by drive_is_entry() ────────
extern PlantState plant;
static uint8_t entry_flag = 0;

// ── Open-loop / alignment state ───────────────────────────────────────────────
static float ol_theta = 0.0f;          // open-loop electrical angle accumulator

// ─────────────────────────────────────────────────────────────────────────────
// drive_init
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
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_enable(void)
{
    enable_req = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_request_fault — called from any ISR on fault condition
// ─────────────────────────────────────────────────────────────────────────────
void drive_request_fault(void)
{
    fault_req = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_get_state
// ─────────────────────────────────────────────────────────────────────────────
DriveState drive_get_state(void)
{
    return state;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_is_entry
// ─────────────────────────────────────────────────────────────────────────────
uint8_t drive_is_entry(void)
{
    uint8_t e = entry_flag;
    entry_flag = 0;
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_align_rotor — apply fixed d-axis voltage at theta=0
//
// Forces rotor to a known electrical angle before closing the FOC loop.
// Required for AKM11E (N2 option — no Hall sensors).
// Call repeatedly from STATE_ALIGN for ALIGN_TIME_MS before transitioning.
//
// v_d = ALIGN_VOLTAGE, v_q = 0 → torque = 0, rotor locks to theta=0
// ─────────────────────────────────────────────────────────────────────────────
void drive_align_rotor(void)
{
    pwm_apply_vq(0.0f, ALIGN_VOLTAGE, 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_open_loop_step — advance electrical angle and apply voltage
//
// Spins motor open loop without encoder feedback.
// Used to verify phase wiring and encoder direction before closing loops.
//
// v_mag:   voltage magnitude (volts) — keep low, e.g. 1.0–2.0V at 12V bus
// d_theta: angle increment per call (radians) — sets speed
//          at 1kHz SysTick: d_theta = 2π * f_elec / 1000
//          e.g. 1Hz electrical: d_theta = 0.00628f
// ─────────────────────────────────────────────────────────────────────────────
void drive_open_loop_step(float v_mag, float d_theta)
{
    ol_theta += d_theta;
    if (ol_theta >= 2.0f * M_PI) ol_theta -= 2.0f * M_PI;
    pwm_apply_vq(v_mag, 0.0f, ol_theta);
}

// ─────────────────────────────────────────────────────────────────────────────
// drive_update — call from SysTick at 1kHz
// ─────────────────────────────────────────────────────────────────────────────
void drive_update(void)
{
    // fault takes priority from any state — latches immediately
    if (fault_req)
    {
        pwm_disable();
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
            ol_theta = 0.0f;        // reset open-loop angle for clean start
            pwm_enable();
            state      = STATE_ENABLED;
            entry_flag = 1;
        }
        break;

    case STATE_ALIGN:
        // Hold rotor at theta=0 for ALIGN_TIME_MS before enabling FOC
        // AKM11E has no Hall sensors (N2 option) — alignment required at startup
        // TODO: add tick counter, transition to STATE_ENABLED after ALIGN_TIME_MS
        drive_align_rotor();
        break;

    case STATE_ENABLED:
        if (ring.count == 0 && first_sample_ready && samples_consumed > 0)
        {
            pwm_disable();
            state              = STATE_IDLE;
            entry_flag         = 1;
            first_sample_ready = 0;
        }
        break;

    case STATE_FAULT:
        // PWM disabled — waiting for explicit fault clear
        // TODO: add fault reset request and transition back to IDLE
        pwm_disable();
        break;
    }
}