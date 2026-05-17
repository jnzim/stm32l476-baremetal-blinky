#include "loops.h"
#include "control.h"
#include "spi.h"

// ── Reset after each move  ──────────────────────────────────
volatile uint32_t samples_consumed = 0;

// ── Controller state ──────────────────────────────────────────────────────────
PIState current_loop;
PIState velocity_loop;
PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
float    vel_cmd  = 0.0f;
float    iq_cmd   = 0.0f;
float    v_q_cmd  = 0.0f;

// ── Rate dividers ─────────────────────────────────────────────────────────────
uint32_t vel_div  = 0;

// ── Sequencing ────────────────────────────────────────────────────────────────
volatile uint8_t first_sample_ready = 0;

// ─────────────────────────────────────────────────────────────────────────────
// loops_reset — called from drive.c on STATE_ENABLED entry
// ─────────────────────────────────────────────────────────────────────────────
void loops_reset(void)
{
    pi_init(&current_loop,  3.0f, 0.0f, -24.0f, 24.0f);
    pi_init(&velocity_loop, 0.1f, 0.0f,  -5.0f,  5.0f);
    p_init(&position_loop, 10.0f);
    vel_cmd             = 0.0f;
    iq_cmd              = 0.0f;
    v_q_cmd             = 0.0f;
    vel_div             = 0;
    samples_consumed    = 0;
    first_sample_ready  = 0;
}