// loops.c — controller state and reset
#include "loops.h"
#include "control.h"
#include "plant.h"
#include "spi.h"
#include <string.h>

extern PlantState plant;

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
//
// Resets all controller state, setpoints, and sequencing flags.
// Ensures clean start before first trajectory sample is consumed.
// ─────────────────────────────────────────────────────────────────────────────
void loops_reset(void)
{
    memset((void*)&telem_buf[1], 0, sizeof(TelemetryFrame));
    pi_init(&current_loop,  0.1f,  0.0f, -24.0f, 24.0f);
    pi_init(&velocity_loop, 0.01f, 0.0f,  -5.0f,  5.0f);
    p_init(&position_loop,  1.0f);

    vel_cmd  = 0.0f;
    iq_cmd   = 0.0f;
    v_q_cmd  = 0.0f;
    vel_div  = 0;

    samples_consumed   = 0;
    first_sample_ready = 0;

    plant_init(&plant);
}