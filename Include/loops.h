#pragma once
#include "control.h"
#include "plant.h"

// ── Controller state ──────────────────────────────────────────────────────────
extern PIState current_loop;
extern PIState velocity_loop;
extern PState  position_loop;

extern PlantState plant;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
extern float    vel_cmd;
extern float    iq_cmd;
extern float    v_q_cmd;

// ── Rate dividers ─────────────────────────────────────────────────────────────
extern uint32_t vel_div;

// ── Sequencing ────────────────────────────────────────────────────────────────
extern volatile uint8_t first_sample_ready;

// ── Trajectory consumption ────────────────────────────────────────────────────
extern volatile uint32_t samples_consumed;

// ── API ───────────────────────────────────────────────────────────────────────
void loops_reset(void);