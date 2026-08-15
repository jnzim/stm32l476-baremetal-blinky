#include "loops.h"
#include "control.h"
#include "spi.h"
#include "pwm.h"
#include <math.h>
#include "config.h"


// Zero-cancellation design from the fitted mechanical plant (K=248.095 rad/s
// per A, tau=13.30ms) at a 50 Hz BW target -- see bode_vel_plot.py. Re-fit
// after the intermittent phase-wiring connection was resolved; still the
// bare motor (stage not yet attached) -- K/tau land within a few percent of
// the original fit (238.05 / 12.98ms), confirming the motor itself hasn't
// changed, just re-measured cleanly this time.
#define VEL_KP              0.0168f    // A / (rad/s)
#define VEL_KI              1.2663f    // A / rad
#define VEL_IQ_LIMIT        0.5f


// ── Reset after each move ─────────────────────────────────────────────────────
volatile uint32_t samples_consumed = 0;

// ── Controller state ──────────────────────────────────────────────────────────
PIState current_loop;
PIState d_current_loop;
PIState velocity_loop;
PState  position_loop;

// ── Inter-loop setpoints ──────────────────────────────────────────────────────
float          vel_cmd_rad_sec  = 0.0f;
float          iq_cmd           = 0.0f;
volatile float v_q_cmd          = 0.0f; 

// ── Rate dividers ─────────────────────────────────────────────────────────────
uint32_t vel_div = 0;

// ── Sequencing ────────────────────────────────────────────────────────────────
volatile uint8_t first_sample_ready = 0;

// ─────────────────────────────────────────────────────────────────────────────
// loops_reset — called from drive.c on SERVO_ON entry
// ─────────────────────────────────────────────────────────────────────────────
void loops_reset(void)
{


    // Output clamp was +-10V, but volts_to_duty() physically can't produce
    // more than +-V_BUS/2 per phase (6V @ V_BUS=12V) -- the PI's own
    // anti-windup never saw real saturation until 10V, while pwm.c was
    // silently clipping at 6V the whole time. Clamp now matches the actual
    // achievable limit so the integrator stops winding up past what the
    // hardware can really deliver.
    pi_init(&current_loop, 3.86, 5561.0f, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    //pi_init(&current_loop, 7.78f, 11153.0f, -12.0f, 12.0f);
    // Same gains as the q-axis current loop -- Ld ~= Lq for this machine, so
    // the d-axis plant (id/vd) is the same R/L electrical dynamics. Cancels
    // the id = we*Lq*iq/R cross-coupling that shows up when vd is forced to
    // zero (see foc-sysid RIPPLE_DEBUG id_mean investigation).
    pi_init(&d_current_loop, 3.86, 5561.0f, -(V_BUS / 2.0f), (V_BUS / 2.0f));
    pi_init(&velocity_loop, VEL_KP, VEL_KI, -VEL_IQ_LIMIT, VEL_IQ_LIMIT);


    p_init(&position_loop, POSITION_LOOP_KP);

    vel_cmd_rad_sec    = 0.0f;
    iq_cmd             = 0.0f;
    v_q_cmd            = 0.0f;
    vel_div            = 0;
    samples_consumed   = 0;
    first_sample_ready = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// open_loop_step — called from TIM1 ISR when STATE_OPEN_LOOP active
//
// Advances electrical angle by d_theta per call, applies rotating voltage
// vector at magnitude v_mag with v_d=0. No closed-loop feedback.
// ─────────────────────────────────────────────────────────────────────────────
void open_loop_step(float *theta, float v_mag, float d_theta)
{
    *theta += d_theta;
    if (*theta >= 2.0f * M_PI) *theta -= 2.0f * M_PI;
    pwm_apply_dq(0.0f, v_mag, *theta);
}