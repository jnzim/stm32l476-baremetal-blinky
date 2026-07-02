// foc_sysid.c — system identification mode FOC
//
// Extracted from main.c (jz-f411). Logic unchanged from working sysid firmware.
// Added: FOC_SYSID_CL_STEP test mode for closed-loop current step response.
//
// Internal test selected by SYSID_TEST in config.h:
//   SYSID_TEST_CHIRP    — OL Vd chirp sweep (proven, existing)
//   SYSID_TEST_CL_STEP  — CL iq step, current loop closed, logs iq_ref/iq_meas

#include "foc_sysid.h"
#include "config.h"
#include "encoder.h"
#include "current_feedback.h"
#include "pwm.h"
#include "loops.h"
#include "spi.h"

#include <math.h>
#include <stdint.h>

// =============================================================================
// Alignment stage
// =============================================================================

typedef enum
{
    SYSID_STAGE_ALIGN = 0,
    SYSID_STAGE_RUN,
    SYSID_STAGE_IDLE
} SysidStage;

static SysidStage sysid_stage      = SYSID_STAGE_ALIGN;
static uint32_t   sysid_align_tick = 0;
static int32_t    sysid_enc_offset = 0;

// =============================================================================
// Chirp state
// =============================================================================

static float sysid_t     = 0.0f;
static float sysid_phase = 0.0f;
static float sysid_f     = SYSID_F_START;

// =============================================================================
// CL step state
// =============================================================================

#define CL_STEP_SETTLE_TICKS  20000u    // 1 s at 20 kHz — let align field decay
#define CL_STEP_HOLD_TICKS    20000u    // 1 s step hold — enough for full settling
#define CL_STEP_AMPLITUDE     1.0f      // A — iq step magnitude, keep modest

typedef enum
{
    CL_STEP_SETTLE = 0,
    CL_STEP_HIGH,
    CL_STEP_LOW,
    CL_STEP_DONE
} ClStepPhase;

static ClStepPhase cl_step_phase = CL_STEP_SETTLE;
static uint32_t    cl_step_tick  = 0;

// =============================================================================
// Shared measured values (written each ISR, read by spi_sysid_update_latest)
// =============================================================================

static float ia_meas  = 0.0f;
static float ib_meas  = 0.0f;
static float ic_meas  = 0.0f;
static float i_d_meas = 0.0f;
static float i_q_meas = 0.0f;

static float foc_vd_applied = 0.0f;
static float foc_vq_applied = 0.0f;

// =============================================================================
// foc_theta_from_encoder — electrical angle relative to alignment offset
// =============================================================================

static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - sysid_enc_offset;

    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;

    theta_elec = fmodf(theta_elec, FOC_TWO_PI);
    if (theta_elec < 0.0f)
        theta_elec += FOC_TWO_PI;

    return theta_elec;
}

// =============================================================================
// run_chirp — OL Vd chirp, theta=0, proven from sysid bring-up
// =============================================================================

static void run_chirp(void)
{
    sysid_f = SYSID_F_START * powf(SYSID_F_END / SYSID_F_START,
                                    sysid_t / SYSID_DURATION);

    sysid_phase += 2.0f * M_PI * sysid_f * SYSID_DT;
    if (sysid_phase > 2.0f * M_PI)
        sysid_phase -= 2.0f * M_PI;

    foc_vd_applied = SYSID_AMPLITUDE * sinf(sysid_phase);
    foc_vq_applied = 0.0f;

    pwm_apply_vq(foc_vq_applied, foc_vd_applied, 0.0f);

    // Current feedback at theta=0 (d-axis locked)
    current_feedback_get_dq(0.0f, &i_d_meas, &i_q_meas);

    sysid_t += SYSID_DT;
    if (sysid_t >= SYSID_DURATION)
        sysid_stage = SYSID_STAGE_IDLE;
}

// =============================================================================
// run_cl_step — CL iq step response
//
// Current loop is closed. iq_cmd steps from 0 -> CL_STEP_AMPLITUDE -> 0.
// vd = 0, theta from encoder.
// Logs iq_ref / iq_meas / vq_applied via spi_sysid_update_latest().
// =============================================================================

static void run_cl_step(void)
{
    float theta = foc_theta_from_encoder();

    // Current feedback
    if (!current_feedback_sample_valid())
        return;

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    // Step sequencer
    switch (cl_step_phase)
    {
        case CL_STEP_SETTLE:
            // iq_cmd = 0, loop closed, wait for transients from align to decay
            iq_cmd = 0.0f;
            if (++cl_step_tick >= CL_STEP_SETTLE_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_HIGH;
            }
            break;

        case CL_STEP_HIGH:
            iq_cmd = CL_STEP_AMPLITUDE;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_LOW;
            }
            break;

        case CL_STEP_LOW:
            iq_cmd = 0.0f;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_DONE;
                sysid_stage   = SYSID_STAGE_IDLE;
            }
            break;

        case CL_STEP_DONE:
        default:
            iq_cmd         = 0.0f;
            foc_vq_applied = 0.0f;
            foc_vd_applied = 0.0f;
            pwm_apply_vq(0.0f, 0.0f, theta);
            return;
    }

    // Current loop @ 20 kHz — Kp/Ki from loops_reset(), tuned from sysid params
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);
    foc_vd_applied = 0.0f;

    pwm_apply_vq(foc_vq_applied, foc_vd_applied, theta);
}

// =============================================================================
// foc_sysid_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
// =============================================================================

void foc_sysid_step(void)
{
    switch (sysid_stage)
    {
        // ---------------------------------------------------------------------
        case SYSID_STAGE_ALIGN:
        {
            // Force d-axis at electrical zero to establish encoder reference.
            foc_vd_applied = V_ALIGN;
            foc_vq_applied = 0.0f;
            pwm_apply_vq(foc_vq_applied, foc_vd_applied, 0.0f);

            if (++sysid_align_tick >= ALIGN_TICKS)
            {
                sysid_enc_offset = encoder_get_position();
                sysid_stage      = SYSID_STAGE_RUN;
            }
        }
        break;

        // ---------------------------------------------------------------------
        case SYSID_STAGE_RUN:
        {
        #if SYSID_TEST == SYSID_TEST_CHIRP
            run_chirp();
        #elif SYSID_TEST == SYSID_TEST_CL_STEP
            run_cl_step();
        #endif
        }
        break;

        // ---------------------------------------------------------------------
        case SYSID_STAGE_IDLE:
        {
            // Test complete. Hold zero voltage. RPi data reader drains remainder.
            foc_vq_applied = 0.0f;
            foc_vd_applied = 0.0f;
            pwm_apply_vq(0.0f, 0.0f, 0.0f);
        }
        break;
    }

    // -------------------------------------------------------------------------
    // Telemetry — written every ISR tick regardless of stage.
    // RPi sysid reader ingests this stream at 10 kHz via separate project.
    // sysid_f is repurposed as step phase indicator in CL_STEP mode.
    // -------------------------------------------------------------------------

    uint16_t flags = (uint16_t)sysid_stage;

    // In CL_STEP mode theta is live from the encoder; in chirp theta=0 (d-axis locked).
    // Compute here so it's available for the telemetry call in both modes.
#if SYSID_TEST == SYSID_TEST_CL_STEP
    float theta_telem = foc_theta_from_encoder();
#else
    float theta_telem = 0.0f;
#endif

    int32_t enc_pos = encoder_get_position();

    spi_sysid_update_latest(
        (int16_t)(enc_pos >> 16),              /* enc_hi */
        (int16_t)(enc_pos & 0xFFFF),           /* enc_lo */
        (int16_t)sysid_f,                      /* sysid_f */
        (int16_t)(i_d_meas * 1000.0f),         /* id_mA   */
        (int16_t)(i_q_meas * 1000.0f),         /* iq_mA   */
        (int16_t)(foc_vd_applied * 1000.0f),   /* vd_mV   */
        (int16_t)(foc_vq_applied * 1000.0f),   /* vq_mV   */
        (int16_t)(theta_telem * 1000.0f),      /* theta   */
        (int16_t)(ia_meas * 1000.0f),          /* adc_a — repurpose for ia */
        (int16_t)(ib_meas * 1000.0f),          /* adc_b — repurpose for ib */
        0,
        flags
    );
}

// =============================================================================
// foc_sysid_reset — call on fault recovery or re-enable
// =============================================================================

void foc_sysid_reset(void)
{
    sysid_stage      = SYSID_STAGE_ALIGN;
    sysid_align_tick = 0;
    sysid_enc_offset = 0;

    sysid_t     = 0.0f;
    sysid_phase = 0.0f;
    sysid_f     = SYSID_F_START;

    cl_step_phase = CL_STEP_SETTLE;
    cl_step_tick  = 0;

    iq_cmd         = 0.0f;
    foc_vq_applied = 0.0f;
    foc_vd_applied = 0.0f;

    ia_meas = ib_meas = ic_meas = 0.0f;
    i_d_meas = i_q_meas = 0.0f;
}