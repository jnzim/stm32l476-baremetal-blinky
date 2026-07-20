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

static SysidStage   sysid_stage             = SYSID_STAGE_ALIGN;
static uint32_t     sysid_align_tick        = 0;
static int32_t      sysid_enc_offset        = 0;
static uint32_t     vel_chirp_settle_tick   = 0;

// =============================================================================
// Chirp state
// =============================================================================

static float sysid_t            = 0.0f;
static float chirp_angle_rad    = 0.0f;
static float sysid_f            = SYSID_F_START;

// =============================================================================
// CL step state
// =============================================================================

#define CL_STEP_AMPLITUDE    0.5      // A
#define CL_STEP_HOLD_TICKS   1000u    // 50ms at 20 kHz
#define CL_STEP_SETTLE_TICKS 1000u    // 50ms at 20 kHz
#define VEL_CHIRP_SETTLE_TICKS 20000U  // 1 second at 20 kHz

typedef enum
{
    CL_STEP_SETTLE = 0,
    CL_STEP_HIGH,
    CL_STEP_LOW,
    CL_STEP_ZERO,
    CL_STEP_DONE
} ClStepPhase;

static ClStepPhase cl_step_phase = CL_STEP_SETTLE;
static uint32_t    cl_step_tick  = 0;


// =============================================================================
// Velocity STEP
// =============================================================================

#define VEL_STEP_AMPLITUDE_A        0.10f
#define VEL_STEP_NUM_FREQUENCIES    5u
#define VEL_STEP_CYCLES             20u
#define VEL_STEP_DISCARD_CYCLES     3u
#define VEL_STEP_BETWEEN_TIME_S     0.25f

static const float vel_step_frequencies[VEL_STEP_NUM_FREQUENCIES] =
{
    1.0f,
    10.0f,
    30.0f,
    40.0f,
    50.0f
};

typedef enum
{
    VEL_STEP_STATE_SETTLE,
    VEL_STEP_STATE_RUN,
    VEL_STEP_STATE_BETWEEN,
    VEL_STEP_STATE_DONE
} VelStepState;

static VelStepState vel_step_state = VEL_STEP_STATE_SETTLE;

static uint32_t vel_step_index       = 0u;
static uint32_t vel_step_cycle_count = 0u;
static float vel_step_angle_rad      = 0.0f;
static float vel_step_between_time   = 0.0f;

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

// static void run_current_test(void)
// {
//     float theta = foc_theta_from_encoder();

//     current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

//     iq_cmd = 0.2f;  // Global, if you want telemetry to show it

//     foc_vd_applied = 0.0f;
//     foc_vq_applied = 0.5f;

//     pwm_apply_dq(
//         foc_vd_applied,
//         foc_vq_applied,
//         theta
//     );

//     sysid_t += SYSID_DT;

//     if (sysid_t >= 1.0f)
//     {
//         pwm_apply_dq(0.0f, 0.0f, theta);
//         sysid_stage = SYSID_STAGE_IDLE;
//     }
// }

static void run_current_test(void)
{
    current_feedback_get_phase_amps(
        &ia_meas,
        &ib_meas,
        &ic_meas
    );

    pwm_apply_phase_volts(0.5f, -0.25f, -0.25f);

    sysid_t += SYSID_DT;

    if (sysid_t >= 1.0f)
    {
        pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
        sysid_stage = SYSID_STAGE_IDLE;
    }
}






// =============================================================================
// run_chirp — OL Vd chirp, theta=0, proven from sysid bring-up
// =============================================================================

static void run_chirp(void)
{
    // instantaneous frequency of a logarithmic chirp
    sysid_f = SYSID_F_START * powf(SYSID_F_END / SYSID_F_START, sysid_t / SYSID_DURATION);

    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
    if (chirp_angle_rad >= FOC_TWO_PI)
        chirp_angle_rad -= FOC_TWO_PI;

    foc_vd_applied = SYSID_AMPLITUDE * sinf(chirp_angle_rad);
    foc_vq_applied = 0.0f;

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

    // Current feedback at theta=0 (d-axis locked)
    current_feedback_get_dq(0.0f, &i_d_meas, &i_q_meas);

    sysid_t += SYSID_DT;
    if (sysid_t >= SYSID_DURATION)
        sysid_stage = SYSID_STAGE_IDLE;
}
// =============================================================================
// run_vel_chirp — CL iq chirp for mechanical plant identification
//
// Current loop remains closed.
// Input:  measured iq
// Output: encoder-derived mechanical velocity
// Plant:  omega(s) / iq(s)
// =============================================================================


static void run_vel_chirp(void)
{
     const float theta = foc_theta_from_encoder();

    if (vel_chirp_settle_tick < VEL_CHIRP_SETTLE_TICKS)
    {
        iq_cmd = 0.0f;

        foc_vd_applied = 0.0f;
        foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

        pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);

        ++vel_chirp_settle_tick;
        return;
    }
    

    if (!current_feedback_sample_valid()) {return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    sysid_f = VEL_CHIRP_F_START * powf(VEL_CHIRP_F_END / VEL_CHIRP_F_START, sysid_t / VEL_CHIRP_DURATION);

    // advance the sine angle  by the correct amount on every 20 kHz control tick:               
    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;

    if (chirp_angle_rad >= FOC_TWO_PI)
        chirp_angle_rad -= FOC_TWO_PI;

    iq_cmd = VEL_CHIRP_AMPLITUDE * cosf(chirp_angle_rad);

    foc_vd_applied = 0.0f;

    /* Preserve the sign convention confirmed by the current-step test. */
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas,SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);

    sysid_t += SYSID_DT;

    if (sysid_t >= VEL_CHIRP_DURATION)
    {
        iq_cmd         = 0.0f;
        foc_vd_applied = 0.0f;
        foc_vq_applied = 0.0f;
        pwm_apply_dq(0.0f, 0.0f, theta);
        loops_reset();
        sysid_stage = SYSID_STAGE_IDLE;
    }
}

// =============================================================================
// run_vel_stepped_sine — CL iq step sine sweep for mechanical plant identification
//
// Current loop remains closed.
// Input:  measured iq
// Output: encoder-derived mechanical velocity
// Plant:  omega(s) / iq(s)
// =============================================================================
static void run_vel_stepped_sine(void)
{
    const float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) {return; }

    current_feedback_get_phase_amps(&ia_meas,&ib_meas,&ic_meas);

    current_feedback_get_dq(theta,&i_d_meas,&i_q_meas);

    switch (vel_step_state)
    {
        case VEL_STEP_STATE_SETTLE:
            iq_cmd = 0.0f;
            sysid_f = 0.0f;

            ++vel_chirp_settle_tick;

            if (vel_chirp_settle_tick >= VEL_CHIRP_SETTLE_TICKS)
            {
                vel_step_index       = 0u;
                vel_step_cycle_count = 0u;
                vel_step_angle_rad   = 0.0f;
                vel_step_state       = VEL_STEP_STATE_RUN;
            }
            break;

        case VEL_STEP_STATE_RUN:
        {
            const float frequency_hz =vel_step_frequencies[vel_step_index];

            sysid_f = frequency_hz;

            iq_cmd = VEL_STEP_AMPLITUDE_A * cosf(vel_step_angle_rad);

            vel_step_angle_rad += FOC_TWO_PI * frequency_hz * SYSID_DT;

            if (vel_step_angle_rad >= FOC_TWO_PI)
            {
                vel_step_angle_rad -= FOC_TWO_PI;
                ++vel_step_cycle_count;
            }

            if (vel_step_cycle_count >= VEL_STEP_CYCLES)
            {
                iq_cmd = 0.0f;
                sysid_f = 0.0f;

                vel_step_angle_rad     = 0.0f;
                vel_step_cycle_count   = 0u;
                vel_step_between_time  = 0.0f;

                ++vel_step_index;

                if (vel_step_index >= VEL_STEP_NUM_FREQUENCIES)
                {
                    vel_step_state = VEL_STEP_STATE_DONE;
                }
                else
                {
                    vel_step_state = VEL_STEP_STATE_BETWEEN;
                }
            }
            break;
        }

        case VEL_STEP_STATE_BETWEEN:
            iq_cmd = 0.0f;
            sysid_f = 0.0f;

            vel_step_between_time += SYSID_DT;

            if (vel_step_between_time >= VEL_STEP_BETWEEN_TIME_S)
            {
                vel_step_between_time = 0.0f;
                vel_step_angle_rad    = 0.0f;
                vel_step_cycle_count  = 0u;
                vel_step_state        = VEL_STEP_STATE_RUN;
            }
            break;

        case VEL_STEP_STATE_DONE:
        default:
            iq_cmd         = 0.0f;
            sysid_f        = 0.0f;
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;

            pwm_apply_dq(0.0f, 0.0f, theta);

            loops_reset();

            vel_step_state         = VEL_STEP_STATE_SETTLE;
            vel_step_index         = 0u;
            vel_step_cycle_count   = 0u;
            vel_step_angle_rad     = 0.0f;
            vel_step_between_time  = 0.0f;
            vel_chirp_settle_tick  = 0u;

            sysid_stage = SYSID_STAGE_IDLE;
            return;
    }

    foc_vd_applied = 0.0f;

    foc_vq_applied = pi_step(&current_loop,iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}



// =============================================================================
// run_cl_step — CL iq step response
//
// Current loop is closed. iq_cmd steps from 0 -> CL_STEP_AMPLITUDE -> 0.
// vd = 0, theta from encoder.
// Logs iq_cmd / iq_meas / vq_applied via spi_sysid_update_latest().
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
            iq_cmd = -1 * CL_STEP_AMPLITUDE;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_ZERO;
            }
            break;

        case CL_STEP_ZERO:
            iq_cmd = 0;
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
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, theta);
            return;
    }

    // Current loop @ 20 kHz
    foc_vd_applied = 0.0f;
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);
    

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
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
            foc_vd_applied = V_ALIGN;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

            if (++sysid_align_tick >= 2000)
            {
                sysid_enc_offset = encoder_get_position();
                loops_reset();
                sysid_stage      = SYSID_STAGE_RUN;
            }
        }
        break;

        // ---------------------------------------------------------------------
        case SYSID_STAGE_RUN:
        {
        #if SYSID_TEST == SYSID_TEST_CHIRP
            run_chirp();
            //run_current_test();
        #elif SYSID_TEST == SYSID_TEST_CL_STEP
            run_cl_step(); 
        #elif SYSID_TEST == SYSID_TEST_VEL_CHIRP
            run_vel_chirp();
        #elif SYSID_TEST == SYSID_STEP_SINE
            run_vel_stepped_sine();
        #endif
        }
        break;


        // ---------------------------------------------------------------------
        case SYSID_STAGE_IDLE:
        {
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, 0.0f);
        }
        break;
    }

    // -------------------------------------------------------------------------
    // Telemetry — written every ISR tick regardless of stage.
    // -------------------------------------------------------------------------

    uint16_t flags = (uint16_t)sysid_stage;

#if SYSID_TEST == SYSID_TEST_CHIRP
    float theta_telem = 0.0f;
#else
    float theta_telem = foc_theta_from_encoder();
#endif

    int32_t enc_pos = encoder_get_position();

    spi_sysid_update_latest(
    (int16_t)(enc_pos >> 16),              /* enc_hi        */
    (int16_t)(enc_pos & 0xFFFF),           /* enc_lo        */
    (int16_t)(sysid_f),                    /* sysid_f_Hz     */
    (int16_t)(i_d_meas * 1000.0f),         /* id_mA          */
    (int16_t)(i_q_meas * 1000.0f),         /* iq_mA          */
    (int16_t)(foc_vd_applied * 1000.0f),   /* vd_mV          */
    (int16_t)(foc_vq_applied * 1000.0f),   /* vq_mV          */
    (int16_t)(theta_telem * 1000.0f),      /* theta_mrad     */
    (int16_t)(ia_meas * 1000.0f),          /* ia_mA          */
    (int16_t)(ib_meas * 1000.0f),          /* ib_mA          */
    (int16_t)(iq_cmd * 1000.0f),           /* iq_cmd_mA      */
   //  (int16_t)(ic_meas * 1000.0f),           /* iq_cmd_mA      */
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

    sysid_t         = 0.0f;
    chirp_angle_rad = 0.0f;
    sysid_f         = SYSID_F_START;

    cl_step_phase = CL_STEP_SETTLE;
    cl_step_tick  = 0;

    vel_step_state        = VEL_STEP_STATE_SETTLE;
    vel_step_index        = 0u;
    vel_step_cycle_count  = 0u;
    vel_step_angle_rad    = 0.0f;
    vel_step_between_time = 0.0f;

    iq_cmd         = 0.0f;
    foc_vd_applied = 0.0f;
    foc_vq_applied = 0.0f;

    ia_meas = ib_meas = ic_meas = 0.0f;
    i_d_meas = i_q_meas = 0.0f;
    vel_chirp_settle_tick = 0;
}
