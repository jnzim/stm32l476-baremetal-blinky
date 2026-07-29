// foc_sysid.c — system identification mode FOC
//
// Extracted from main.c (jz-f411). Logic unchanged from working sysid firmware.
// Added: FOC_SYSID_CL_STEP test mode for closed-loop current step response.
//        FOC_SYSID_CL_VEL_STEP test mode for closed-loop velocity step response.
//
// Internal test selected by SYSID_TEST in config.h:
//   SYSID_TEST_CHIRP        — OL Vd chirp sweep (proven, existing)
//   SYSID_TEST_CL_STEP      — CL iq step, current loop closed, logs iq_ref/iq_meas
//   SYSID_TEST_CL_VEL_STEP  — CL velocity step, velocity loop closed
//                             sysid_f_Hz slot  → vel_cmd  [mrad/s]  Python: val/1000.0
//                             iq_cmd_mA slot   → vel_meas [counts/s] Python: val*(2pi/8192)

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
    SYSID_STAGE_ALIGN_PERTURB,
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

#define CL_STEP_AMPLITUDE       0.1f        // A
#define CL_STEP_VEL_AMPLITUDE   10.0f       // rad/s
#define CL_STEP_HOLD_TICKS      80000u       
#define CL_STEP_SETTLE_TICKS    1000u     
#define VEL_CHIRP_SETTLE_TICKS  20000u     

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

static VelStepState vel_step_state        = VEL_STEP_STATE_SETTLE;
static uint32_t     vel_step_index        = 0u;
static uint32_t     vel_step_cycle_count  = 0u;
static float        vel_step_angle_rad    = 0.0f;
static float        vel_step_between_time = 0.0f;

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

// Velocity step telemetry
// vel_cmd_rad_sec is defined in loops.c, declared extern in loops.h
static float vel_meas_counts = 0.0f;   // measured velocity [counts/s] — raw from encoder


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
// run_voltage_test
// =============================================================================

static void run_voltage_test(void)
{
    pwm_apply_phase_volts(0.5f, -0.25f, -0.25f);

    sysid_t += SYSID_DT;

    if (sysid_t >= 30.0f)
    {
        pwm_apply_phase_volts(0.0f, 0.0f, 0.0f);
        sysid_stage = SYSID_STAGE_IDLE;
    }
}


// =============================================================================
// run_current_test
// =============================================================================

static void run_current_test(void)
{
    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);

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
    sysid_f = SYSID_F_START * powf(SYSID_F_END / SYSID_F_START, sysid_t / SYSID_DURATION);

    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
    if (chirp_angle_rad >= FOC_TWO_PI)
        chirp_angle_rad -= FOC_TWO_PI;

    foc_vd_applied = SYSID_AMPLITUDE * sinf(chirp_angle_rad);
    foc_vq_applied = 0.0f;

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, 0.0f);

    current_feedback_get_dq(0.0f, &i_d_meas, &i_q_meas);

    sysid_t += SYSID_DT;
    if (sysid_t >= SYSID_DURATION)
        sysid_stage = SYSID_STAGE_IDLE;
}


// =============================================================================
// run_vel_chirp — CL iq chirp for mechanical plant identification
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

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    sysid_f = VEL_CHIRP_F_START * powf(VEL_CHIRP_F_END / VEL_CHIRP_F_START, sysid_t / VEL_CHIRP_DURATION);

    chirp_angle_rad += FOC_TWO_PI * sysid_f * SYSID_DT;
    if (chirp_angle_rad >= FOC_TWO_PI)
    {
        chirp_angle_rad -= FOC_TWO_PI;
    }
    iq_cmd = VEL_CHIRP_AMPLITUDE * cosf(chirp_angle_rad);

    foc_vd_applied = 0.0f;
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

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
// run_vel_stepped_sine — CL iq stepped sine sweep
// =============================================================================

static void run_vel_stepped_sine(void)
{
    const float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    switch (vel_step_state)
    {
        case VEL_STEP_STATE_SETTLE:
            iq_cmd  = 0.0f;
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
            const float frequency_hz = vel_step_frequencies[vel_step_index];
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
                iq_cmd  = 0.0f;
                sysid_f = 0.0f;

                vel_step_angle_rad    = 0.0f;
                vel_step_cycle_count  = 0u;
                vel_step_between_time = 0.0f;

                ++vel_step_index;

                vel_step_state = (vel_step_index >= VEL_STEP_NUM_FREQUENCIES)
                                 ? VEL_STEP_STATE_DONE
                                 : VEL_STEP_STATE_BETWEEN;
            }
            break;
        }

        case VEL_STEP_STATE_BETWEEN:
            iq_cmd  = 0.0f;
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

            vel_step_state        = VEL_STEP_STATE_SETTLE;
            vel_step_index        = 0u;
            vel_step_cycle_count  = 0u;
            vel_step_angle_rad    = 0.0f;
            vel_step_between_time = 0.0f;
            vel_chirp_settle_tick = 0u;

            sysid_stage = SYSID_STAGE_IDLE;
            return;
    }

    foc_vd_applied = 0.0f;
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_cl_step — CL iq step response
// =============================================================================

static void run_cl_step(void)
{

    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid())
        return;

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);
    vel_meas_counts = encoder_get_velocity();  // ADD HERE

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
            iq_cmd = -1.0f * CL_STEP_AMPLITUDE;
            if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
            {
                cl_step_tick  = 0;
                cl_step_phase = CL_STEP_ZERO;
            }
            break;

        case CL_STEP_ZERO:
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
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, theta);
            return;
    }

    foc_vd_applied = 0.0f;
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// run_cl_vel_step — CL velocity loop step response
//
// Architecture:
//   Outer loop (velocity): runs every 4th ISR tick = 5 kHz (DT_VELOCITY)
//     vel_cmd → [vel PI] → iq_cmd  [A]
//   Inner loop (current): runs every ISR tick = 20 kHz (SYSID_DT)
//     iq_cmd  → [cur PI] → vq_applied  [V]
//
// Telemetry slot usage:
//   sysid_f_Hz slot → vel_cmd_rad_sec * 1000  [mrad/s]
//                      Python: df["sysid_f"] / 1000.0  → rad/s
//   iq_cmd_mA slot  → vel_meas_counts          [counts/s, int16]
//                      Python: df["iq_cmd_mA"] * (2*pi / 8192)  → rad/s
// =============================================================================

#define VEL_LOOP_DECIMATE  4u   // 20 kHz / 4 = 5 kHz

static uint32_t vel_loop_tick = 0;

static void run_cl_vel_step(void)
{
    float theta = foc_theta_from_encoder();

    if (!current_feedback_sample_valid()) { return; }

    current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
    current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

    vel_meas_counts = encoder_get_velocity();   // counts/s, stored for telemetry

    // ── Outer velocity loop — 5 kHz ──────────────────────────────────────────
    if (++vel_loop_tick >= VEL_LOOP_DECIMATE)
    {
        vel_loop_tick = 0;

        float vel_meas_rad = vel_meas_counts * (FOC_TWO_PI / ENCODER_CPR);

        switch (cl_step_phase)
        {
            case CL_STEP_SETTLE:
                vel_cmd_rad_sec = 0.0f;
                if (++cl_step_tick >= CL_STEP_SETTLE_TICKS)
                {
                    cl_step_tick             = 0;
                    cl_step_phase            = CL_STEP_HIGH;
                    velocity_loop.integrator = 0.0f;  // clear vel integrator only
                }
                break;

            case CL_STEP_HIGH:
                vel_cmd_rad_sec = CL_STEP_VEL_AMPLITUDE;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_LOW;
                }
                break;

            case CL_STEP_LOW:
                vel_cmd_rad_sec = -1.0f * CL_STEP_VEL_AMPLITUDE;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_ZERO;
                }
                break;

            case CL_STEP_ZERO:
                vel_cmd_rad_sec = 0.0f;
                if (++cl_step_tick >= CL_STEP_HOLD_TICKS)
                {
                    cl_step_tick  = 0;
                    cl_step_phase = CL_STEP_DONE;
                    sysid_stage   = SYSID_STAGE_IDLE;
                }
                break;

            case CL_STEP_DONE:
            default:
                vel_cmd_rad_sec = 0.0f;
                iq_cmd          = 0.0f;
                foc_vd_applied  = 0.0f;
                foc_vq_applied  = 0.0f;
                pwm_apply_dq(0.0f, 0.0f, theta);
                return;
        }

        // Velocity PI → iq_cmd [A], clamped to ±0.5A
        iq_cmd = pi_step(&velocity_loop, vel_cmd_rad_sec - vel_meas_rad, DT_VELOCITY);
    }

    // ── Inner current loop — 20 kHz ──────────────────────────────────────────
    foc_vd_applied = 0.0f;
    foc_vq_applied = pi_step(&current_loop, iq_cmd - i_q_meas, SYSID_DT);

    pwm_apply_dq(foc_vd_applied, foc_vq_applied, theta);
}


// =============================================================================
// foc_sysid_step — called from TIM1_UP_TIM10_IRQHandler at 20 kHz
// =============================================================================
#define SYSID_ALIGN_REPEATS   10u
#define SYSID_ALIGN_TICKS     40000u   /* same hold time you already use */
#define SYSID_PERTURB_TICKS   500u    /* kick duration between repeats */
#define SYSID_PERTURB_VQ      0.3f    /* torque-axis kick, not d-axis */

static uint32_t sysid_align_repeat = 0u;
static uint32_t sysid_perturb_tick = 0u;
static uint32_t sysid_idle_tick = 0u;
static int32_t  sysid_align_log[SYSID_ALIGN_REPEATS];   /* <-- set breakpoint, read this array */

void foc_sysid_step(void)
{
    switch (sysid_stage)
    {
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





        case SYSID_STAGE_RUN:
        {
        #if   SYSID_TEST == SYSID_TEST_CHIRP
            run_chirp();
        #elif SYSID_TEST == SYSID_TEST_CL_STEP
            run_cl_step();
        #elif SYSID_TEST == SYSID_TEST_VEL_CHIRP
            run_vel_chirp();
        #elif SYSID_TEST == SYSID_STEP_SINE
            run_vel_stepped_sine();
        #elif SYSID_TEST == SYSID_TEST_CL_VEL_STEP
            run_cl_vel_step();
        #elif SYSID_TEST == RIPPLE_DEBUG
            run_cl_step();
        #endif
        }
        break;

        case SYSID_STAGE_IDLE:
        {
            foc_vd_applied = 0.0f;
            foc_vq_applied = 0.0f;
            pwm_apply_dq(0.0f, 0.0f, 0.0f);

            if (++sysid_idle_tick >= 80000)
            {
                sysid_align_tick = 0u;
                sysid_idle_tick = 0;

            }


        }
        break;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    uint16_t flags = (uint16_t)sysid_stage;

#if SYSID_TEST == SYSID_TEST_CHIRP
    float theta_telem = 0.0f;
#else
    float theta_telem = foc_theta_from_encoder();
#endif

    int32_t enc_pos = encoder_get_position();

#if SYSID_TEST == SYSID_TEST_CL_VEL_STEP

    /* Closed-loop velocity step:
     *   sysid_f slot -> velocity command [mrad/s]
     *   last slot    -> measured velocity [counts/s]
     */
    int16_t sysid_f_slot = (int16_t)(vel_cmd_rad_sec * 1000.0f);
    int16_t last_slot    = (int16_t)vel_meas_counts;

#elif SYSID_TEST == RIPPLE_DEBUG

    /* Constant-iq ripple test:
     *   sysid_f slot -> actual iq command [mA]
     *   last slot    -> same measured velocity used by velocity controller
     */
    #define VEL_TELEM_DIV 8.0f
    int16_t sysid_f_slot = (int16_t)(iq_cmd * 1000.0f);
    int16_t last_slot = (int16_t)(vel_meas_counts / VEL_TELEM_DIV);


#else

    int16_t sysid_f_slot = (int16_t)sysid_f;
    int16_t last_slot    = (int16_t)(iq_cmd * 1000.0f);

#endif

    spi_sysid_update_latest(
        (int16_t)(enc_pos >> 16),               /* enc_hi         */
        (int16_t)(enc_pos & 0xFFFF),            /* enc_lo         */
        sysid_f_slot,                           /* sysid_f_Hz  OR vel_cmd mrad/s  */
        (int16_t)(i_d_meas * 1000.0f),          /* id_mA          */
        (int16_t)(i_q_meas * 1000.0f),          /* iq_mA          */
        (int16_t)(foc_vd_applied * 1000.0f),    /* vd_mV          */
        (int16_t)(foc_vq_applied * 1000.0f),    /* vq_mV          */
        (int16_t)(theta_telem * 1000.0f),       /* theta_mrad     */
        (int16_t)(ia_meas * 1000.0f),           /* ia_mA          */
        (int16_t)(ib_meas * 1000.0f),           /* ib_mA          */
        last_slot,                               /* iq_cmd_mA  OR vel_meas counts/s */
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

    iq_cmd          = 0.0f;
    foc_vd_applied  = 0.0f;
    foc_vq_applied  = 0.0f;
    vel_cmd_rad_sec = 0.0f;
    vel_meas_counts = 0.0f;
    vel_loop_tick   = 0u;

    ia_meas  = ib_meas = ic_meas = 0.0f;
    i_d_meas = i_q_meas = 0.0f;

    vel_chirp_settle_tick = 0u;
}