
#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "clock.h"
#include "spi.h"
#include "tim1.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "plant.h"
#include "protocol.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"


/* =============================================================================
 * Alignment timing
 *
 * ALIGN_TICKS is counted from SysTick_Handler().
 * SysTick = 1 kHz, so:
 *
 *   1000 ticks = 1.0 second
 *
 * =============================================================================*/
#define ALIGN_TICKS 1000u


/* =============================================================================
 * Bench bring-up voltages
 *
 * These are intentionally modest.
 * Increase only after encoder angle and current feedback look sane.
 * =============================================================================*/
#define V_ALIGN  3.0f
#define V_RUN    1.5f
#define ENC_DIR  (+1.0f)


/* =============================================================================
 * FOC / encoder constants
 * =============================================================================*/
#define FOC_TWO_PI 6.28318530718f


typedef enum
{
    FOC_STAGE_ALIGN = 0,
    FOC_STAGE_RUN
} FocStage;


/* =============================================================================
 * FOC commutation state
 * =============================================================================*/
static FocStage foc_stage      = FOC_STAGE_ALIGN;
static uint32_t foc_align_tick = 0;
static int32_t  encoder_offset = 0;


/* =============================================================================
 * Global system state
 * =============================================================================*/
volatile uint32_t tick_ms = 0;
volatile bool     system_initialized = false;


/* =============================================================================
 * Original profile/control scaffold state
 * =============================================================================*/
static uint32_t div_1k = 0;
static uint32_t div_5k = 0;

static bool     move_in_progress = false;
static uint32_t prev_count = 0;
static int32_t  profile_vel_cmd = 0;


/* =============================================================================
 * Current feedback debug values
 *
 * ia/ib/ic/i_d/i_q are updated from ADC/DMA when a fresh sample is available.
 *
 * i_d_plot / i_q_plot are low-pass filtered versions for telemetry display only.
 * Do not use the plot-filtered values for control.
 * =============================================================================*/
static float ia_meas  = 0.0f;
static float ib_meas  = 0.0f;
static float ic_meas  = 0.0f;
static float i_d_meas = 0.0f;
static float i_q_meas = 0.0f;

static float i_d_plot = 0.0f;
static float i_q_plot = 0.0f;


/* =============================================================================
 * Applied voltage debug values
 *
 * These reflect the actual voltage-mode FOC command currently sent to PWM.
 * Telemetry should use these during bring-up, not the old simulated v_q_cmd.
 * =============================================================================*/
static float foc_vq_applied = 0.0f;
static float foc_vd_applied = 0.0f;


/* =============================================================================
 * foc_theta_from_encoder
 *
 * Electrical angle from encoder, referenced to captured alignment offset.
 *
 *   theta_mech = encoder counts -> mechanical radians
 *   theta_elec = pole_pairs * theta_mech + offset
 *
 * encoder_offset is captured after d-axis alignment.
 * =============================================================================*/
static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - encoder_offset;

    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;

    theta_elec = fmodf(theta_elec, FOC_TWO_PI);

    if (theta_elec < 0.0f)
        theta_elec += FOC_TWO_PI;

    return theta_elec;
}


/* =============================================================================
 * run_foc_commutation
 *
 * Temporary encoder-angle voltage-mode commutation test.
 *
 * This is NOT closed current-loop FOC yet.
 *
 * Stage 1:
 *   Apply d-axis voltage at electrical angle zero.
 *   Rotor pulls into alignment.
 *
 * Stage 2:
 *   Capture encoder position as electrical zero.
 *   Use encoder-derived electrical angle and apply q-axis voltage.
 *
 * =============================================================================*/
static void run_foc_commutation(void)
{
    switch (foc_stage)
    {
        case FOC_STAGE_ALIGN:
        {
            /*
             * Force d-axis at electrical zero.
             *
             * v_q = 0
             * v_d = V_ALIGN
             * theta = 0
             */
            foc_vq_applied = 0.0f;
            foc_vd_applied = V_ALIGN;

            pwm_apply_vq(foc_vq_applied, foc_vd_applied, 0.0f);

            if (++foc_align_tick >= ALIGN_TICKS)
            {
                /*
                 * Rotor has settled. Capture this encoder count as electrical zero.
                 */
                encoder_offset = encoder_get_position();

                foc_stage = FOC_STAGE_RUN;
            }
        }
        break;

        case FOC_STAGE_RUN:
        {
            /*
             * Use encoder-derived electrical angle.
             *
             * Do not add 90 degrees here.
             * q-axis behavior comes from v_q.
             */
            float theta_elec = foc_theta_from_encoder();

            foc_vq_applied = V_RUN;
            foc_vd_applied = 0.0f;

            pwm_apply_vq(foc_vq_applied, foc_vd_applied, theta_elec);
        }
        break;

        default:
        {
            foc_stage = FOC_STAGE_ALIGN;
            foc_align_tick = 0;
            encoder_offset = 0;
        }
        break;
    }
}


/* =============================================================================
 * foc_commutation_reset
 *
 * Optional reset for re-enable / fault recovery.
 * =============================================================================*/
void foc_commutation_reset(void)
{
    foc_stage      = FOC_STAGE_ALIGN;
    foc_align_tick = 0;
    encoder_offset = 0;

    foc_vq_applied = 0.0f;
    foc_vd_applied = 0.0f;
}


/* =============================================================================
 * SysTick_Handler — 1 kHz
 *
 * Runs temporary voltage-mode FOC commutation test and drive state machine.
 * =============================================================================*/
void SysTick_Handler(void)
{
    tick_ms++;

    if (!system_initialized)
        return;

  //  run_foc_commutation();

    drive_sm_run();
}


void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;

    if (!system_initialized)
        return;

    /*
     * 20 kHz sysid ISR.
     *
     * Keep:
     *   - encoder update
     *   - voltage-mode FOC PWM command
     *   - current feedback update
     *   - SPI sysid publish
     *
     * Remove for now:
     *   - trajectory ring
     *   - plant simulation
     *   - position/velocity loop scaffold
     *   - old telemetry generation
     */

    encoder_update(tick_ms);

    /*
     * Run the real PWM command here, not behind trajectory state.
     * This keeps the motor/FOC free-running for sysid.
     */
    run_foc_commutation();

    /*
     * Update current feedback.
     * Non-blocking: if no fresh sample, keep previous measured values.
     */
    current_feedback_update();

    float theta = foc_theta_from_encoder();
    uint16_t flags = 0;

    if (current_feedback_sample_valid())
    {
        current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
        current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);

        i_q_plot += 0.02f * (i_q_meas - i_q_plot);
        i_d_plot += 0.02f * (i_d_meas - i_d_plot);

        flags |= 0x0001u;   // fresh ADC/current sample consumed
    }

    /*
     * Publish latest sysid sample every TIM1 tick.
     *
     * RPi reads this as one packed SysIdSample:
     *   t, ia, ib, ic, id, iq, vd, vq, theta, adc fields, flags
     */
    spi_sysid_update_latest(
        (int16_t)(ia_meas * 1000.0f),
        (int16_t)(ib_meas * 1000.0f),
        (int16_t)(ic_meas * 1000.0f),
        (int16_t)(i_d_meas * 1000.0f),
        (int16_t)(i_q_meas * 1000.0f),
        (int16_t)(foc_vd_applied * 1000.0f),
        (int16_t)(foc_vq_applied * 1000.0f),
        (int16_t)(theta * 1000.0f),
        0,
        0,
        0,
        flags
    );
}


/* =============================================================================
 * main
 * =============================================================================*/
int main(void)
{
    /*
     * Enable FPU coprocessor.
     */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));

    /*
     * Core board/peripheral initialization.
     */
    clock_init();

    tim1_init();
    encoder_init();
    drive_init();
    spi_init();
    ring_init();

    /*
     * DRV8353 initialization.
     */
    drv8353_init();
    bool cfg_ok = drv8353_configure();

    /*
     * PWM and current feedback initialization.
     *
     * PWM outputs remain disabled until pwm_enable().
     */
    pwm_init();
    current_feedback_init();

    /*
     * Enable DRV before current calibration so current-sense amplifiers are alive.
     *
     * PWM is still disabled here, so the motor should not be driven.
     */
    drv_enable_high();

    /*
     * Calibrate current feedback zero offsets with PWM disabled.
     */
    current_feedback_calibrate();

    /*
     * Now enable PWM output.
     */
    pwm_enable();

    /*
     * Optional DRV sanity reads.
     */
    volatile uint16_t fs1 = drv8353_read_reg(DRV8353_REG_FAULT_STATUS_1);
    volatile uint16_t vgs = drv8353_read_reg(DRV8353_REG_VGS_STATUS_2);
    volatile uint16_t dc  = drv8353_read_reg(DRV8353_REG_DRIVER_CONTROL);

    (void)cfg_ok;
    (void)fs1;
    (void)vgs;
    (void)dc;

    /*
     * Start 1 kHz SysTick only after PWM/DRV/current feedback are ready.
     */
    SysTick_Config(SystemCoreClock / 1000u);

    /*
     * Allow interrupt handlers to start doing work.
     */
    system_initialized = true;

    while (1)
    {
        __WFI();
    }
}
