
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


/* =============================================================================
 * Global system state
 * =============================================================================*/
volatile uint32_t tick_ms = 0;
volatile bool     system_initialized = false;


/* =============================================================================
 * Original control / profile state
 *
 * Keep this scaffold alive because the project still needs:
 *   - RPi SPI profile loading
 *   - ring buffer flow control
 *   - telemetry frame population
 *   - old plant/profile debug path
 *
 * During the current open-loop FOC voltage-vector test, the motor command is
 * generated in SysTick_Handler().
 * =============================================================================*/
static uint32_t div_1k = 0;
static uint32_t div_5k = 0;

static bool     move_in_progress = false;
static uint32_t prev_count = 0;
static int32_t  profile_vel_cmd = 0;


/* =============================================================================
 * Open-loop FOC voltage-vector bring-up
 *
 * This is NOT closed-loop FOC yet.
 *
 * What this proves:
 *
 *   vq/vd command
 *      -> inverse Park
 *      -> inverse Clarke
 *      -> phase voltage commands
 *      -> PWM duty
 *      -> DRV8353
 *      -> motor rotation
 *
 * What this does NOT use yet:
 *   - encoder electrical angle
 *   - current feedback
 *   - id/iq current regulation
 *   - velocity loop
 *   - position loop
 *
 * Next real FOC step:
 *
 *      theta_mech = encoder_counts_to_rad(encoder.position);
 *      theta_elec = pole_pairs * theta_mech + encoder_offset;
 *
 * =============================================================================*/

#define FOC_TWO_PI      6.28318530718f
#define FOC_TEST_VQ     5.0f

/*
 * Electrical radians per SysTick.
 *
 * SysTick = 1 kHz, so:
 *
 *   0.001 rad/tick = 1 rad/s electrical
 *   0.010 rad/tick = 10 rad/s electrical
 *
 * Negative sign reverses the rotating voltage vector.
 */
#define FOC_THETA_STEP (0.2f)

static float foc_theta_open_loop = 0.0f;


/* =============================================================================
 * SysTick_Handler — 1 kHz open-loop FOC voltage-vector test
 *
 * For this bring-up stage, SysTick advances a fake electrical angle and applies
 * a fixed q-axis voltage vector.
 *
 * This is temporary. Once encoder offset/alignment works, the synthetic theta
 * ramp should be replaced by encoder-derived electrical angle.
 * =============================================================================*/
void SysTick_Handler(void)
{
    tick_ms++;

    if (!system_initialized)
        return;

    foc_theta_open_loop += FOC_THETA_STEP;

    /*
     * Wrap angle into [0, 2pi).
     * Need both checks because FOC_THETA_STEP may be positive or negative.
     */
    if (foc_theta_open_loop >= FOC_TWO_PI)
        foc_theta_open_loop -= FOC_TWO_PI;

    if (foc_theta_open_loop < 0.0f)
        foc_theta_open_loop += FOC_TWO_PI;

    /*
     * Open-loop FOC voltage synthesis.
     *
     * v_d = 0
     * v_q = fixed test voltage
     * theta = synthetic electrical angle ramp
     */
    pwm_apply_vq(FOC_TEST_VQ, 0.0f, foc_theta_open_loop);
}


/* =============================================================================
 * TIM1_UP_TIM10_IRQHandler — 20 kHz interrupt
 *
 * Existing project scaffold:
 *   - encoder update
 *   - simulated current / velocity / position loops
 *   - ring-buffer trajectory consumption
 *   - READY refill GPIO
 *   - telemetry population
 *
 * Important during current bring-up:
 *
 *   The real motor PWM output is currently commanded by SysTick_Handler()
 *   through pwm_apply_vq(FOC_TEST_VQ, 0, foc_theta_open_loop).
 *
 *   The old plant/profile loop below is kept for architecture continuity, but
 *   it should not directly write motor PWM during this open-loop FOC test.
 *
 * Later real FOC current-loop work should move here:
 *
 *   1. read phase currents
 *   2. Clarke transform
 *   3. Park transform using encoder-derived theta_elec
 *   4. run id/iq PI loops
 *   5. call pwm_apply_vq(v_q_cmd, v_d_cmd, theta_elec)
 *
 * =============================================================================*/
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;

    encoder_update(tick_ms);

    if (first_sample_ready)
    {
        /*
         * 20 kHz: current loop + plant model
         *
         * NOTE:
         * This currently updates the simulated plant/debug variables.
         * It does not command the real PWM output during the open-loop FOC test.
         */
        v_q_cmd = pi_step(&current_loop,
                          iq_cmd - plant.i_q,
                          1.0f / 20000.0f);

        plant_step(&plant, v_q_cmd, 1.0f / 20000.0f);

        /*
         * 5 kHz: velocity loop
         */
        if (++div_5k >= 4u)
        {
            div_5k = 0;

            float vel_err_rad_sec = vel_cmd_rad_sec - (float)plant.vel_rad;

            iq_cmd = pi_step(&velocity_loop,
                             vel_err_rad_sec,
                             1.0f / 5000.0f);
        }
    }

    /*
     * 1 kHz: position loop + trajectory pop + telemetry
     */
    if (++div_1k >= 20u)
    {
        div_1k = 0;

        /* ---------------------------------------------------------------------
         * Refill logic
         * ------------------------------------------------------------------ */

        uint32_t curr_count = ring_count();

        if ((prev_count == 0u) && (curr_count > 0u))
        {
            move_in_progress = true;
        }

        if (curr_count == 0u)
        {
            move_in_progress = false;
        }

        /*
         * Flow control to RPi.
         */
        if (move_in_progress && (curr_count < READY_THRESHOLD))
        {
            GPIOC->BSRR = (1u << READY_RING_REFILL);          // Set READY
        }
        else
        {
            GPIOC->BSRR = (1u << (READY_RING_REFILL + 16u));  // Clear READY
        }

        prev_count = curr_count;


        /* ---------------------------------------------------------------------
         * Position loop / telemetry logic
         * ------------------------------------------------------------------ */

        if (drive_is_servo_on() && (drive.samples_consumed < expected_samples))
        {
            TrajSlot s;

            if (ring_pop(&s))
            {
                drive.samples_consumed++;
                first_sample_ready = 1;

                /*
                 * Position loop step.
                 *
                 * This is still using the simulated plant position for now.
                 * Later this should move to encoder.position when the real
                 * servo loop is brought online.
                 */
                profile_vel_cmd = s.vel_cmd;

                float pos_err_cnt = (float)(s.pos_cmd - plant.pos_counts);

                vel_cmd_rad_sec =
                    (p_step(&position_loop, pos_err_cnt) / COUNTS_PER_RAD) +
                    ((float)profile_vel_cmd / COUNTS_PER_RAD) * FF_GAIN;


                /*
                 * Populate telemetry frame.
                 */
                telem_buf[1].pos_cmd = s.pos_cmd;

                telem_buf[1].pos_fbk = plant.pos_counts;
                // telem_buf[1].pos_fbk = encoder.position;

                telem_buf[1].vel_cmd = (int32_t)s.vel_cmd;

                telem_buf[1].vel_fbk = plant.vel_counts;
                // telem_buf[1].vel_fbk = encoder.velocity;

                telem_buf[1].timestamp_ms = tick_ms;
                telem_buf[1].drive_state = drive_get_state();
                telem_buf[1].fault_flags = drive.fault_flags;
                telem_buf[1].samples_consumed = drive.samples_consumed;

                telem_buf[1].iq_cmd  = (int16_t)(iq_cmd * 1000.0f);
                telem_buf[1].i_q_fbk = (int16_t)(plant.i_q * 1000.0f);
                telem_buf[1].v_q_cmd = (int16_t)(v_q_cmd * 1000.0f);
            }
        }
    }
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
    clock_init();       // 100 MHz system clock
    tim1_init();        // TIM1 PWM / update interrupt base
    encoder_init();     // quadrature encoder feedback
    drive_init();       // drive software state
    spi_init();         // RPi SPI transport
    ring_init();        // trajectory ring buffer

    /*
     * DRV8353 initialization.
     */
    drv8353_init();     // SPI1 + DRV GPIO + DRV awake
    bool cfg_ok = drv8353_configure();

    /*
     * PWM output initialization.
     */
    pwm_init();
    pwm_enable();

    /*
     * Enable gate driver after PWM is initialized.
     */
    drv_enable_high();

    /*
     * Optional DRV sanity reads.
     *
     * Keep these during bring-up so the debugger can inspect them.
     */
    volatile uint16_t fs1 = drv8353_read_reg(DRV8353_REG_FAULT_STATUS_1);
    volatile uint16_t vgs = drv8353_read_reg(DRV8353_REG_VGS_STATUS_2);
    volatile uint16_t dc  = drv8353_read_reg(DRV8353_REG_DRIVER_CONTROL);

    (void)cfg_ok;
    (void)fs1;
    (void)vgs;
    (void)dc;

    /*
     * Start 1 kHz SysTick only after PWM/DRV are ready.
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
