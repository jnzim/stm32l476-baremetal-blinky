
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

//   @ 1 kHz  SysTick : 1000 ticks = 1.0 s
//   @ 20 kHz TIM10   : 20000 ticks = 1.0 s  
#define ALIGN_TICKS           1000u
 
// Voltages (volts). Bench bring-up values.
#define V_ALIGN               6.0f      // d-axis hold during align
#define V_RUN                 3.0f      // q-axis run voltage
#define ENC_DIR               (+1.0f)
 

typedef enum {
    FOC_STAGE_ALIGN = 0,
    FOC_STAGE_RUN
} FocStage;
 
static FocStage foc_stage      = FOC_STAGE_ALIGN;
static uint32_t foc_align_tick = 0;
static int32_t  encoder_offset = 0;     // encoder count at electrical zero


volatile uint32_t tick_ms = 0;
volatile bool     system_initialized = false;

static uint32_t div_1k = 0;
static uint32_t div_5k = 0;

static bool     move_in_progress = false;
static uint32_t prev_count = 0;
static int32_t  profile_vel_cmd = 0;


#define FOC_TWO_PI      6.28318530718f
#define FOC_TEST_VQ     5.0f


// Electrical angle from encoder, referenced to the captured offset.
static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - encoder_offset;
 
    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;
 
    // wrap into [0, 2pi)
    theta_elec = fmodf(theta_elec, FOC_TWO_PI);
    if (theta_elec < 0.0f) theta_elec += FOC_TWO_PI;
 
    return theta_elec;
}


// ── Call once per ISR tick. Make sure ALIGN_TICKS is consistant ────────────────────────
void run_foc_commutation(void)
{
    switch (foc_stage)
    {
        case FOC_STAGE_ALIGN:
        {
            // Force the d-axis at electrical zero. Rotor pulls into alignment.
            // v_q = 0, v_d = V_ALIGN, theta = 0.
            pwm_apply_vq(0.0f, V_ALIGN, 0.0f);
 
            if (++foc_align_tick >= ALIGN_TICKS)
            {
                // Rotor has settled at electrical zero. This encoder reading
                // IS electrical zero. Capture it as the offset.
                encoder_offset = encoder_get_position();
                foc_stage      = FOC_STAGE_RUN;
            }
        }
        break;
 
        case FOC_STAGE_RUN:
        {
            // Commutate from real rotor position. The q-axis vector is, by
            // construction of the inverse Park transform, 90 electrical degrees
            // ahead of the rotor d-axis — so do NOT add another 90 here.
            // If it locks instead of spinning, the offset is a quadrant off:
            // adjust encoder_offset by +/- (ENCODER_CPR / POLE_PAIRS / 4).
            float theta_elec = foc_theta_from_encoder();
            pwm_apply_vq(V_RUN, 0.0f, theta_elec);
        }
        break;
    }
}
 
// Optional: call to restart the align sequence (e.g. on re-enable).
void foc_commutation_reset(void)
{
    foc_stage      = FOC_STAGE_ALIGN;
    foc_align_tick = 0;
    encoder_offset = 0;
}
 

void SysTick_Handler(void)
{
    tick_ms++;
    if (!system_initialized) return;
    run_foc_commutation();
}



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
