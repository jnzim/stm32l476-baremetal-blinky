#include "stm32f4xx.h"
#include "clock.h"
#include "spi.h"
#include "tim1.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "plant.h"
#include "protocol.h"
#include <stdint.h>
#include "config.h"
#include "encoder.h"
#include "drv8353.h"




/* =============================================================================
 * 
 * =============================================================================*/
volatile uint32_t   tick_ms             = 0;
static  uint32_t    div_1k              = 0;
static  uint32_t    div_5k              = 0;
static  bool        move_in_progress    = false;
static  uint32_t    prev_count          = 0;
static  int32_t     profile_vel_cmd     = 0;

/* =============================================================================
 * SysTick_Handler — 1 kHz
 * Calls drive state machine
 * =============================================================================*/
void SysTick_Handler(void)
{
    tick_ms++;
    drive_sm_run();
}

/* =============================================================================
 * TIM1_UP_TIM10_IRQHandler — 20 kHz (cascaded: 5 kHz, 1 kHz)
 *
 * 20 kHz: current loop + plant
 * 5 kHz:  velocity loop (every 4th tick)
 * 1 kHz:  position loop + trajectory pop (every 20th tick)
 * =============================================================================*/
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;
    encoder_update(tick_ms);

    
    if (first_sample_ready)
    {
        /* 20 kHz: current loop + plant */
        v_q_cmd = pi_step(&current_loop, iq_cmd - plant.i_q, 1.0f / 20000.0f);
        plant_step(&plant, v_q_cmd, 1.0f / 20000.0f);
        
        /* 5 kHz: velocity loop */
        if (++div_5k >= 4u)
        {
            div_5k = 0;
            float vel_err_rad_sec = vel_cmd_rad_sec - (float)plant.vel_rad;
            iq_cmd = pi_step(&velocity_loop, vel_err_rad_sec, 1.0f / 5000.0f);
        }
    }
    
    /* 1 kHz: position loop + trajectory pop */
    if (++div_1k >= 20u)
    {
        div_1k = 0;
        
        /* =============================================================================
        * Refill logic
        * =============================================================================*/
        uint32_t curr_count = ring_count();
        if (prev_count == 0 && curr_count > 0)  { move_in_progress = true; }
        if (curr_count == 0)                    { move_in_progress = false; }
        // Flow control
        if (move_in_progress && curr_count < READY_THRESHOLD) 
        {
            GPIOC->BSRR = (1u << READY_RING_REFILL);        // Set READY
        }
        else 
        {
            GPIOC->BSRR = (1u << (READY_RING_REFILL + 16)); // Clear READY
        }
        prev_count = curr_count;

        /* =============================================================================
        * Postion loop / telem logic
        * =============================================================================*/
        if (drive_is_servo_on() && drive.samples_consumed < expected_samples)
        {
            TrajSlot s;
            if (ring_pop(&s))
            {
                /* Update state */
                drive.samples_consumed++;
                first_sample_ready = 1;

                // Run postion loop step
                profile_vel_cmd =   s.vel_cmd;
                float pos_err_cnt = (float)(s.pos_cmd - plant.pos_counts);
                vel_cmd_rad_sec =   (p_step(&position_loop, pos_err_cnt) / COUNTS_PER_RAD) + 
                                    (profile_vel_cmd / COUNTS_PER_RAD) * FF_GAIN;

                /* Populate telemetry frame */
                telem_buf[1].pos_cmd          = s.pos_cmd;
                telem_buf[1].pos_fbk          = plant.pos_counts;
                //telem_buf[1].pos_fbk          = encoder.position;
                telem_buf[1].vel_cmd          = (int32_t)s.vel_cmd;
                telem_buf[1].vel_fbk          = plant.vel_counts;
                //telem_buf[1].vel_fbk          = encoder.velocity;;
                telem_buf[1].timestamp_ms     = tick_ms;
                telem_buf[1].drive_state      = drive_get_state();
                telem_buf[1].fault_flags      = drive.fault_flags;
                telem_buf[1].samples_consumed = drive.samples_consumed;
                telem_buf[1].iq_cmd           = (int16_t)(iq_cmd * 1000.0f);
                telem_buf[1].i_q_fbk          = (int16_t)(plant.i_q * 1000.0f);
                telem_buf[1].v_q_cmd          = (int16_t)(v_q_cmd * 1000.0f); // mV
            }
        }
    }
}


int main(void)
{
    /* Enable FPU coprocessor */
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    /* Initialize subsystems */
    clock_init();
    tim1_init();
    encoder_init();
    drive_init();
    spi_init();
    ring_init();

    // -------------------------------------------------------------------------
    // DRV8353 SPI/register bring-up test
    //
    // Purpose:
    //   Verify STM32 SPI1 can communicate with DRV8353.
    //   Verify DRV status/fault registers can be read.
    //   Verify faults can be cleared.
    //   Verify one writable config register can be written/read/restored.
    //
    // Verified working configuration (June 2026 bring-up):
    //   - nSCS on PC8 (CN10 pin 2)
    //   - 4.7k external pull-up on MISO/SDO (open-drain) to 3.3 V
    //   - SPI1 at /256 (~328 kHz) for breadboard harness
    //   - LOW speed GPIO edges on SCK/MOSI/CS (high-speed edges rang on
    //     the breadboard; DRV counted phantom clocks and discarded writes)
    //
    // Safety:
    //   PWM is not wired yet.
    //   TIM1/PWM bring-up is intentionally not part of this test.
    // -------------------------------------------------------------------------

    drv8353_init();      // SPI1 + DRV GPIO + DRV awake for register access
    volatile bool cfg_ok = drv8353_configure();   // expect true
    (void)cfg_ok;

    /* Configure SysTick for 1 kHz */
    SysTick_Config(SystemCoreClock / 1000u);

    while (1)
    {
        int32_t pos = encoder_get_position();
        int32_t vel = encoder_get_velocity();
        (void)pos;
        (void)vel;
    }
}