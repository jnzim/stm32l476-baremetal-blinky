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



/* =============================================================================
 * Timing and loop dividers
 * =============================================================================
 */
volatile uint32_t tick_ms = 0;
static uint32_t div_1k = 0;
static uint32_t div_5k = 0;

/* =============================================================================
 * SysTick_Handler — 1 kHz
 * Calls drive state machine
 * =============================================================================
 */
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
 * =============================================================================
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;
    
    if (first_sample_ready)
    {
        /* 20 kHz: current loop + plant */
        v_q_cmd = pi_step(&current_loop, iq_cmd - plant.i_q, 1.0f / 20000.0f);
        plant_step(&plant, v_q_cmd, 1.0f / 20000.0f);
        
        /* 5 kHz: velocity loop */
        if (++div_5k >= 4u)
        {
            div_5k = 0;
            float vel_err_rads = vel_cmd - ((float)plant.vel_counts / COUNTS_PER_RAD);
            iq_cmd = pi_step(&velocity_loop, vel_err_rads, 1.0f / 5000.0f);
        }
    }
    
    /* 1 kHz: position loop + trajectory pop */
    if (++div_1k >= 20u)
    {
        div_1k = 0;
        
if (drive.samples_consumed >= 400 && drive.samples_consumed < 401)
{
    __NOP();  /* breakpoint: inspect ring at sample 400 */
}

        if (drive_is_servo_on() && drive.samples_consumed < expected_samples)
        {
            TrajSlot s;
            if (ring_pop(&s))
            {
                /* Check for CRC error flag from ring_push */
                // if (ring_get_crc_error())
                // {
                //     drive.fault_flags |= FAULT_TRAJ_CRC;
                // }
                
                /* Update state */
                drive.samples_consumed++;
                first_sample_ready = 1;
                
                /* Position loop: compute velocity command */
                float pos_err = (float)(s.pos_cmd - plant.pos_counts);
                vel_cmd = ((float)s.vel_cmd + p_step(&position_loop, pos_err)) / COUNTS_PER_RAD;
                
                /* Populate telemetry frame */
                telem_buf[1].pos_cmd          = s.pos_cmd;
                telem_buf[1].pos_fbk          = plant.pos_counts;
                telem_buf[1].vel_cmd          = (int32_t)s.vel_cmd;
                telem_buf[1].vel_fbk          = plant.vel_counts;
                telem_buf[1].timestamp_ms     = tick_ms;
                telem_buf[1].drive_state      = drive_get_state();
                telem_buf[1].fault_flags      = drive.fault_flags;
                telem_buf[1].samples_consumed = drive.samples_consumed;
                telem_buf[1].iq_cmd           = (int16_t)(iq_cmd * 1000.0f);
                telem_buf[1].i_q_fbk          = (int16_t)(plant.i_q * 1000.0f);
            }
        }
        else if (drive.samples_consumed >= expected_samples)
        {
            vel_cmd = 0.0f;
            //drive_request_servo_off();
        }
    }
}

/* =============================================================================
 * main
 * =============================================================================
 */
int main(void)
{
    /* Enable FPU coprocessor */
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
    
    /* Initialize subsystems */
    clock_init();
    tim1_init();
    drive_init();
    spi_init();
    ring_init();
    
    /* Configure SysTick for 1 kHz */
    SysTick_Config(SystemCoreClock / 1000u);
    
    /* Main loop — all work happens in ISRs */
    while (1)
    {
    }
    
    return 0;
}