#include "stm32f4xx.h"
#include "spi.h"
#include "ringBuffer.h"
#include "protocol.h"
#include "drive.h"
#include "loops.h"
#include "clock.h"
#include "tim1.h"
#include "plant.h"

#include <stdint.h>

#define READY_PIN      13u
#define READY_SET_HIGH (1u << READY_PIN)
#define READY_CLR_LOW  (1u << (READY_PIN + 16u))

#define RING_REFILL_THRESHOLD 2048u

extern volatile uint32_t    cnt_data;
extern volatile uint32_t    cnt_block_hdr;
extern volatile uint32_t    cnt_error;
extern volatile uint8_t     ready_asserted;


volatile uint32_t tick_ms          = 0;
volatile uint32_t cnt_systick      = 0;
volatile uint32_t debug_ring_count = 0;

volatile uint32_t cnt_ring_pop     = 0;
volatile int32_t  debug_pop_pos    = 0;
volatile int32_t  debug_pop_vel    = 0;

volatile int32_t last_pos_cmd = 0;
volatile int32_t last_vel_cmd = 0;

volatile uint32_t cnt_tim1 = 0;

void SysTick_Handler(void)
{
    cnt_systick++;
    tick_ms++;

    drive_sm_run();

    debug_ring_count = ring_count();

    if (debug_ring_count < RING_REFILL_THRESHOLD && !ready_asserted)
    {
        GPIOC->BSRR    = READY_CLR_LOW;
        ready_asserted = 1;
    }
}



static uint32_t   div_1k   = 0;

void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;
    cnt_tim1++;

    if (++div_1k >= 20u)
    {
        div_1k = 0;

        if (drive_is_servo_on())
        {
            TrajSample s;

            if (ring_pop(&s))
            {
                samples_consumed++;
                cnt_ring_pop++;
                first_sample_ready = 1;

                float pos_err = (float)(s.pos_cmd - plant.pos_counts);
                vel_cmd = p_step(&position_loop, pos_err);

                telem_buf[1].pos_cmd          = s.pos_cmd;
                telem_buf[1].pos_fbk          = plant.pos_counts;
                telem_buf[1].vel_cmd          = (int32_t)vel_cmd;
                telem_buf[1].timestamp_ms     = tick_ms;
                telem_buf[1].samples_consumed = samples_consumed;
                telem_buf[1].drive_state      = drive_get_state();
            }

            if (first_sample_ready)
            {
                float vel_err = vel_cmd - (float)plant.vel_counts;
                iq_cmd = pi_step(&velocity_loop, vel_err, 1.0f / 1000.0f);

                v_q_cmd = pi_step(&current_loop, iq_cmd - plant.i_q, 1.0f / 1000.0f);

                plant_step(&plant, v_q_cmd, 1.0f / 1000.0f);
            }
        }
    }
}
static void debug_blink_fresh_firmware(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3u << 10);
    GPIOA->MODER |=  (1u << 10);

    for (int i = 0; i < 5; i++)
    {
        GPIOA->BSRR = (1u << 5);
        for (volatile int d = 0; d < 200000; d++) {}

        GPIOA->BSRR = (1u << 21);
        for (volatile int d = 0; d < 200000; d++) {}
    }
}

int main(void)
{
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    debug_blink_fresh_firmware();

    clock_init();
    tim1_init();
    drive_init();
    spi_init();

    uint32_t systick_ok = SysTick_Config(SystemCoreClock / 1000u);
    (void)systick_ok;

    while (1)
    {
    }
}