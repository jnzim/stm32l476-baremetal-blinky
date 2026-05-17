#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"
#include "plant.h"
#include "control.h"
#include "protocol.h"
#include "drive.h"
#include "loops.h"

// ── READY signal — PC13, active low → Pi refills ring buffer ─────────────────
#define READY_PIN       13u
#define READY_SET_HIGH  (1u << READY_PIN)
#define READY_CLR_LOW   (1u << (READY_PIN + 16u))

// ── Control loop sample periods ───────────────────────────────────────────────
#define DT_CURRENT      (1.0f / 20000.0f)
#define DT_VELOCITY     (1.0f / 5000.0f)
#define DT_POSITION     (1.0f / 1000.0f)

// ── Sim mode plant ────────────────────────────────────────────────────────────
 PlantState plant;

// ── Diagnostics ───────────────────────────────────────────────────────────────
volatile uint32_t debug_ring_count = 0;
volatile uint32_t tick_ms          = 0;

void SysTick_Handler(void)
{
    tick_ms++;
    drive_update();

    if (drive_get_state() == STATE_ENABLED)
    {
        if (drive_is_entry())
            first_sample_ready = 0;

        TrajSample s;
        if (ring_pop(&s))
        {
            first_sample_ready = 1;
            telem_buf[1].pos_cmd          = s.pos_cmd;
            telem_buf[1].vel_cmd          = (int16_t)s.vel_cmd;
            telem_buf[1].timestamp_ms     = tick_ms;
            telem_buf[1].samples_consumed = ++samples_consumed;
            telem_buf[1].pos_fbk          = plant.pos_counts;
            telem_buf[1].vel_fbk          = (int16_t)plant.vel_counts;

            float pos_err = (float)(s.pos_cmd - plant.pos_counts);
            vel_cmd = p_step(&position_loop, pos_err) / COUNTS_PER_RAD
                    + (float)s.vel_cmd / COUNTS_PER_RAD;
        }
    }

    debug_ring_count = ring.count;

    if (ring.count <= 2048)
        GPIOC->BSRR = READY_CLR_LOW;
    else
        GPIOC->BSRR = READY_SET_HIGH;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = 0;
    if (drive_get_state() != STATE_ENABLED) return;
    if (!first_sample_ready) return;

    if (++vel_div >= 4)
    {
        vel_div = 0;
        float v_err = vel_cmd - plant.vel;
        iq_cmd = pi_step(&velocity_loop, v_err, DT_VELOCITY);
    }

    float i_err = iq_cmd - plant.i_q;
    v_q_cmd = pi_step(&current_loop, i_err, DT_CURRENT);
    plant_step(&plant, v_q_cmd, DT_CURRENT);
}

static void tim1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    TIM1->CR1  = TIM_CR1_CMS_0;
    TIM1->PSC  = 0;
    TIM1->ARR  = 4499;
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->SR   = 0;
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    TIM1->CR1 |= TIM_CR1_CEN;
}

int main(void)
{
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx           = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));
    GPIOC->OTYPER &= ~(1u << READY_PIN);
    GPIOC->BSRR    =  READY_SET_HIGH;

    spi_init();
    encoder_init();
    tim1_init();
    SysTick_Config(180000);

    while (1) {}
}