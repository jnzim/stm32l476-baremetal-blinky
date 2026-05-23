#include "stm32f4xx.h"
#include "spi.h"
#include "ringBuffer.h"
#include "protocol.h"
#include "drive.h"
#include "loops.h"
#include "clock.h"

#define READY_PIN      13u
#define READY_SET_HIGH (1u << READY_PIN)
#define READY_CLR_LOW  (1u << (READY_PIN + 16u))

volatile uint32_t tick_ms          = 0;
volatile uint32_t debug_ring_count = 0;

// ─────────────────────────────────────────────────────────────────────────────
// SysTick_Handler — 1 kHz, NVIC priority 15
// ─────────────────────────────────────────────────────────────────────────────
void SysTick_Handler(void)
{
    tick_ms++;
    drive_sm_run();

    static TrajSample last_sample = { 0, 0 };

    if (drive_is_servo_on())
    {
        TrajSample s;
        if (ring_pop(&s))
        {
            samples_consumed++;
            first_sample_ready = 1;
            last_sample        = s;
        }
    }

    uint32_t cnt = ring_count();
    debug_ring_count = cnt;
    if (first_sample_ready) {
        if (cnt <= 2048u) GPIOC->BSRR = READY_CLR_LOW;
        else              GPIOC->BSRR = READY_SET_HIGH;
    }

    /* ── Pack and publish telemetry — 32 bytes returned on next MISO ── */
    TelemetryFrame tf;
    tf.pos_cmd          = last_sample.pos_cmd;
    tf.pos_fbk          = 0;                              /* no plant yet */
    tf.vel_cmd          = (int16_t)last_sample.vel_cmd;   /* truncate; OK until plant integration */
    tf.vel_fbk          = 0;
    tf.timestamp_ms     = tick_ms;
    tf.drive_state      = (uint8_t)drive_get_state();
    tf.fault_flags      = 0;
    tf.samples_consumed = samples_consumed;
    tf.pos_err          = 0;
    tf.i_q_fbk          = 0;
    tf.v_q_cmd          = 0.0f;
    tf._pad[0]          = 0;
    tf._pad[1]          = 0;
    spi_update_telem(&tf);
}

int main(void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    clock_init();
    drive_init();

    /* READY pin — PC13 output, low at boot */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));
    GPIOC->OTYPER &= ~(1u << READY_PIN);
    GPIOC->BSRR    =  READY_CLR_LOW;

    spi_init();
    GPIOC->BSRR = READY_SET_HIGH;
    SysTick_Config(100000);
    while (1) {}
}