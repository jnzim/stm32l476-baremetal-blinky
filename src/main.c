#include "stm32f4xx.h"
#include "spi.h"
#include "ringBuffer.h"
#include "protocol.h"
#include "drive.h"
#include "loops.h"
#include "clock.h"
#include <stddef.h>
#include <stdint.h>

#define READY_PIN      13u
#define READY_SET_HIGH (1u << READY_PIN)
#define READY_CLR_LOW  (1u << (READY_PIN + 16u))

#define RING_REFILL_THRESHOLD 2048u

extern volatile uint32_t cnt_data;
extern volatile uint32_t cnt_block_hdr;
extern volatile uint32_t cnt_error;
extern volatile uint32_t cnt_cs;
extern volatile uint8_t  ready_asserted;

volatile uint32_t tick_ms          = 0;
volatile uint32_t debug_ring_count = 0;

static int32_t last_pos_cmd = 0;
static int32_t last_vel_cmd = 0;

void SysTick_Handler(void)
{
    tick_ms++;

    // drive_sm_run();

    // if (drive_is_servo_on())
    // {
    //     TrajSample s;
    //     if (ring_pop(&s))
    //     {
    //         samples_consumed++;
    //         first_sample_ready = 1;
    //         last_pos_cmd = s.pos_cmd;
    //         last_vel_cmd = s.vel_cmd;
    //     }
    // }

    // uint32_t cnt = ring_count();
    // debug_ring_count = cnt;

    // /*
    //  * READY assertion (Design A2, edge-triggered):
    //  *   - When ring drains to ≤ THRESHOLD AND not already asserted,
    //  *     drop PC13 low to request a refill.
    //  *   - SPI RX DMA ISR clears the flag and raises PC13 on the first
    //  *     DATA frame received after assertion.
    //  */
    // if (first_sample_ready && !ready_asserted && cnt <= RING_REFILL_THRESHOLD)
    // {
    //     GPIOC->BSRR    = READY_CLR_LOW;
    //     ready_asserted = 1;
    // }

    // // ── Telemetry frame ───────────────────────────────────────────────────
    // // For initial bring-up, debug counters are surfaced through the
    // // (now int32) vel fields so the Pi can see what's going on.
    // // TelemetryFrame tf;
    // // tf.pos_cmd          = last_pos_cmd;
    // // tf.pos_fbk          = (int32_t)cnt_error;       // debug: CRC/framing errors
    // // tf.vel_cmd          = (int32_t)cnt_data;        // debug: RX DATA count
    // // tf.vel_fbk          = (int32_t)cnt_block_hdr;   // debug: RX BLOCK_HDR count
    // // tf.timestamp_ms     = tick_ms;
    // // tf.drive_state      = (uint8_t)drive_get_state();
    // // tf.fault_flags      = 0;
    // // tf.samples_consumed = samples_consumed;
    // // tf.pos_err          = 0;
    // // tf.i_q_fbk          = 0;
    // // tf._pad[0]          = 0;
    // // tf._pad[1]          = 0;

    // TelemetryFrame tf;
    // uint8_t *p = (uint8_t *)&tf;
    // for (int i = 0; i < 32; i++) {
    //     p[i] = (uint8_t)(i + 1);
    // }



    // spi_update_telem(&tf);
}

int main(void)
{
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // DEBUG: preload pattern so DMA has something to clock out
    extern volatile TelemetryFrame telem_buf[2];
    //uint8_t *p = (uint8_t *)&telem_buf[0];
    uint8_t *p = (uint8_t *)&telem_buf[1];
    for (int i = 0; i < 32; i++) p[i] = (uint8_t)(i + 1);

    clock_init();
    drive_init();
    spi_init();

    SysTick_Config(SystemCoreClock / 1000u);

    while (1)
    {
    }
}