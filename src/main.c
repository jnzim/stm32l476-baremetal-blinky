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
extern volatile uint8_t  ready_asserted;
extern volatile TelemetryFrame telem_buf[2];

volatile uint32_t tick_ms          = 0;
volatile uint32_t debug_ring_count = 0;

static int32_t last_pos_cmd = 0;
static int32_t last_vel_cmd = 0;

void SysTick_Handler(void)
{
    tick_ms++;
    // Pattern test mode — SysTick does not touch telem.
    // telem_buf[1] is preloaded in main() with 01..20.
}

int main(void)
{
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // ── DEBUG: blink LD2 (PA5) 5x to prove fresh firmware is running ─────────
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << 10);
    GPIOA->MODER |=  (1u << 10);
    for (int i = 0; i < 5; i++) {
        GPIOA->BSRR = (1u << 5);
        for (volatile int d = 0; d < 200000; d++);
        GPIOA->BSRR = (1u << 21);
        for (volatile int d = 0; d < 200000; d++);
    }

    // ── DEBUG: preload telem_buf[1] with pattern 01..20 ──────────────────────
    uint8_t *p = (uint8_t *)&telem_buf[1];
    for (int i = 0; i < 32; i++) p[i] = (uint8_t)(i + 1);

    clock_init();
    drive_init();
    spi_init();

    SysTick_Config(SystemCoreClock / 1000u);

    while (1) {}
}