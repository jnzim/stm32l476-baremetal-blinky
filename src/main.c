#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"

volatile uint32_t debug_ring_count = 0;
volatile uint32_t tick_ms = 0;

void SysTick_Handler(void) {
    tick_ms++;

    TrajSample s;
    if (ring_pop(&s)) {
        telem_buf[1].pos_cmd = (uint32_t)s.pos_cmd;
        telem_buf[1].vel_cmd  = (uint32_t)s.vel_cmd;
    }
}

int main(void) {
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    spi_init();
    encoder_init();

    // 1ms SysTick — 180MHz / 1000 = 180000 cycles
    SysTick_Config(180000);

  
    while (1) {}
}