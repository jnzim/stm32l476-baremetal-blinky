#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"

#define READY_PIN       13u
#define READY_SET_HIGH  (1u << READY_PIN)
#define READY_CLR_LOW   (1u << (READY_PIN + 16u))

volatile uint32_t debug_ring_count  = 0;
volatile uint32_t tick_ms           = 0;


void SysTick_Handler(void) {
    tick_ms++;

    TrajSample s;
    if (ring_pop(&s)) {
        telem_buf[1].pos_cmd            = s.pos_cmd;
        telem_buf[1].vel_cmd            = (int16_t)s.vel_cmd;
        telem_buf[1].timestamp_ms       = tick_ms;
        telem_buf[1].samples_consumed   = ++samples_consumed;
    }

    debug_ring_count = ring.count;

    // PC13 READY — active low, Pi refills when asserted
    if (ring.count <= 2048) {
        GPIOC->BSRR = READY_CLR_LOW;   // assert low — refill needed
    } else {
        GPIOC->BSRR = READY_SET_HIGH;  // deassert high — buffer ok
    }
}

int main(void) {
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx           = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    // PC13 output push-pull, deassert high at startup
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));  // output
    GPIOC->OTYPER &= ~(1u << READY_PIN);          // push-pull
    GPIOC->BSRR    =  READY_SET_HIGH;             // deassert at startup

    spi_init();
    encoder_init();
    SysTick_Config(180000);

    while (1) {}
}