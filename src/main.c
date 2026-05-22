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

void SysTick_Handler(void)
{
    tick_ms++;
    drive_update();

    if (drive_get_state() == STATE_SERVO_ON)
    {
        TrajSample s;
        if (ring_pop(&s))
        {
            telem_buf[1].pos_cmd          = s.pos_cmd;
            telem_buf[1].vel_cmd          = (int16_t)s.vel_cmd;
            telem_buf[1].timestamp_ms     = tick_ms;
            telem_buf[1].samples_consumed = ++samples_consumed;
            first_sample_ready            = 1;
        }
    }

    debug_ring_count = ring.count;

    if (ring.count <= 2048)
        GPIOC->BSRR = READY_CLR_LOW;
    else
        GPIOC->BSRR = READY_SET_HIGH;
}

int main(void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    clock_init();
    drive_init();

    // READY pin — PC13 output
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));
    GPIOC->OTYPER &= ~(1u << READY_PIN);
    GPIOC->BSRR    =  READY_SET_HIGH;

    spi_init();

    SysTick_Config(100000);  // 100MHz / 100000 = 1kHz

    while (1) {}
}