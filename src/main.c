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
//
// Owns (writes):  tick_ms, samples_consumed, first_sample_ready, telem_buf[]
// Reads:          ring (via ring_pop), drive state (via drive_is_*)
// Preempted by:   TIM1 (current loop, NVIC priority 1)
//                 DMA1_Stream3 (SPI RX, NVIC priority 2)
//                 EXTI15_10 (CS rising edge, NVIC priority 0)
//
// Run order (see SW_DESIGN.md for rationale):
//   1. tick_ms++
//   2. drive_sm_run()    — state transitions; must run before branching on state
//   3. motion pop        — pop next trajectory sample if servo on
//   4. ready_pin update  — READY signal to Pi based on ring count
//   5. telem snapshot    — (later) capture state into telem_buf
// ─────────────────────────────────────────────────────────────────────────────
void SysTick_Handler(void)
{
    tick_ms++;
    drive_sm_run();

    if (drive_is_servo_on())
    {
        TrajSample s;
        if (ring_pop(&s))
        {
            samples_consumed++;
            first_sample_ready = 1;
        }
    }
    uint32_t cnt = ring_count();    
    debug_ring_count = cnt;
    if (first_sample_ready) {
    if (cnt <= 2048u) GPIOC->BSRR = READY_CLR_LOW;
    else              GPIOC->BSRR = READY_SET_HIGH;
    }
}

int main(void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    clock_init();
    drive_init();

    // READY pin — PC13 output, low at boot
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));
    GPIOC->OTYPER &= ~(1u << READY_PIN);
    GPIOC->BSRR    =  READY_CLR_LOW;   // not ready yet

    spi_init();

    GPIOC->BSRR = READY_SET_HIGH;      // ready — Pi can send

    SysTick_Config(100000);

    while (1) {}
}