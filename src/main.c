#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"
#include "plant.h"
#include "control.h"
#include "protocol.h"
#include "drive.h"
#include "loops.h"
#include "clock.h"
#include "pwm.h"

// ── READY signal — PC13, active high → Pi refills ring buffer ────────────────
#define READY_PIN       13u
#define READY_SET_HIGH  (1u << READY_PIN)
#define READY_CLR_LOW   (1u << (READY_PIN + 16u))

// ── ENABLE — PB8, active high → DRV8353RS exits sleep mode ──────────────────
#define ENABLE_PIN      8u

// ── nFAULT — PC6, active low → DRV8353RS asserts on any fault condition ──────
#define NFAULT_PIN      6u

// ── Control loop sample periods ───────────────────────────────────────────────
#define DT_CURRENT      (1.0f / 20000.0f)
#define DT_VELOCITY     (1.0f / 5000.0f)
#define DT_POSITION     (1.0f / 1000.0f)

// ── Sim mode plant ────────────────────────────────────────────────────────────
PlantState plant;

// ── Diagnostics ───────────────────────────────────────────────────────────────
volatile uint32_t debug_ring_count = 0;
volatile uint32_t tick_ms          = 0;

// ─────────────────────────────────────────────────────────────────────────────
// EXTI9_5_IRQHandler — nFAULT falling edge on PC6
//
// DRV8353RS pulls nFAULT low on any fault (OCP, OVP, UVP, OTW, GDF).
// EXTI6 fires on falling edge -> set fault_req flag.
// State machine picks it up next SysTick tick → STATE_FAULT, PWM disabled.
// PC6 shares the EXTI9_5 vector with pins 5-9 — check PR bit before acting.
// ─────────────────────────────────────────────────────────────────────────────
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1u << NFAULT_PIN))
    {
        EXTI->PR = (1u << NFAULT_PIN);     // clear pending flag
        drive_request_fault();              // set fault_req — state machine acts next tick
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SysTick_Handler — 1kHz position loop + state machine
// ─────────────────────────────────────────────────────────────────────────────
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
            telem_buf[1].pos_fbk          = plant.pos_counts;
            telem_buf[1].vel_fbk          = (int16_t)plant.vel_counts;
            telem_buf[1].i_q_fbk          = (int16_t)(plant.i_q * 1000.0f);
            telem_buf[1].v_q_cmd          = v_q_cmd;
            float pos_err                 = (float)(s.pos_cmd - plant.pos_counts);
            telem_buf[1].pos_err          = (int16_t)pos_err;
            vel_cmd = p_step(&position_loop, pos_err) / COUNTS_PER_RAD;
            first_sample_ready            = 1;
        }
    }

    debug_ring_count = ring.count;

    // READY signal — Pi monitors this to know when to refill ring buffer
    if (ring.count <= 2048)
        GPIOC->BSRR = READY_CLR_LOW;
    else
        GPIOC->BSRR = READY_SET_HIGH;
}

// ─────────────────────────────────────────────────────────────────────────────
// TIM1_UP_TIM10_IRQHandler — 20kHz velocity + current loops
// ─────────────────────────────────────────────────────────────────────────────
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = 0;
    if (drive_get_state() != STATE_SERVO_ON) return;
    if (!first_sample_ready) return;

    // ── Velocity loop — runs every 4th tick (5kHz) ───────────────────────────
    if (++vel_div >= 4)
    {
        vel_div = 0;
        float v_err = vel_cmd - plant.vel;
        iq_cmd = pi_step(&velocity_loop, v_err, DT_VELOCITY);
    }

    // ── Current loop — runs every tick (20kHz) ───────────────────────────────
    float i_err = iq_cmd - plant.i_q;
    v_q_cmd = pi_step(&current_loop, i_err, DT_CURRENT);

    // ── Plant step — sim only, replaced by pwm_apply_vq() on hardware ────────
    plant_step(&plant, v_q_cmd, DT_CURRENT);
}

// ─────────────────────────────────────────────────────────────────────────────
// main — runs once at startup, then sleeps forever
// All real work is interrupt-driven: TIM1 ISR, SysTick, DMA ISR
// ─────────────────────────────────────────────────────────────────────────────
int main(void)
{
    // ── Enable FPU — required before any float instructions ──────────────────
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    // ── System clock — HSI → PLL → 180MHz ────────────────────────────────────
    clock_init();

    // ── Telemetry sentinel values — Pi detects stale frame if these appear ───
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx           = 0;

    // ── GPIO clocks — enable GPIOB (ENABLE) and GPIOC (READY, nFAULT) ───────
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;                         // flush write pipeline

    // ── READY FOR PROFILE — PC13 output, deasserted high at startup ──────────────────────
    // Pi polls this pin to know when ring buffer needs refilling
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));  // output
    GPIOC->OTYPER &= ~(1u << READY_PIN);         // push-pull
    GPIOC->BSRR    =  READY_SET_HIGH;            // deassert — ring not ready yet

    // ── ENABLE DRV — PB8 output, driven high immediately ─────────────────────────
    // DRV8353RS stays in sleep mode until ENABLE is high.
    // Must be high before SPI1 communication /PWM to drive output.
    GPIOB->MODER  &= ~(3u << (ENABLE_PIN * 2));
    GPIOB->MODER  |=  (1u << (ENABLE_PIN * 2));  // output
    GPIOB->OTYPER &= ~(1u << ENABLE_PIN);         // push-pull
    GPIOB->BSRR    =  (1u << ENABLE_PIN);         // drive high — DRV wakes up

    // ── nFAULT — PC6 input with pull-up, EXTI6 falling edge interrupt ────────
    // DRV8353RS open-drain output, pulled low on fault.
    // EVM has onboard pullup but STM internal pullup added for safety.
    GPIOC->MODER  &= ~(3u << (NFAULT_PIN * 2));  // input mode
    GPIOC->PUPDR  &= ~(3u << (NFAULT_PIN * 2));
    GPIOC->PUPDR  |=  (1u << (NFAULT_PIN * 2));  // internal pull-up

    // Route EXTI6 to PORTC via SYSCFG
    // Mux that connects GPIO pins to EXTI interrupt lines
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->EXTICR[1] &= ~(0xFu << 8);           // clear EXTI6 source [11:8]
    SYSCFG->EXTICR[1] |=  (0x2u << 8);           // 0x2 = PORTC → PC6

    // Configure EXTI6: falling edge only, unmask, clear any pending
    EXTI->FTSR |=  (1u << NFAULT_PIN);            // trigger on falling edge
    EXTI->RTSR &= ~(1u << NFAULT_PIN);            // not rising edge
    EXTI->IMR  |=  (1u << NFAULT_PIN);            // unmask interrupt
    EXTI->PR    =  (1u << NFAULT_PIN);            // clear any stale pending

    NVIC_SetPriority(EXTI9_5_IRQn, 3);            // lower priority than TIM1/DMA
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // ── Enable DMA1 clock before spi_init() ──────────────────────────────────────
    // Data path: SPI2 DR → DMA1 Stream3 → spi2_rx_buf → DMA ISR → ring buffer → SysTick
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    spi_init();                                    // SPI2 slave + DMA ring fill
    encoder_init();                                // TIM5 quadrature decoder
    pwm_init();                                    // TIM1 20kHz PWM, MOE=0

    // ── Start scheduler — 1kHz SysTick ───────────────────────────────────────
    SysTick_Config(100000);                        // 100MHz 

    while (1) {}                                   // all work is interrupt-driven
}