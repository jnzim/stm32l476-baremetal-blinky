#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"
#include "plant.h"
#include "control.h"
#include "protocol.h"

// ── READY signal — PC13, active low → Pi refills ring buffer ─────────────────
#define READY_PIN       13u
#define READY_SET_HIGH  (1u << READY_PIN)
#define READY_CLR_LOW   (1u << (READY_PIN + 16u))

// ── Control loop sample periods ───────────────────────────────────────────────
#define DT_CURRENT      (1.0f / 20000.0f)   // 20 kHz — TIM1 ISR
#define DT_VELOCITY     (1.0f / 5000.0f)    // 5 kHz  — rate divided ÷4 in TIM1 ISR
#define DT_POSITION     (1.0f / 1000.0f)    // 1 kHz  — SysTick

// ── Sim mode plant and controller state ──────────────────────────────────────
// In sim mode these replace ADC/encoder hardware feedback.
// plant.pos and plant.vel are the simulated motor position and velocity.
static PlantState plant;
static PIState    current_loop;
static PIState    velocity_loop;
static PState     position_loop;

// ── Inter-loop setpoints (written by outer loop, read by inner loop) ──────────
static float     v_q_cmd  = 0.0f;   // voltage command → plant (from current loop)
static float     iq_cmd   = 0.0f;   // current setpoint (from velocity loop)
static float     vel_cmd  = 0.0f;   // velocity setpoint (from position loop)
static uint32_t  vel_div  = 0;      // velocity loop rate divider counter

// ── Diagnostics ───────────────────────────────────────────────────────────────
volatile uint32_t debug_ring_count = 0;
volatile uint32_t tick_ms          = 0;

// ─────────────────────────────────────────────────────────────────────────────
// SysTick_Handler — 1 kHz
//
// Consumes one trajectory sample per tick from the ring buffer.
// Runs position loop — slowest outer loop.
// Updates READY signal to Pi for ring buffer refill.
// ─────────────────────────────────────────────────────────────────────────────
void SysTick_Handler(void)
{
    tick_ms++;

    TrajSample s;
    if (ring_pop(&s)) {
        telem_buf[1].pos_cmd          = s.pos_cmd;
        telem_buf[1].vel_cmd          = (int16_t)s.vel_cmd;
        telem_buf[1].timestamp_ms     = tick_ms;
        telem_buf[1].samples_consumed = ++samples_consumed;


        telem_buf[1].pos_fbk = plant.pos_counts;
        telem_buf[1].vel_fbk = (int16_t)plant.vel_counts;

        float pos_err = (float)(s.pos_cmd - plant.pos_counts);
        vel_cmd = p_step(&position_loop, pos_err);
    }

    debug_ring_count = ring.count;

    // PC13 READY — active low, Pi refills when asserted
    if (ring.count <= 2048) {
        GPIOC->BSRR = READY_CLR_LOW;   // assert low — refill needed
    } else {
        GPIOC->BSRR = READY_SET_HIGH;  // deassert high — buffer ok
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TIM1_UP_TIM10_IRQHandler — 20 kHz
//
// Highest priority motion ISR. Runs current loop every tick.
// Velocity loop runs at 5 kHz (every 4th tick).
// Plant model is stepped here — replaces ADC/encoder reads on real hardware.
// ─────────────────────────────────────────────────────────────────────────────
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = 0;   // clear update interrupt flag
    if (!sim_active) return;  
    // current loop — 20 kHz
    // error = commanded current - simulated current
    // output = voltage applied to motor (v_q)
    float i_err = iq_cmd - plant.i_q;
    v_q_cmd = pi_step(&current_loop, i_err, DT_CURRENT);

    // step plant with new voltage command — updates vel, pos, i_q
    plant_step(&plant, v_q_cmd, DT_CURRENT);

    // velocity loop — 5 kHz (÷4)
    // error = commanded velocity - simulated velocity
    // output = current setpoint (iq_cmd)
    if (++vel_div >= 4) {
        vel_div = 0;

        // velocity loop — compare in counts/sec
        float v_err = (float)vel_cmd - (float)plant.vel_counts;
        iq_cmd = pi_step(&velocity_loop, v_err, DT_VELOCITY);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// tim1_init
//
// TIM1 on APB2 = 180 MHz. Center-aligned mode — update event fires at
// both peak and valley of the counter, giving 20 kHz from ARR = 4499.
// Update interrupt drives the current and velocity control loops.
// ─────────────────────────────────────────────────────────────────────────────
static void tim1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;   // ensure clock is enabled before register access

    TIM1->CR1  = TIM_CR1_CMS_0;   // center-aligned mode 1
    TIM1->PSC  = 0;                // no prescaler — 180 MHz input
    TIM1->ARR  = 4499;             // 180MHz / (2 * 4500) = 20 kHz
    TIM1->DIER = TIM_DIER_UIE;    // update interrupt enable
    TIM1->EGR  = TIM_EGR_UG;      // force update event to load ARR/PSC
    TIM1->SR   = 0;                // clear any pending flags before enabling NVIC

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);   // higher priority than SysTick (15)
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;     // start counter
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(void)
{
    // ── Telemetry init — known pattern so Pi can detect uninitialized frames ──
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx           = 0;

    // ── DMA clock — must be enabled before spi_init ───────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    // ── PC13 READY output — push-pull, deassert high at startup ──────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->MODER  &= ~(3u << (READY_PIN * 2));
    GPIOC->MODER  |=  (1u << (READY_PIN * 2));   // output mode
    GPIOC->OTYPER &= ~(1u << READY_PIN);           // push-pull
    GPIOC->BSRR    =  READY_SET_HIGH;              // deassert at startup

    // ── Plant and controller init ─────────────────────────────────────────────
    // Gains are placeholder — tune after sim validation with matplotlib.
    plant_init(&plant);
pi_init(&current_loop,  5.0f,   0.0f,  -24.0f, 24.0f);  // Ki=0
pi_init(&velocity_loop, 0.005f, 0.0f,   -3.0f,  3.0f);  // Ki=0
p_init(&position_loop,  1.0f);

    // ── Peripheral init ───────────────────────────────────────────────────────
    spi_init();
    encoder_init();
    tim1_init();
    SysTick_Config(180000);   // 180MHz / 180000 = 1 kHz

    while (1) {}
}