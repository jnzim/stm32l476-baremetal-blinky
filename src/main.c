#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "clock.h"
#include "spi.h"
#include "tim1.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "protocol.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"
#include "foc_sysid.h"
#include "foc_trajectory.h"

volatile uint32_t tick_ms          = 0;
volatile bool     system_initialized = false;

void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM1->SR = ~TIM_SR_UIF;

    if (!system_initialized)
        return;

    encoder_update(tick_ms);

#if RUN_MODE == RUN_MODE_SYSID
    foc_sysid_step();
#else
    foc_trajectory_step();
#endif
}

int main(void)
{
    system_initialized = false;
    
    // Enable full access to FPU coprocessors CP10 and CP11. */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));
    

    clock_init();
    encoder_init();
    drive_init();
    spi_init();
    ring_init();

    GPIOC->MODER &= ~(3u << (3u * 2u));
    GPIOC->PUPDR &= ~(3u << (3u * 2u));
    GPIOC->PUPDR |=  (1u << (3u * 2u));

    while (!(GPIOC->IDR & (1u << 3u))) {}
    while (GPIOC->IDR & (1u << 3u)) {}

    drv8353_init();
    drv8353_configure();

#if RUN_MODE == RUN_MODE_SYSID
    volatile uint16_t csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
    csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
    (void)csa_ctrl;
#endif

    pwm_init();
    current_feedback_init();
    drv_enable_high();
    current_feedback_calibrate();
    pwm_enable();

    SysTick_Config(SystemCoreClock / 1000u);

    system_initialized = true;

    while (1)
    {
        __WFI();
    }
}