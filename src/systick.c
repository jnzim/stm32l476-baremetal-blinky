#include "systick.h"
#include "stm32f0_regs.h"

static volatile uint32_t g_ms = 0;

void systick_init_reload(uint32_t reload)
{
    g_ms = 0;

    SYST_CSR = 0;
    SYST_RVR = reload;
    SYST_CVR = 0;

    SYST_CSR = SYST_CSR_CLKSOURCE |
               SYST_CSR_TICKINT   |
               SYST_CSR_ENABLE;
}

uint32_t millis(void)
{
    return g_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) { }
}

void SysTick_Handler(void)
{
    g_ms++;
}
