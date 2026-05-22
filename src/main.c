#include "spi.h"
#include "protocol.h"
#include "stm32f4xx.h"
#include <string.h>
#include "clock.h"
#include "drive.h"

extern volatile uint32_t cnt_data;
extern volatile uint32_t cnt_error;
extern volatile uint8_t  dbg_rx0;

int main(void)
{
    clock_init();
    drive_init();
    spi_init();

    while (1) {}
}