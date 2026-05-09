#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"

int main(void) {
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx = 0;

    spi_init();
    encoder_init();

    while (1) {}
}