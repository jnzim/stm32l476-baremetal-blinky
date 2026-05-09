#include "stm32f4xx.h"
#include "spi.h"
#include "encoder.h"
#include "ringBuffer.h"

volatile uint32_t debug_ring_count = 0;

int main(void) {
    telem_buf[1].pos_cmd      = 0x12345678;
    telem_buf[1].pos_fbk      = 0x87654321;
    telem_buf[1].timestamp_ms = 0xDEADBEEF;
    telem_buf[1].drive_state  = DRIVE_IDLE;
    telem_write_idx = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
(   void)RCC->AHB1ENR;  // force AHB bus sync before any DMA register access

    spi_init();
    encoder_init();
volatile uint32_t debug_spi_sr = 0;

while (1) {

     debug_spi_sr = SPI2->SR;
    debug_ring_count = ring.count;
}
  
}