#include "spi.h"
#include "protocol.h"
#include "stm32f4xx.h"
#include <string.h>

volatile uint8_t  dbg_rx0   = 0;
volatile uint32_t cnt_data  = 0;
volatile uint32_t cnt_error = 0;

static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

static void spi_process(uint8_t *rx)
{
    switch (rx[0]) {
        case SPI2_OP_DATA:
            if (crc8_xor(rx, 9) != rx[9]) { cnt_error++; break; }
            cnt_data++;
            break;
        case SPI2_OP_BLOCK_HDR:
            if (crc8_xor(rx, 3) != rx[3]) { cnt_error++; break; }
            break;
        case SPI2_OP_TELEM_REQ:  break;
        case SPI2_OP_READY_ACK:  break;
        case SPI2_OP_STOP:
            if (crc8_xor(rx, 1) != rx[1]) { cnt_error++; break; }
            break;
        default:
            cnt_error++;
            break;
    }
}

int main(void)
{
    spi_init();

    uint8_t rx[32] = {0};
    uint8_t tx[32];
    memset(tx, 0xAB, 32);
    spi_set_tx(tx);

    while (1) {
        spi_transfer(rx);
        dbg_rx0 = rx[0];
        spi_process(rx);
        spi_set_tx(tx);
    }
}