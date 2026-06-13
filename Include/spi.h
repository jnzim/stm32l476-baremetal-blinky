#pragma once
#include <stdint.h>
#include "protocol.h"

// #define READY_RING_REFILL   13u
// #define SPI2_NSS_PIN        12u
// #define SPI2_SCK_PIN        13u
// #define SPI2_MISO_PIN       14u
// #define SPI2_MOSI_PIN       15u


void spi_init(void);
void spi_set_tx(const uint8_t *data);
void spi_update_telem(const TelemetryFrame *frame);
void spi2_dma_arm_test_tx(void);

extern volatile     TelemetryFrame telem_buf[2];
extern volatile     uint8_t        telem_write_idx;
extern volatile     uint16_t       expected_samples;