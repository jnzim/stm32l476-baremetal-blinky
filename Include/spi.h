#pragma once
#include <stdint.h>
#include "protocol.h"

void spi_init(void);
void spi_set_tx(const uint8_t *data);
void spi_update_telem(const TelemetryFrame *frame);
void spi2_dma_arm_test_tx(void);

extern volatile     TelemetryFrame telem_buf[2];
extern volatile     uint8_t        telem_write_idx;
extern volatile     uint16_t       expected_samples;