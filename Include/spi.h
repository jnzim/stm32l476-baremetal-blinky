#pragma once
#include <stdint.h>

void spi_init(void);
void spi_set_tx(const uint8_t *data);