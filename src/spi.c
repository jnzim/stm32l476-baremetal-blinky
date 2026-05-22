#include "spi.h"
#include "stm32f4xx.h"
#include <string.h>

static uint8_t tx_buf[32];

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI — AF5
    GPIOB->MODER  &= ~((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));
    GPIOB->MODER  |=  ((2u<<24)|(2u<<26)|(2u<<28)|(2u<<30));
    GPIOB->AFR[1] &= ~((0xFu<<16)|(0xFu<<20)|(0xFu<<24)|(0xFu<<28));
    GPIOB->AFR[1] |=  ((5u<<16)|(5u<<20)|(5u<<24)|(5u<<28));
    GPIOB->OSPEEDR|=  ((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));

    SPI2->CR1 = 0;
    SPI2->CR2 = 0;
    SPI2->CR1 = SPI_CR1_SPE;
}

void spi_set_tx(const uint8_t *data)
{
    memcpy(tx_buf, data, 32);
}

void spi_transfer(uint8_t *rx)
{
    // Preload first TX byte
    while (!(SPI2->SR & SPI_SR_TXE));
    *(volatile uint8_t *)&SPI2->DR = tx_buf[0];

    for (int i = 0; i < 32; i++) {
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx[i] = *(volatile uint8_t *)&SPI2->DR;
        if (i + 1 < 32) {
            while (!(SPI2->SR & SPI_SR_TXE));
            *(volatile uint8_t *)&SPI2->DR = tx_buf[i + 1];
        }
    }
}