#include "spi.h"
#include "protocol.h"
#include "stm32f4xx.h"
#include <string.h>

volatile uint32_t cnt_data  = 0;
volatile uint32_t cnt_error = 0;
volatile uint8_t  dbg_rx0   = 0;
volatile uint32_t cnt_cs    = 0;

static uint8_t tx_buf[32];

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI — AF5
    GPIOB->MODER  &= ~((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));
    GPIOB->MODER  |=  ((2u<<24)|(2u<<26)|(2u<<28)|(2u<<30));
    GPIOB->AFR[1] &= ~((0xFu<<16)|(0xFu<<20)|(0xFu<<24)|(0xFu<<28));
    GPIOB->AFR[1] |=  ((5u<<16)|(5u<<20)|(5u<<24)|(5u<<28));
    GPIOB->OSPEEDR|=  ((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));

    // EXTI12 — PB12 falling edge
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (1u   << 0);
    EXTI->FTSR |=  (1u << 12);
    EXTI->RTSR &= ~(1u << 12);
    EXTI->IMR  |=  (1u << 12);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // SPI2
    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_CR2_TXDMAEN;

    // DMA1 Stream4 Ch0: SPI2_TX
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4  | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)tx_buf;
    DMA1_Stream4->NDTR = 32;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        (1u << DMA_SxCR_DIR_Pos)   |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC;
    DMA1_Stream4->CR |= DMA_SxCR_EN;

    memset(tx_buf, 0xAB, 32);

    SPI2->CR1 = SPI_CR1_SPE;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 12)) {
        EXTI->PR = (1u << 12);
        // Rearm TX DMA to byte 0
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream4->CR & DMA_SxCR_EN);
        DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                      DMA_HIFCR_CTEIF4  | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
        DMA1_Stream4->M0AR = (uint32_t)tx_buf;
        DMA1_Stream4->NDTR = 32;
        DMA1_Stream4->CR  |= DMA_SxCR_EN;
        cnt_cs++;
    }
}

void spi_set_tx(const uint8_t *data)
{
    memcpy(tx_buf, data, 32);
}

void spi_transfer(uint8_t *rx)
{
    for (int i = 0; i < 32; i++) {
        while (!(SPI2->SR & SPI_SR_RXNE));
        rx[i] = *(volatile uint8_t *)&SPI2->DR;
    }
    dbg_rx0 = rx[0];
}