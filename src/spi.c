#include "spi.h"
#include "protocol.h"
#include "stm32f4xx.h"
#include <string.h>

volatile uint32_t cnt_data  = 0;
volatile uint32_t cnt_error = 0;
volatile uint32_t cnt_isr   = 0;
volatile uint8_t  dbg_rx0   = 0;

static uint8_t rx_buf[32];
static uint8_t tx_buf[32];

static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    GPIOB->MODER  &= ~((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));
    GPIOB->MODER  |=  ((2u<<24)|(2u<<26)|(2u<<28)|(2u<<30));
    GPIOB->AFR[1] &= ~((0xFu<<16)|(0xFu<<20)|(0xFu<<24)|(0xFu<<28));
    GPIOB->AFR[1] |=  ((5u<<16)|(5u<<20)|(5u<<24)|(5u<<28));
    GPIOB->OSPEEDR|=  ((3u<<24)|(3u<<26)|(3u<<28)|(3u<<30));

    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_CR2_RXDMAEN;

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3  | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)rx_buf;
    DMA1_Stream3->NDTR = 32;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        (0u << DMA_SxCR_DIR_Pos)   |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;
    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    memset(tx_buf, 0xAB, 32);

    SPI2->CR1 = SPI_CR1_SPE;
}

void spi_set_tx(const uint8_t *data)
{
    memcpy(tx_buf, data, 32);
}

void DMA1_Stream3_IRQHandler(void)
{
    cnt_isr++;
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    uint8_t local[32];
    memcpy(local, rx_buf, 32);

    dbg_rx0 = local[0];

    switch (local[0]) {
        case SPI2_OP_DATA:
            if (crc8_xor(local, 9) != local[9]) { cnt_error++; break; }
            cnt_data++;
            break;
        case SPI2_OP_BLOCK_HDR:
            if (crc8_xor(local, 3) != local[3]) { cnt_error++; break; }
            break;
        case SPI2_OP_TELEM_REQ:  break;
        case SPI2_OP_READY_ACK:  break;
        case SPI2_OP_STOP:
            if (crc8_xor(local, 1) != local[1]) { cnt_error++; break; }
            break;
        default:
            cnt_error++;
            break;
    }
}