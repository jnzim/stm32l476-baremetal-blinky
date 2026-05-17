// spi.c — SPI2 slave + DMA, STM32F446RE bare metal
//
// SPI2 pins:  PB12=NSS  PB13=SCK  PB14=MISO  PB15=MOSI  (AF5)
// DMA1 Stream3 Ch0 = SPI2_RX  peripheral→memory, circular, TC every 24 bytes
// DMA1 Stream4 Ch0 = SPI2_TX  memory→peripheral, circular, replays telem_buf[1]

#include "spi.h"
#include "ringBuffer.h"
#include "stm32f4xx.h"
#include <string.h>
#include "drive.h"
#include "loops.h"

#define SPI2_PKT_LEN  SPI2_TRANSACTION_BYTES   // 24 bytes

// ── Telemetry buffer — written by SysTick, read by TX DMA ────────────────────
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// ── RX buffer — circular DMA writes here continuously ────────────────────────
static volatile uint8_t spi2_rx_buf[SPI2_PKT_LEN];

// ── Diagnostic counters ───────────────────────────────────────────────────────
volatile uint32_t cnt_data  = 0;
volatile uint32_t cnt_telem = 0;
volatile uint32_t cnt_error = 0;

void spi_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // ── GPIO PB12–PB15: AF5 = SPI2 ───────────────────────────────────────────
    GPIOB->MODER &= ~((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));
    GPIOB->MODER |=  ((2u << 24) | (2u << 26) | (2u << 28) | (2u << 30));

    GPIOB->AFR[1] &= ~((0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28));
    GPIOB->AFR[1] |=  ((5u   << 16) | (5u   << 20) | (5u   << 24) | (5u   << 28));

    GPIOB->OSPEEDR |= ((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));

    // ── SPI2: slave, Mode 0, 8-bit, SSM=1 SSI=0 (CS driven by Pi) ───────────
    SPI2->CR1 = 0;
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;
    SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // ── DMA1 Stream4 Ch0: SPI2_TX — circular, replays telem_buf[1] ──────────
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4  | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[1];
    DMA1_Stream4->NDTR = SPI2_PKT_LEN;
    DMA1_Stream4->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        (1u << DMA_SxCR_DIR_Pos)   |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC;

    DMA1_Stream4->CR |= DMA_SxCR_EN;

    // ── DMA1 Stream3 Ch0: SPI2_RX — circular, TC fires every 24 bytes ────────
    DMA1_Stream3->CR = 0;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);

    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI2_PKT_LEN;
    DMA1_Stream3->CR   =
        (0u << DMA_SxCR_CHSEL_Pos) |
        (0u << DMA_SxCR_DIR_Pos)   |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    // ── NVIC ──────────────────────────────────────────────────────────────────
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;

    uint8_t local[SPI2_PKT_LEN];
    memcpy(local, (void*)spi2_rx_buf, SPI2_PKT_LEN);
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    switch (local[0]) {

    case SPI2_OP_DATA: {
        uint8_t crc = 0;
        for (int i = 0; i < 9; i++) crc ^= local[i];
        if (crc != local[9]) { cnt_error++; break; }
        TrajSample s;
        memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
        memcpy(&s.vel_cmd, &local[5], sizeof(int32_t));
        ring_push(&s);
        cnt_data++;
        break;
    }

    case SPI2_OP_TELEM_REQ:
        cnt_telem++;
        break;

    case SPI2_OP_BLOCK_HDR:
        first_sample_ready = 0;
        ring_reset();
        samples_consumed = 0;
        memset((void*)&telem_buf[1], 0, sizeof(TelemetryFrame));
        drive_request_enable();
        break;

    case SPI2_OP_READY_ACK:
        break;

    default:
        cnt_error++;
        break;
    }
}