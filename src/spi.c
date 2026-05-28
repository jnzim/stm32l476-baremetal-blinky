// spi.c — SPI2 slave RX DMA, STM32F411 bare metal
//
// SPI2 pins:
//   PB12 = NSS / CS from Pi, AF5 hardware NSS
//   PB13 = SCK, AF5
//   PB14 = MISO, AF5
//   PB15 = MOSI, AF5
//
// DMA1 Stream3 Ch0 = SPI2_RX
// DMA1 Stream4 Ch0 = SPI2_TX, currently unused
//
// Current baseline:
//   Pi TX / MOSI -> STM SPI2 RX -> DMA -> local[] -> CRC -> ring_push()
//   MISO telemetry is intentionally disabled/deferred.

#include "spi.h"
#include "protocol.h"
#include "ringBuffer.h"
#include "drive.h"
#include "loops.h"
#include "stm32f4xx.h"

#include <string.h>
#include <stdint.h>

#define SPI_FRAME_BYTES SPI2_TRANSACTION_BYTES

volatile uint32_t cnt_data      = 0;
volatile uint32_t cnt_error     = 0;
volatile uint32_t cnt_telem     = 0;
volatile uint32_t cnt_block_hdr = 0;

volatile uint8_t ready_asserted = 0;

volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

static volatile uint8_t spi2_rx_buf[SPI_FRAME_BYTES];

typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI_FRAME_BYTES) ? 1 : -1
];

typedef char TrajSample_must_be_8_bytes[
    (sizeof(TrajSample) == 8u) ? 1 : -1
];

static void spi2_dma_clear_flags(void)
{
    // DMA1 Stream3 RX flags are in LIFCR.
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    // DMA1 Stream4 TX flags are in HIFCR.
    DMA1->HIFCR = DMA_HIFCR_CTCIF4  |
                  DMA_HIFCR_CHTIF4  |
                  DMA_HIFCR_CTEIF4  |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;
}

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // PC13 READY output, active-low, idle high.
    GPIOC->MODER   &= ~(3u << 26);
    GPIOC->MODER   |=  (1u << 26);
    GPIOC->OTYPER  &= ~(1u << 13);
    GPIOC->OSPEEDR &= ~(3u << 26);
    GPIOC->OSPEEDR |=  (1u << 26);
    GPIOC->PUPDR   &= ~(3u << 26);
    GPIOC->BSRR     =  (1u << 13);

    // PB12/PB13/PB14/PB15 all AF5 SPI2.
    GPIOB->MODER &= ~((3u << 24) |
                      (3u << 26) |
                      (3u << 28) |
                      (3u << 30));

    GPIOB->MODER |=  ((2u << 24) |   // PB12 NSS
                      (2u << 26) |   // PB13 SCK
                      (2u << 28) |   // PB14 MISO
                      (2u << 30));   // PB15 MOSI

    GPIOB->AFR[1] &= ~((0xFu << 16) |
                       (0xFu << 20) |
                       (0xFu << 24) |
                       (0xFu << 28));

    GPIOB->AFR[1] |=  ((5u << 16) |  // PB12 SPI2_NSS
                       (5u << 20) |  // PB13 SPI2_SCK
                       (5u << 24) |  // PB14 SPI2_MISO
                       (5u << 28));  // PB15 SPI2_MOSI

    GPIOB->OSPEEDR |= ((3u << 24) |
                       (3u << 26) |
                       (3u << 28) |
                       (3u << 30));

    // Pull NSS high at boot so slave is deselected if Pi is not driving yet.
    GPIOB->PUPDR &= ~(3u << 24);
    GPIOB->PUPDR |=  (1u << 24);

    // Disable SPI while configuring.
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;

    // Disable RX/TX DMA streams.
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    spi2_dma_clear_flags();

    // RX DMA only: SPI2->DR -> spi2_rx_buf.
    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buf;
    DMA1_Stream3->NDTR = SPI_FRAME_BYTES;

    DMA1_Stream3->CR =
        (0u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_MINC              |
        DMA_SxCR_CIRC              |
        DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    // SPI2 slave, Mode 1, hardware NSS.
    // CPOL = 0, CPHA = 1.
    // SSM = 0, MSTR = 0.
    SPI2->CR1 = SPI_CR1_CPHA;

    // RX DMA only. MISO/telemetry deferred.
    SPI2->CR2 = SPI_CR2_RXDMAEN;

    __DMB();

    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_update_telem(const TelemetryFrame *frame)
{
    memcpy((void *)&telem_buf[1], frame, sizeof(TelemetryFrame));
    __DMB();
    telem_write_idx = 1;
}

void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) {
        return;
    }

    uint8_t local[SPI_FRAME_BYTES];

    // Copy completed 32-byte RX frame before parsing.
    memcpy(local, (void *)spi2_rx_buf, SPI_FRAME_BYTES);

    // Clear RX DMA flags.
    DMA1->LIFCR = DMA_LIFCR_CTCIF3  |
                  DMA_LIFCR_CHTIF3  |
                  DMA_LIFCR_CTEIF3  |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    uint8_t crc = 0;

    switch (local[0]) {
        case SPI2_OP_DATA:
            for (int i = 0; i < 9; i++) {
                crc ^= local[i];
            }

            if (crc != local[9]) {
                cnt_error++;
                break;
            }

            {
                TrajSample s;
                memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
                memcpy(&s.vel_cmd, &local[5], sizeof(int32_t));

                ring_push(&s);
                cnt_data++;
            }

            if (ready_asserted) {
                GPIOC->BSRR    = (1u << 13);
                ready_asserted = 0;
            }

            break;

        case SPI2_OP_BLOCK_HDR:
            for (int i = 0; i < 3; i++) {
                crc ^= local[i];
            }

            if (crc != local[3]) {
                cnt_error++;
                break;
            }

            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            cnt_block_hdr++;

            GPIOC->BSRR    = (1u << 13);
            ready_asserted = 0;

            drive_request_servo_on();
            break;

        case SPI2_OP_TELEM_REQ:
            cnt_telem++;
            break;

        case SPI2_OP_READY_ACK:
        case SPI2_OP_NOP:
            break;

        default:
            cnt_error++;
            break;
    }
}