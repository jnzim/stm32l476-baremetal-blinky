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
volatile uint32_t cnt_isr       = 0;
volatile uint32_t cnt_cs        = 0;
volatile uint32_t cnt_block_hdr = 0;

volatile uint8_t  ready_asserted = 0;

// RX double-buffered via DMA DBM.
static uint8_t rx_buf[2][SPI_FRAME_BYTES];

// Producer telemetry double-buffer. SysTick writes these.
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// Dedicated TX DMA buffer.
static uint8_t tx_dma_buf[SPI_FRAME_BYTES];

typedef char TelemetryFrame_must_be_32_bytes[
    (sizeof(TelemetryFrame) == SPI_FRAME_BYTES) ? 1 : -1
];

typedef char TrajSample_must_be_8_bytes[
    (sizeof(TrajSample) == 8u) ? 1 : -1
];

static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
    }
    return crc;
}

void spi_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PC13 — READY output to Pi (active-low, idle high)
    GPIOC->MODER   &= ~(3u << 26);
    GPIOC->MODER   |=  (1u << 26);
    GPIOC->OTYPER  &= ~(1u << 13);
    GPIOC->OSPEEDR &= ~(3u << 26);
    GPIOC->OSPEEDR |=  (1u << 26);
    GPIOC->PUPDR   &= ~(3u << 26);
    GPIOC->BSRR     =  (1u << 13);

    // PB12 NSS — plain GPIO input (Pi drives CS, EXTI12 watches edges)
    // PB13 SCK, PB14 MISO, PB15 MOSI — AF5
    GPIOB->MODER  &= ~((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));
    GPIOB->MODER  |=  ((2u << 26) | (2u << 28) | (2u << 30));

    GPIOB->AFR[1] &= ~((0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28));
    GPIOB->AFR[1] |=  ((5u << 20) | (5u << 24) | (5u << 28));

    GPIOB->OSPEEDR |= ((3u << 26) | (3u << 28) | (3u << 30));

    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (1u   << 0);

    EXTI->FTSR |= (1u << 12);
    EXTI->RTSR |= (1u << 12);
    EXTI->IMR  |= (1u << 12);

    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    SPI2->CR1 = SPI_CR1_SSM;
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    // RX DMA — DMA1 Stream3 Channel0, P->M, double-buffer
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
                  DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    DMA1_Stream3->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)rx_buf[0];
    DMA1_Stream3->M1AR = (uint32_t)rx_buf[1];
    DMA1_Stream3->NDTR = SPI_FRAME_BYTES;

    DMA1_Stream3->CR = (0u << DMA_SxCR_CHSEL_Pos) |
                       (0u << DMA_SxCR_DIR_Pos)   |
                       DMA_SxCR_MINC              |
                       DMA_SxCR_DBM               |
                       DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    // TX DMA — DMA1 Stream4 Channel0, M->P
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    DMA1->HIFCR = DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;

    DMA1_Stream4->PAR  = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)tx_dma_buf;
    DMA1_Stream4->NDTR = SPI_FRAME_BYTES;

    DMA1_Stream4->CR = (0u << DMA_SxCR_CHSEL_Pos) |
                       (1u << DMA_SxCR_DIR_Pos)   |
                       DMA_SxCR_MINC;

    SPI2->CR1 |= SPI_CR1_SPE;
}

static void spi_clear_ovr(void)
{
    volatile uint32_t tmp;
    tmp = SPI2->DR;
    tmp = SPI2->SR;
    (void)tmp;
}

static void tx_dma_rearm(void)
{
    uint8_t slot = telem_write_idx;

    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN) {}

    DMA1->HIFCR = DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4 |
                  DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CFEIF4;

    memcpy(tx_dma_buf, (const void *)&telem_buf[slot], SPI_FRAME_BYTES);

    __DMB();

    // ── Critical: preload byte 0 into DR before clocking starts ──────────
    // The SPI shift register needs to already hold byte 0 when SCK begins,
    // otherwise the first byte clocked out is whatever was left in DR from
    // the previous frame (off-by-one byte slide). DMA serves bytes 1..31.
    DMA1_Stream4->M0AR = (uint32_t)(tx_dma_buf + 1);
    DMA1_Stream4->NDTR = SPI_FRAME_BYTES - 1;

    SPI2->DR = tx_dma_buf[0];

    DMA1_Stream4->CR |= DMA_SxCR_EN;
}

static void rx_dma_reset_to_frame_boundary(void)
{
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN) {}

    DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
                  DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    spi_clear_ovr();

    DMA1_Stream3->M0AR = (uint32_t)rx_buf[0];
    DMA1_Stream3->M1AR = (uint32_t)rx_buf[1];
    DMA1_Stream3->NDTR = SPI_FRAME_BYTES;

    DMA1_Stream3->CR &= ~DMA_SxCR_CT;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 12)) {
        EXTI->PR = (1u << 12);

        if ((GPIOB->IDR & (1u << 12)) == 0u) {
            tx_dma_rearm();
            cnt_cs++;
        }
        // CS rising — temporarily disabled for bring-up
        // else {
        //     if (DMA1_Stream3->NDTR != SPI_FRAME_BYTES) {
        //         cnt_error++;
        //         rx_dma_reset_to_frame_boundary();
        //     }
        // }
    }
}

void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;

    memcpy((void *)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));

    __DMB();

    telem_write_idx = write_slot;
}

void DMA1_Stream3_IRQHandler(void)
{
    cnt_isr++;

    if (!(DMA1->LISR & DMA_LISR_TCIF3)) {
        return;
    }

    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    const uint8_t *done =
        (DMA1_Stream3->CR & DMA_SxCR_CT) ? rx_buf[0] : rx_buf[1];

    switch (done[0]) {
        case SPI2_OP_DATA:
            // DATA:
            // [0]   opcode
            // [1-4] int32 pos_cmd
            // [5-8] int32 vel_cmd
            // [9]   crc over bytes 0-8
            if (crc8_xor(done, 9) != done[9]) {
                cnt_error++;
                break;
            }

            {
                TrajSample s;
                memcpy(&s.pos_cmd, &done[1], sizeof(int32_t));
                memcpy(&s.vel_cmd, &done[5], sizeof(int32_t));
                ring_push(&s);
                cnt_data++;
            }

            // A2: clear READY on first DATA after assertion
            if (ready_asserted) {
                GPIOC->BSRR    = (1u << 13);
                ready_asserted = 0;
            }
            break;

        case SPI2_OP_BLOCK_HDR:
            if (crc8_xor(done, 3) != done[3]) {
                cnt_error++;
                break;
            }

            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            cnt_block_hdr++;

            // Force READY deasserted at start of new stream
            GPIOC->BSRR    = (1u << 13);
            ready_asserted = 0;

            drive_request_servo_on();
            break;

        case SPI2_OP_OPEN_LOOP:
            if (crc8_xor(done, 9) != done[9]) {
                cnt_error++;
                break;
            }

            {
                float v_mag;
                float d_theta;

                memcpy(&v_mag,   &done[1], sizeof(float));
                memcpy(&d_theta, &done[5], sizeof(float));

                drive_request_open_loop(v_mag, d_theta);
            }
            break;

        case SPI2_OP_STOP:
            if (crc8_xor(done, 1) != done[1]) {
                cnt_error++;
                break;
            }

            drive_request_stop();
            break;

        case SPI2_OP_TELEM_REQ:
        case SPI2_OP_READY_ACK:
        case SPI2_OP_NOP:
            break;

        default:
            cnt_error++;
            break;
    }
}