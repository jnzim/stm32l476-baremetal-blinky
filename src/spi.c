// spi.c — SPI2 slave + DMA, STM32F411RE bare metal
//
// SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
// DMA1 Stream3 Ch0 = SPI2_RX   (periph → memory, circular, fires every 32 bytes)
// DMA1 Stream4 Ch0 = SPI2_TX   (memory → periph, re-armed on each CS falling edge)
//
// TX strategy: EXTI12 fires on CS falling edge, resets Stream4 to byte 0 of
//   telem_buf before Pi clocks out MISO. Ensures MISO always starts at byte 0.

#include "spi.h"
#include "ringBuffer.h"
#include "stm32f4xx.h"
#include <string.h>
#include "drive.h"
#include "loops.h"
#include "protocol.h"

// ── Packet length ─────────────────────────────────────────────────────────────
#define SPI2_PKT_LEN  SPI2_TRANSACTION_BYTES   // 32

// ── Telemetry double-buffer ───────────────────────────────────────────────────
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// ── RX buffer — circular DMA writes here continuously ────────────────────────
static volatile uint8_t spi2_rx_buf[SPI2_PKT_LEN];

// ── Diagnostic counters ───────────────────────────────────────────────────────
volatile uint32_t cnt_data  = 0;
volatile uint32_t cnt_telem = 0;
volatile uint32_t cnt_error = 0;
volatile uint32_t cnt_ovr   = 0;
volatile uint32_t cnt_cs    = 0;   // CS falling edge count

// ─────────────────────────────────────────────────────────────────────────────
// tx_dma_rearm — reset Stream4 to byte 0 of current telem slot
// Called from EXTI12 on CS falling edge
// ─────────────────────────────────────────────────────────────────────────────
static inline void tx_dma_rearm(void)
{
    // Disable Stream4
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);

    // Clear flags
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |
                  DMA_HIFCR_CTEIF4  | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    // Reset to byte 0 of current telem slot
    uint8_t slot = telem_write_idx ^ 1u;
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[slot];
    DMA1_Stream4->NDTR = SPI2_PKT_LEN;
    DMA1_Stream4->CR  |= DMA_SxCR_EN;
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_init
// ─────────────────────────────────────────────────────────────────────────────
void spi_init(void)
{
    // ── Clocks ────────────────────────────────────────────────────────────────
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR  |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR  |= RCC_APB2ENR_SYSCFGEN;

    // ── GPIO PB12–PB15: AF5 = SPI2 ───────────────────────────────────────────
    GPIOB->MODER   &= ~((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));
    GPIOB->MODER   |=  ((2u << 24) | (2u << 26) | (2u << 28) | (2u << 30));

    GPIOB->AFR[1]  &= ~((0xFu << 16) | (0xFu << 20) | (0xFu << 24) | (0xFu << 28));
    GPIOB->AFR[1]  |=  ((5u   << 16) | (5u   << 20) | (5u   << 24) | (5u   << 28));

    GPIOB->OSPEEDR |=  ((3u << 24) | (3u << 26) | (3u << 28) | (3u << 30));

    // ── EXTI12 — CS falling edge → rearm TX DMA ──────────────────────────────
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);   // EXTI12 → PB
    SYSCFG->EXTICR[3] |=  (1u   << 0);

    EXTI->FTSR |=  (1u << 12);   // falling edge trigger
    EXTI->RTSR &= ~(1u << 12);   // not rising
    EXTI->IMR  |=  (1u << 12);   // unmask

    NVIC_SetPriority(EXTI15_10_IRQn, 1);   // higher priority than DMA RX
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // ── SPI2: configure, do NOT enable yet ───────────────────────────────────
    SPI2->CR1 = 0;
    SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // ── DMA1 Stream4 Ch0: SPI2_TX ────────────────────────────────────────────
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

    // ── DMA1 Stream3 Ch0: SPI2_RX ────────────────────────────────────────────
    DMA1_Stream3->CR = 0;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);

    DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 |
                  DMA_LIFCR_CTEIF3  | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

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

    // ── NVIC RX ───────────────────────────────────────────────────────────────
    NVIC_SetPriority(DMA1_Stream3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    // ── Enable SPI last ───────────────────────────────────────────────────────
    SPI2->CR1 = SPI_CR1_SPE;
}

// ─────────────────────────────────────────────────────────────────────────────
// EXTI15_10_IRQHandler — CS falling edge
// ─────────────────────────────────────────────────────────────────────────────
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1u << 12)) {
        EXTI->PR = (1u << 12);   // clear pending
        tx_dma_rearm();
        cnt_cs++;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// crc8_xor
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA1_Stream3_IRQHandler — SPI2 RX complete
// ─────────────────────────────────────────────────────────────────────────────
void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;

    // ── OVR check ─────────────────────────────────────────────────────────────
    if (SPI2->SR & SPI_SR_OVR) {
        (void)SPI2->DR;
        (void)SPI2->SR;
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;
        cnt_ovr++;
        return;
    }

    uint8_t local[SPI2_PKT_LEN];
    memcpy(local, (void*)spi2_rx_buf, SPI2_PKT_LEN);
    DMA1->LIFCR = DMA_LIFCR_CTCIF3;

    switch (local[0])
    {
        case SPI2_OP_DATA:
        {
            if (crc8_xor(local, 9) != local[9]) { cnt_error++; break; }
            TrajSample s;
            memcpy(&s.pos_cmd, &local[1], sizeof(int32_t));
            memcpy(&s.vel_cmd, &local[5], sizeof(int32_t));
            ring_push(&s);
            cnt_data++;
            break;
        }

        case SPI2_OP_BLOCK_HDR:
        {
            if (crc8_xor(local, 3) != local[3]) { cnt_error++; break; }
            first_sample_ready = 0;
            ring_reset();
            samples_consumed = 0;
            memset((void*)&telem_buf[1], 0, sizeof(TelemetryFrame));
            drive_request_servo_on();
            break;
        }

        case SPI2_OP_OPEN_LOOP:
        {
            if (crc8_xor(local, 9) != local[9]) { cnt_error++; break; }
            float v_mag, d_theta;
            memcpy(&v_mag,   &local[1], sizeof(float));
            memcpy(&d_theta, &local[5], sizeof(float));
            drive_request_open_loop(v_mag, d_theta);
            break;
        }

        case SPI2_OP_STOP:
        {
            if (crc8_xor(local, 1) != local[1]) { cnt_error++; break; }
            drive_request_stop();
            break;
        }

        case SPI2_OP_TELEM_REQ:
            cnt_telem++;
            break;

        case SPI2_OP_READY_ACK:
            break;

        default:
            cnt_error++;
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_process — superloop stub
// ─────────────────────────────────────────────────────────────────────────────
void spi_process(void) {}

// ─────────────────────────────────────────────────────────────────────────────
// spi_update_telem
// ─────────────────────────────────────────────────────────────────────────────
void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;
    memcpy((void*)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[write_slot];
    telem_write_idx = write_slot;
}