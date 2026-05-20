// spi.c — SPI2 slave + DMA, STM32F446RE bare metal
//
// SPI2 pins: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
// DMA1 Stream3 Ch0 = SPI2_RX   (periph → memory, circular, fires every 32 bytes)
// DMA1 Stream4 Ch0 = SPI2_TX   (memory → periph, circular)
//
// RX strategy: circular DMA into spi2_rx_buf[32].
//   TCIF fires every 32 bytes. ISR snapshots buffer into local[], clears TCIF,
//   decodes from snapshot. Safe because DMA just reset its pointer to byte 0
//   and needs one SPI clock before writing — memcpy of 32 bytes at 180MHz
//   (~18 cycles, ~100ns) is well inside that window.
//
// TX strategy: circular DMA replays telem_buf[read_slot] continuously.
//   spi_update_telem() atomically swaps M0AR to the freshly written slot.
//   TX never needs an interrupt.

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
// telem_buf[0]: written by TIM1 ISR via spi_update_telem()
// telem_buf[1]: read by TX DMA (M0AR points here until swapped)
volatile TelemetryFrame telem_buf[2];
volatile uint8_t        telem_write_idx = 0;

// ── RX buffer — circular DMA writes here continuously ────────────────────────
static volatile uint8_t spi2_rx_buf[SPI2_PKT_LEN];

// ── Diagnostic counters (remove when stable) ─────────────────────────────────
volatile uint32_t cnt_data          = 0;
volatile uint32_t cnt_telem         = 0;
volatile uint32_t cnt_error         = 0;

// ─────────────────────────────────────────────────────────────────────────────
// spi_init
// ─────────────────────────────────────────────────────────────────────────────
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

    // ── SPI2: slave, Mode 0, 8-bit, SSM=1 SSI=1 (software NSS) ──────────────
    SPI2->CR1 = 0;
    SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;

    SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // ── DMA1 Stream4 Ch0: SPI2_TX — telem_buf[1] → SPI DR, circular ─────────
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

    // ── DMA1 Stream3 Ch0: SPI2_RX — circular, fires every 32 bytes ───────────
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

// ─────────────────────────────────────────────────────────────────────────────
// crc8_xor — XOR of bytes [0..len-1]
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t crc8_xor(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc ^= buf[i];
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// DMA1_Stream3_IRQHandler — SPI2 RX complete
//
// Data path: SPI2 DR → DMA1 Stream3 → spi2_rx_buf → ISR → ring buffer → SysTick
//
// Circular DMA restarts immediately after TCIF fires. Snapshot buffer before
// clearing TCIF — DMA just reset its pointer to byte 0 and needs one SPI clock
// before writing. memcpy of 32 bytes at 180MHz (~100ns) is well inside window.
// ─────────────────────────────────────────────────────────────────────────────
void DMA1_Stream3_IRQHandler(void)
{
    if (!(DMA1->LISR & DMA_LISR_TCIF3)) return;

    // Snapshot before clearing — DMA is at byte 0 of next revolution
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
            // v_mag and d_theta packed as float32 little-endian
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
void spi_process(void)
{
    // TODO: build TelemetryFrame from drive state, call spi_update_telem()
}

// ─────────────────────────────────────────────────────────────────────────────
// spi_update_telem — atomically swap TX DMA to freshly written telem slot
//
// Safe: M0AR write is 32-bit atomic on Cortex-M4. TX DMA reads M0AR only at
// the start of each revolution (NDTR wrap). Update while DMA is mid-packet
// so swap takes effect at next clean packet boundary.
// ─────────────────────────────────────────────────────────────────────────────
void spi_update_telem(const TelemetryFrame *frame)
{
    uint8_t write_slot = telem_write_idx ^ 1u;
    memcpy((void*)&telem_buf[write_slot], frame, sizeof(TelemetryFrame));
    DMA1_Stream4->M0AR = (uint32_t)&telem_buf[write_slot];
    telem_write_idx = write_slot;
}